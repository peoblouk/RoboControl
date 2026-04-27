// ===============================
// file_manager.c
// ===============================

#include "file_manager.h"

static const char *FM_BASE = FILE_STORAGE_PATH;
static const char *FILE_PREFIX = HTTP_FILE_PREFIX;

static bool ext_allowed_(const char *name) {
    const char *dot = strrchr(name, '.');
    if (!dot) {
        return false;
    }
    return (strcasecmp(dot, ".txt") == 0 || strcasecmp(dot, ".gcode") == 0);
}

static int hex2int_(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

static void url_decode_(char *dst, const char *src) {
    size_t di = 0;
    for (size_t si = 0; src[si]; ++si) {
        if (src[si] == '%' && src[si + 1] && src[si + 2]) {
            int hi = hex2int_(src[si + 1]);
            int lo = hex2int_(src[si + 2]);
            if (hi >= 0 && lo >= 0) {
                dst[di++] = (char)((hi << 4) | lo);
                si += 2;
                continue;
            }
        } else if (src[si] == '+') {
            dst[di++] = ' ';
            continue;
        }
        dst[di++] = src[si];
    }
    dst[di] = '\0';
}

static bool make_path_from_tail_(const char *uri_tail, char *out, size_t out_sz) {
    if (!uri_tail || !*uri_tail) {
        return false;
    }
    char name[192];
    url_decode_(name, uri_tail);
    if (strstr(name, "..") || strchr(name, '/') || strchr(name, '\\') || name[0] == 0) {
        return false;
    }
    if (!ext_allowed_(name)) {
        return false;
    }
    int n = snprintf(out, out_sz, "%s/%s", FM_BASE, name);
    return n > 0 && (size_t)n < out_sz;
}

static esp_err_t files_list_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");

    DIR *dir = opendir(FM_BASE);
    if (!dir) {
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    httpd_resp_sendstr_chunk(req, "[");
    bool first = true;

    struct dirent *ent;
    struct stat st;
    char path[512];

    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_name[0] == '.') {
            continue;
        }
        if (!ext_allowed_(ent->d_name)) {
            continue;
        }

        int len = snprintf(path, sizeof(path), "%s/%s", FM_BASE, ent->d_name);
        if (len < 0 || (size_t)len >= sizeof(path)) {
            continue;
        }

        if (stat(path, &st) == 0 && S_ISREG(st.st_mode)) {
            char item[256];
            int w = snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"size\":%ld}",
                             first ? "" : ",", ent->d_name, (long)st.st_size);
            httpd_resp_send_chunk(req, item, w);
            first = false;
        }
    }

    closedir(dir);
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t file_any_handler(httpd_req_t *req) {
    if (strncmp(req->uri, FILE_PREFIX, strlen(FILE_PREFIX)) != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad uri");
    }

    const char *tail = req->uri + strlen(FILE_PREFIX);
    char path[256];
    if (!make_path_from_tail_(tail, path, sizeof(path))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid name");
    }

    if (req->method == HTTP_GET) {
        FILE *f = fopen(path, "rb");
        if (!f) {
            return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        }
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        char buf[1024];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
            if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
                fclose(f);
                httpd_resp_sendstr_chunk(req, NULL);
                return ESP_FAIL;
            }
        }
        fclose(f);
        return httpd_resp_sendstr_chunk(req, NULL);
    }

    if (req->method == HTTP_DELETE) {
        if (remove(path) == 0) {
            httpd_resp_sendstr(req, "OK");
            return ESP_OK;
        }
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "remove failed");
    }

    if (req->method == HTTP_PUT) {
        FILE *f = fopen(path, "wb");
        if (!f) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");
        }

        int remaining = req->content_len;
        char buf[1024];
        while (remaining > 0) {
            int to_read = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
            int r = httpd_req_recv(req, buf, to_read);
            if (r <= 0) {
                fclose(f);
                return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            }
            if (fwrite(buf, 1, r, f) != (size_t)r) {
                fclose(f);
                return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
            }
            remaining -= r;
        }
        fclose(f);
        httpd_resp_sendstr(req, "OK");
        return ESP_OK;
    }

    return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "method");
}

esp_err_t file_manager_register(httpd_handle_t server) {
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_uri_t uris[] = {
        {"/files", HTTP_GET, files_list_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/file/*", HTTP_ANY, file_any_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
    };

    for (int i = 0; i < (int)(sizeof(uris) / sizeof(uris[0])); i++) {
        esp_err_t err = httpd_register_uri_handler(server, &uris[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}
