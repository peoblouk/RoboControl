// ===============================
// wifi_server.c
// ===============================

#include "wifi_server.h"

// ===============================
// GLOBAL CONFIG
// ===============================
static const char *TAG = "wifi_server";

static const char *FM_BASE = FILE_STORAGE_PATH;      // = FS_DATA_BASE
static const char *FILE_PREFIX = HTTP_FILE_PREFIX;   // = "/file/"

my_wifi_config_t wifi_cfg = {
    .ssid = WIFI_SSID,
    .pass = WIFI_PASS
};

static httpd_handle_t g_httpd = NULL;
#define WS_MAX_CLIENTS 4
static int g_ws_clients[WS_MAX_CLIENTS];
static SemaphoreHandle_t g_ws_lock;

#define WS_SENSORS_PERIOD_MS 200

// ===============================
// WEB SERVER HANDLERS (static files)
// ===============================
static esp_err_t style_get_handler(httpd_req_t *req) {
    FILE* f = fopen(FS_WEB_BASE "/style.css", "r");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
    httpd_resp_set_type(req, "text/css");
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, r);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    FILE* f = fopen(FS_WEB_BASE "/spage.html", "r");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
    httpd_resp_set_type(req, "text/html");
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, r);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t settings_get_handler(httpd_req_t *req) {
    FILE* f = fopen(FS_WEB_BASE "/settings.html", "r");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, r);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t wifi_reset_post_handler(httpd_req_t *req) {
    erase_wifi_nvs();
    httpd_resp_sendstr(req, "WiFi reset, restarting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    wifi_sta_list_t sta_list;
    bool online = false;
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK && sta_list.num > 0)
        online = true;

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"online\":%s}", online ? "true":"false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t icon_get_handler(httpd_req_t *req) {
    FILE *f = fopen(FS_WEB_BASE "/robocontrol.ico", "rb");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=2592000"); // cache

    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, r) != ESP_OK) {
            fclose(f);
            httpd_resp_sendstr_chunk(req, NULL);
            return ESP_FAIL;
        }
    }
    fclose(f);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t wifi_config_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        httpd_resp_sendstr(req, "Use POST to configure WiFi.");
        return ESP_OK;
    } else if (req->method == HTTP_POST) {
        char buf[128];
        int remaining = req->content_len;
        while (remaining > 0) {
            int received = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
            if (received <= 0) return ESP_FAIL;
            remaining -= received;
            buf[received] = '\0';
        }
        cJSON *json = cJSON_Parse(buf);
        if (!json) { httpd_resp_send_500(req); return ESP_FAIL; }
        cJSON *ssid_json = cJSON_GetObjectItemCaseSensitive(json, "ssid");
        cJSON *pass_json = cJSON_GetObjectItemCaseSensitive(json, "password");
        if (cJSON_IsString(ssid_json) && cJSON_IsString(pass_json)) {
            save_wifi_config(ssid_json->valuestring, pass_json->valuestring);
            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_deinit());
            wifi_init_softap();
            httpd_resp_sendstr(req, "WiFi config saved!");
        } else httpd_resp_send_500(req);
        cJSON_Delete(json);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// ===============================
// WEBSOCKET
// ===============================
static void ws_clients_add(int fd) {
    xSemaphoreTake(g_ws_lock, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; i++)
        if (g_ws_clients[i] < 0) { g_ws_clients[i] = fd; break; }
    xSemaphoreGive(g_ws_lock);
}

static void ws_clients_remove(int fd) {
    xSemaphoreTake(g_ws_lock, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; i++)
        if (g_ws_clients[i] == fd) { g_ws_clients[i] = -1; break; }
    xSemaphoreGive(g_ws_lock);
}

static void ws_send_to(int fd, const char *msg) {
    if (!g_httpd) return;
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t*)msg,
        .len = strlen(msg)
    };
    httpd_ws_send_frame_async(g_httpd, fd, &frame);
}

static void ws_broadcast(const char *msg) {
    xSemaphoreTake(g_ws_lock, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; i++)
        if (g_ws_clients[i] >= 0) ws_send_to(g_ws_clients[i], msg);
    xSemaphoreGive(g_ws_lock);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_clients_add(fd);
        ws_send_to(fd, "{\"status\":\"connected\"}");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK || frame.len == 0) return ESP_FAIL;

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        int fd = httpd_req_to_sockfd(req);
        ws_clients_remove(fd);
        return ESP_OK;
    }

    if (frame.len == 0) return ESP_FAIL;

    frame.payload = malloc(frame.len + 1);
    if (!frame.payload) return ESP_ERR_NO_MEM;

    httpd_ws_recv_frame(req, &frame, frame.len);
    frame.payload[frame.len] = '\0';

    cJSON *json = cJSON_Parse((char*)frame.payload);
    if (json) {
        cJSON *cmd = cJSON_GetObjectItem(json, "cmd");

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "joint_set") == 0) {
            cJSON *jid = cJSON_GetObjectItem(json, "id");
            cJSON *angle = cJSON_GetObjectItem(json, "angle");
            if (cJSON_IsNumber(jid) && cJSON_IsNumber(angle)) {
                joint_set_angle(jid->valueint, (float)angle->valuedouble);
                ws_send_to(httpd_req_to_sockfd(req), "{\"status\":\"ok\",\"cmd\":\"joint_set\"}");
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring,"sensors")==0) {
            cJSON *root = cJSON_CreateObject();
            cJSON *arr  = cJSON_AddArrayToObject(root,"sensors");
            for (int i=0; i<SENSOR_COUNT; i++) {
                cJSON *o = cJSON_CreateObject();
                cJSON_AddNumberToObject(o,"id",i);
                cJSON_AddNumberToObject(o,"angle",sensor_read_angle(i));
                cJSON_AddItemToArray(arr,o);
            }
            char *out = cJSON_PrintUnformatted(root);
            ws_send_to(httpd_req_to_sockfd(req), out);
            free(out);
            cJSON_Delete(root);
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring,"move_xyz") == 0) {
            cJSON *jx = cJSON_GetObjectItem(json, "x");
            cJSON *jy = cJSON_GetObjectItem(json, "y");
            cJSON *jz = cJSON_GetObjectItem(json, "z");

            if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy) && cJSON_IsNumber(jz)) {
                float x = (float)jx->valuedouble;
                float y = (float)jy->valuedouble;
                float z = (float)jz->valuedouble;

                bool ok = robot_cmd_move_xyz(x, y, z);
                int fd  = httpd_req_to_sockfd(req);

                if (ok) {
                    ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"move_xyz\",\"queued\":true}");
                } else {
                    ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"move_xyz\",\"reason\":\"queue_full_or_not_started\"}");
                }
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring,"gcode_line") == 0) {
            cJSON *jl = cJSON_GetObjectItem(json, "line");
            int fd = httpd_req_to_sockfd(req);

            if (cJSON_IsString(jl)) {
                bool ok = gcode_push_line(jl->valuestring);
                ws_send_to(fd, ok ? "{\"status\":\"ok\",\"cmd\":\"gcode_line\"}"
                                : "{\"status\":\"err\",\"cmd\":\"gcode_line\"}");
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "run_gcode") == 0) {
            cJSON *jf = cJSON_GetObjectItem(json, "filename");
            int fd = httpd_req_to_sockfd(req);

            if (cJSON_IsString(jf)) {
                robot_core_run_gcode(jf->valuestring);
                ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"run_gcode\",\"state\":\"started\"}");
            } else {
                ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"run_gcode\",\"msg\":\"no_filename\"}");
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring,"gcode_stop") == 0) {
            int fd = httpd_req_to_sockfd(req);
            gcode_stop();
            ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"gcode_stop\"}");
        }

        cJSON_Delete(json);
    }

    free(frame.payload);
    return ESP_OK;
}

static void ws_task_sensors(void *arg) {
    for (;;) {
        cJSON *root = cJSON_CreateObject();
        cJSON *arr  = cJSON_AddArrayToObject(root,"sensors");
        for (int i=0;i<SENSOR_COUNT;i++) {
            cJSON *o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o,"id",i);
            cJSON_AddNumberToObject(o,"angle",sensor_read_angle(i));
            cJSON_AddItemToArray(arr,o);
        }
        char *out = cJSON_PrintUnformatted(root);
        ws_broadcast(out);
        free(out);
        cJSON_Delete(root);
        vTaskDelay(pdMS_TO_TICKS(WS_SENSORS_PERIOD_MS));
    }
}

// ===============================
// FILE SYSTEM MANAGER (TXT + GCODE)
// ===============================

static bool ext_allowed_(const char *name) {
    const char *dot = strrchr(name, '.');
    if (!dot) return false;
    return (strcasecmp(dot, ".txt") == 0 || strcasecmp(dot, ".gcode") == 0);
}

static int hex2int_(char c){
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void url_decode_(char *dst, const char *src){
    size_t di = 0;
    for (size_t si = 0; src[si]; ++si) {
        if (src[si] == '%' && src[si+1] && src[si+2]) {
            int hi = hex2int_(src[si+1]);
            int lo = hex2int_(src[si+2]);
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

static bool make_path_from_tail_(const char *uri_tail, char *out, size_t out_sz){
    if (!uri_tail || !*uri_tail) return false;
    char name[192];
    url_decode_(name, uri_tail);
    if (strstr(name, "..") || strchr(name, '/') || strchr(name, '\\') || name[0]==0) return false;
    if (!ext_allowed_(name)) return false;
    int n = snprintf(out, out_sz, "%s/%s", FM_BASE, name);
    return n>0 && (size_t)n<out_sz;
}

static esp_err_t files_list_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");

    DIR *dir = opendir(FM_BASE);
    if (!dir) { httpd_resp_sendstr(req, "[]"); return ESP_OK; }

    httpd_resp_sendstr_chunk(req, "[");
    bool first = true;

    struct dirent *ent;
    struct stat st;
    char path[512];

    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_name[0] == '.')        continue;
        if (!ext_allowed_(ent->d_name))   continue;

        int len = snprintf(path, sizeof(path), "%s/%s", FM_BASE, ent->d_name);
        if (len < 0 || (size_t)len >= sizeof(path)) continue;

        if (stat(path, &st) == 0 && S_ISREG(st.st_mode)) {
            char item[256];
            int w = snprintf(item, sizeof(item),
                            "%s{\"name\":\"%s\",\"size\":%ld}",
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
    if (strncmp(req->uri, FILE_PREFIX, strlen(FILE_PREFIX)) != 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad uri");

    const char *tail = req->uri + strlen(FILE_PREFIX);
    char path[256];
    if (!make_path_from_tail_(tail, path, sizeof(path)))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid name");

    if (req->method == HTTP_GET) { // READ
        FILE *f = fopen(path, "rb");
        if (!f) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        char buf[1024]; size_t n;
        while ((n=fread(buf,1,sizeof(buf),f))>0) {
            if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) { fclose(f); httpd_resp_sendstr_chunk(req,NULL); return ESP_FAIL; }
        }
        fclose(f);
        return httpd_resp_sendstr_chunk(req, NULL);
    }

    if (req->method == HTTP_DELETE) {  // DELETE
        if (remove(path)==0) { httpd_resp_sendstr(req, "OK"); return ESP_OK; }
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "remove failed");
    }

    if (req->method == HTTP_PUT) { // WRITE
        FILE *f = fopen(path, "wb");
        if (!f) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");

        int remaining = req->content_len;
        char buf[1024];
        while (remaining > 0) {
            int to_read = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
            int r = httpd_req_recv(req, buf, to_read);
            if (r <= 0) { fclose(f); return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed"); }
            if (fwrite(buf,1,r,f)!=(size_t)r){ fclose(f); return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed"); }
            remaining -= r;
        }
        fclose(f);
        httpd_resp_sendstr(req, "OK");

        return ESP_OK;
    }

    return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "method");
}

static esp_err_t upload_post_handler(httpd_req_t *req) {
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "%s/%s", FM_BASE, "gcode_file.gcode");

    FILE *fd = fopen(filepath, "wb");
    if (!fd) { httpd_resp_send_500(req); return ESP_FAIL; }

    char buf[256];
    int received;
    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (fwrite(buf, 1, received, fd) != (size_t)received) {
            fclose(fd);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    fclose(fd);
    ESP_LOGI(TAG, "G-code saved: %s", filepath);
    httpd_resp_sendstr(req, "File uploaded successfully!");
    return ESP_OK;
}

// ===============================
// NVS WIFI CONFIG
// ===============================
void save_wifi_config(const char *ssid, const char *pass) {
    nvs_handle_t nvs;
    if (nvs_open("wifi", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs,"ssid",ssid);
        nvs_set_str(nvs,"pass",pass);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

bool load_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    nvs_handle_t nvs;
    if (nvs_open("wifi", NVS_READONLY, &nvs) != ESP_OK) return false;
    size_t s_len = ssid_len, p_len = pass_len;
    if (nvs_get_str(nvs,"ssid",ssid,&s_len)!=ESP_OK) { nvs_close(nvs); return false; }
    if (nvs_get_str(nvs,"pass",pass,&p_len)!=ESP_OK) { nvs_close(nvs); return false; }
    nvs_close(nvs);
    return true;
}

void erase_wifi_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_OK || err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

// ===============================
// mDNS
// =============================== 
static void start_mdns(void)
{
    esp_err_t err = mdns_init();

    if (err == ESP_ERR_INVALID_STATE) {
        mdns_free();
        ESP_ERROR_CHECK(mdns_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(mdns_hostname_set("robo-control"));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP Robo Control"));

    mdns_service_remove_all();
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
}


// ===============================
// WIFI SOFTAP INIT
// ===============================
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {0};
    char ssid[32], pass[64];
    if (load_wifi_config(ssid,sizeof(ssid),pass,sizeof(pass))) {
        strncpy((char*)wifi_config.ap.ssid,ssid,sizeof(wifi_config.ap.ssid));
        strncpy((char*)wifi_config.ap.password,pass,sizeof(wifi_config.ap.password));
    } else {
        strncpy((char*)wifi_config.ap.ssid,WIFI_SSID,sizeof(wifi_config.ap.ssid));
        strncpy((char*)wifi_config.ap.password,WIFI_PASS,sizeof(wifi_config.ap.password));
    }
    wifi_config.ap.ssid_len = strlen((char*)wifi_config.ap.ssid);
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = strlen((char*)wifi_config.ap.password) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_start());
    start_mdns();
}

// ===============================
// WEBSERVER INIT
// ===============================
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24;
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&g_httpd,&config)!=ESP_OK) return NULL;

    httpd_uri_t uris[] = {
        { "/", HTTP_GET, root_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/status", HTTP_GET, status_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/settings", HTTP_GET, settings_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/web/style.css", HTTP_GET, style_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/web/robocontrol.ico", HTTP_GET, icon_get_handler, NULL, .is_websocket=false, .handle_ws_control_frames=false },
        { "/favicon.ico",               HTTP_GET, icon_get_handler, NULL, .is_websocket=false, .handle_ws_control_frames=false },
        { "/wifi_reset", HTTP_POST, wifi_reset_post_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/wifi_config", HTTP_ANY, wifi_config_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/ws", HTTP_GET, ws_handler, NULL, .is_websocket = true, .handle_ws_control_frames = true },

        { "/upload", HTTP_POST, upload_post_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/files",  HTTP_GET,  files_list_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
        { "/file/*", HTTP_ANY,  file_any_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    };

    for (int i=0; i<(int)(sizeof(uris)/sizeof(uris[0])); i++)
        httpd_register_uri_handler(g_httpd, &uris[i]);

    return g_httpd;
}

// ===============================
// WEBSERVER START
// ===============================
void wifi_servo_server_start(void) {
    g_ws_lock = xSemaphoreCreateMutex();
    for (int i=0;i<WS_MAX_CLIENTS;i++) g_ws_clients[i] = -1;

    wifi_init_softap();
    start_webserver();
    xTaskCreatePinnedToCore(ws_task_sensors, "ws_sensors", 4096, NULL, 5, NULL, CORE_ROBOT);
}