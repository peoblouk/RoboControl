// ===============================
// http_handlers.c
// ===============================

#include "http_handlers.h"

static esp_err_t style_get_handler(httpd_req_t *req) {
    FILE *f = fopen(FS_WEB_BASE "/style.css", "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/css");
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, r);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t app_js_get_handler(httpd_req_t *req) {
    FILE *f = fopen(FS_WEB_BASE "/app.js", "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/javascript");
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, r);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    FILE *f = fopen(FS_WEB_BASE "/spage.html", "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/html");
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, r);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t settings_get_handler(httpd_req_t *req) {
    FILE *f = fopen(FS_WEB_BASE "/settings.html", "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, r);
    }
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

static esp_err_t icon_get_handler(httpd_req_t *req) {
    FILE *f = fopen(FS_WEB_BASE "/robocontrol.ico", "rb");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=2592000");

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
    char buf[128];
    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = remaining > (int)(sizeof(buf) - 1) ? (int)(sizeof(buf) - 1) : remaining;
        int received = httpd_req_recv(req, buf, to_read);
        if (received <= 0) {
            return ESP_FAIL;
        }
        remaining -= received;
        buf[received] = '\0';
    }

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *ssid_json = cJSON_GetObjectItemCaseSensitive(json, "ssid");
    cJSON *pass_json = cJSON_GetObjectItemCaseSensitive(json, "password");
    if (cJSON_IsString(ssid_json) && cJSON_IsString(pass_json)) {
        save_wifi_config(ssid_json->valuestring, pass_json->valuestring);
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_ERROR_CHECK(esp_wifi_deinit());
        wifi_init_softap();
        httpd_resp_sendstr(req, "WiFi config saved!");
    } else {
        httpd_resp_send_500(req);
    }
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t limits_get_handler(httpd_req_t *req) {
    char resp[512];
    int n = snprintf(resp, sizeof(resp),
                     "{"
                     "\"joints\":["
                     "{\"id\":0,\"name\":\"J0\",\"min\":%.2f,\"max\":%.2f,\"v\":%.2f},"
                     "{\"id\":1,\"name\":\"J1\",\"min\":%.2f,\"max\":%.2f,\"v\":%.2f},"
                     "{\"id\":2,\"name\":\"J2\",\"min\":%.2f,\"max\":%.2f,\"v\":%.2f},"
                     "{\"id\":3,\"name\":\"J3\",\"min\":%.2f,\"max\":%.2f,\"v\":%.2f},"
                     "{\"id\":4,\"name\":\"J4\",\"min\":%.2f,\"max\":%.2f,\"v\":%.2f},"
                     "{\"id\":5,\"name\":\"J5\",\"min\":%.2f,\"max\":%.2f,\"v\":%.2f}"
                     "]"
                     "}",
                     (double)J0_MIN, (double)J0_MAX, (double)J0_V,
                     (double)J1_MIN, (double)J1_MAX, (double)J1_V,
                     (double)J2_MIN, (double)J2_MAX, (double)J2_V,
                     (double)J3_MIN, (double)J3_MAX, (double)J3_V,
                     (double)J4_MIN, (double)J4_MAX, (double)J4_V,
                     (double)J5_MIN, (double)J5_MAX, (double)J5_V);

    if (n <= 0) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, strlen(resp));
}

esp_err_t http_handlers_register(httpd_handle_t server) {
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_uri_t uris[] = {
        {"/", HTTP_GET, root_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/settings", HTTP_GET, settings_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/web/style.css", HTTP_GET, style_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/web/app.js", HTTP_GET, app_js_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/web/robocontrol.ico", HTTP_GET, icon_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/favicon.ico", HTTP_GET, icon_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/wifi_reset", HTTP_POST, wifi_reset_post_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/wifi_config", HTTP_POST, wifi_config_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
        {"/api/limits", HTTP_GET, limits_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false},
    };

    for (int i = 0; i < (int)(sizeof(uris) / sizeof(uris[0])); i++) {
        esp_err_t err = httpd_register_uri_handler(server, &uris[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}
