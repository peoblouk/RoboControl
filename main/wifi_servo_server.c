// ===============================
// wifi_servo_server.c
// ===============================

#include "wifi_servo_server.h"

// ===============================
// GLOBAL CONFIG
// ===============================
static const char *TAG = "wifi_server";

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
// G-CODE FILE HANDLER
// ===============================
static void print_gcode_to_console(const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (!file) {
        ESP_LOGE(TAG, "Cannot open file for reading: %s", filename);
        return;
    }
    
    printf("\n=== G-CODE CONTENT ===\n");
    char line[128];
    int line_number = 1;
    
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\r\n")] = 0;
        printf("%4d: %s\n", line_number, line);
        line_number++;
    }
    
    printf("=== END OF G-CODE ===\n\n");
    fclose(file);
}

static esp_err_t upload_post_handler(httpd_req_t *req) {
    char filepath[50];
    snprintf(filepath, sizeof(filepath), "/spiffs/data/gcode_file.gcode");

    FILE *fd = fopen(filepath, "w");
    if (!fd) { httpd_resp_send_500(req); return ESP_FAIL; }

    char buf[256];
    int received;
    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (fwrite(buf, 1, received, fd) != received) {
            fclose(fd);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    fclose(fd);
    ESP_LOGI(TAG, "G-code saved: %s", filepath);
    httpd_resp_sendstr(req, "File uploaded successfully!");
    print_gcode_to_console("/spiffs/data/gcode_file.gcode");
    return ESP_OK;
}

// ===============================
// WEB SERVER HANDLERS
// ===============================
static esp_err_t style_get_handler(httpd_req_t *req) {
    FILE* f = fopen("/spiffs/web/style.css", "r");
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
    FILE* f = fopen("/spiffs/web/spage.html", "r");
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
    FILE* f = fopen("/spiffs/web/settings.html", "r");
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

static esp_err_t wifi_config_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        httpd_resp_sendstr(req, "Use POST to configure WiFi.");
        return ESP_OK;
    } else if (req->method == HTTP_POST) {
        char buf[128];
        int remaining = req->content_len;
        while (remaining > 0) {
            int received = httpd_req_recv(req, buf, remaining > sizeof(buf) ? sizeof(buf) : remaining);
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
    // HANDSHAKE
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_clients_add(fd);
        ws_send_to(fd, "{\"status\":\"connected\"}");
        return ESP_OK;
    }

    // RECEIVE FRAME
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

    // PARSE JSON
    cJSON *json = cJSON_Parse((char*)frame.payload);
    if (json) {
        // 1) SERVO CONTROL
        cJSON *servo = cJSON_GetObjectItem(json, "servo");
        cJSON *angle = cJSON_GetObjectItem(json, "angle");
        if (cJSON_IsNumber(servo) && cJSON_IsNumber(angle)) {
            servo_set_angle(servo->valueint, (float)angle->valuedouble);
            ws_send_to(httpd_req_to_sockfd(req), "{\"status\":\"ok\",\"cmd\":\"servo\"}");
        }

        // 2) SENSORS ON-DEMAND
        cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
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

        // 3) MOVE XYZ
        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring,"move_xyz")==0) {
            cJSON *jx = cJSON_GetObjectItem(json, "x");
            cJSON *jy = cJSON_GetObjectItem(json, "y");
            cJSON *jz = cJSON_GetObjectItem(json, "z");
            if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy) && cJSON_IsNumber(jz)) {
                float x = jx->valuedouble;
                float y = jy->valuedouble;
                float z = jz->valuedouble;
                float q_target[SERVO_COUNT];
                inverse_kinematics(x,y,z,q_target);
                move_to_position(q_target);
                ws_send_to(httpd_req_to_sockfd(req), "{\"status\":\"ok\",\"cmd\":\"move_xyz\"}");
            }
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
}

// ===============================
// WEBSERVER INIT
// ===============================
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24;
    if (httpd_start(&g_httpd,&config)!=ESP_OK) return NULL;

    httpd_uri_t uris[] = {
    { "/", HTTP_GET, root_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/status", HTTP_GET, status_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/settings", HTTP_GET, settings_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/web/style.css", HTTP_GET, style_get_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/upload", HTTP_POST, upload_post_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/wifi_reset", HTTP_POST, wifi_reset_post_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/wifi_config", HTTP_ANY, wifi_config_handler, NULL, .is_websocket = false, .handle_ws_control_frames = false },
    { "/ws", HTTP_GET, ws_handler, NULL, .is_websocket = true, .handle_ws_control_frames = true }
    };

    for (int i=0; i<sizeof(uris)/sizeof(uris[0]); i++)
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
    xTaskCreate(ws_task_sensors,"ws_sensors",4096,NULL,5,NULL);
}