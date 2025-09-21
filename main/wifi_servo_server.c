// ===============================
// wifi_servo_server.c
// ===============================

#include "wifi_servo_server.h"

static const char *TAG = "wifi_server";
// ===============================
// GLOBAL CONFIG
// ===============================
my_wifi_config_t wifi_cfg = {
    .ssid = WIFI_SSID,
    .pass = WIFI_PASS
};

// ===============================
// SERVO CONTROL - FOR ALL
// ===============================
static esp_err_t servo_post_handler(httpd_req_t *req)
{
    char buf[128];
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[received] = '\0';

    // Parse JSON data
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *servo_id_json = cJSON_GetObjectItemCaseSensitive(json, "servo_id");
    cJSON *angle_json = cJSON_GetObjectItemCaseSensitive(json, "angle");

    if (cJSON_IsNumber(servo_id_json) && cJSON_IsNumber(angle_json)) { 
        int servo_id = servo_id_json->valueint;
        float angle = angle_json->valuedouble;

        servo_set_angle(servo_id, angle);

        httpd_resp_sendstr(req, "Servo moved");
    }

    cJSON_Delete(json);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// HOT FIX :D
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

esp_err_t move_xyz_post_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf)-1));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *x_json = cJSON_GetObjectItem(json, "x");
    cJSON *y_json = cJSON_GetObjectItem(json, "y");
    cJSON *z_json = cJSON_GetObjectItem(json, "z");

    if (cJSON_IsNumber(x_json) && cJSON_IsNumber(y_json) && cJSON_IsNumber(z_json)) {
        float x = (float)x_json->valuedouble;
        float y = (float)y_json->valuedouble;
        float z = (float)z_json->valuedouble;

        ESP_LOGI("MOVE_XYZ", "Target position: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

        // TODO: Implement inverse kinematics to get servo angles
        float q_target[SERVO_COUNT];

        //inverse_kinematics(x, y, z, q_target);
        move_to_position(q_target);

        httpd_resp_sendstr(req, "Move command accepted");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing parameters");
    }

    cJSON_Delete(json);
    return ESP_OK;
}

// ===============================
// SENSOR DATA - FOR ALL
// ===============================
static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    cJSON *sensors_array = cJSON_AddArrayToObject(json, "sensors");

    for (int i = 0; i < SENSOR_COUNT; i++) {
        cJSON *sensor_obj = cJSON_CreateObject();
        int raw_value = sensor_read_raw(i);
        float angle = sensor_read_angle(i);

        cJSON_AddNumberToObject(sensor_obj, "id", i);
        cJSON_AddNumberToObject(sensor_obj, "raw", raw_value);
        cJSON_AddNumberToObject(sensor_obj, "angle", angle);

        cJSON_AddItemToArray(sensors_array, sensor_obj);
    }

    char *json_str = cJSON_PrintUnformatted(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

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

static esp_err_t upload_post_handler(httpd_req_t *req)
{
    char filepath[50];
    snprintf(filepath, sizeof(filepath), "/spiffs/gcode_file.gcode");
    
    FILE *fd = fopen(filepath, "w");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to create file: %s", filepath);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char buf[256];
    int received;
    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (fwrite(buf, 1, received, fd) != received) {
            fclose(fd);
            ESP_LOGE(TAG, "File write failed");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    
    fclose(fd);
    ESP_LOGI(TAG, "G-code file saved: %s", filepath);
    print_gcode_to_console(filepath);

    httpd_resp_sendstr(req, "File uploaded successfully!");
    return ESP_OK;
}

// ===============================
// WEB SERVER HANDLERS
// ===============================
static esp_err_t style_get_handler(httpd_req_t *req)
{
    FILE* f = fopen("/spiffs/web/style.css", "r");
    if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "text/css");
    char buf[512];
    size_t read_bytes;
    while ((read_bytes = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, read_bytes);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE* f = fopen("/spiffs/web/spage.html", "r");
    if (!f) { ESP_LOGE(TAG, "Cannot open spage.html"); httpd_resp_send_404(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "text/html");
    char buf[512];
    size_t read_bytes;
    while ((read_bytes = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, read_bytes);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    FILE* f = fopen("/spiffs/web/settings.html", "r");
    if (!f) { ESP_LOGE(TAG, "Cannot open settings.html"); httpd_resp_send_404(req); return ESP_FAIL; }

    char buf[512];
    size_t read_bytes;
    while ((read_bytes = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, read_bytes);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t wifi_reset_post_handler(httpd_req_t *req)
{
    erase_wifi_nvs();
    httpd_resp_sendstr(req, "WiFi settings reset. ESP32 will restart...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ===============================
// STATUS HANDLER
// ===============================
static esp_err_t status_get_handler(httpd_req_t *req)
{
    wifi_sta_list_t sta_list;
    bool online = false;
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK && sta_list.num > 0)
        online = true;

    char resp_str[64];
    snprintf(resp_str, sizeof(resp_str), "{\"online\":%s}", online ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp_str, strlen(resp_str));
}

// ===============================
// WIFI CONFIG HANDLER
// ===============================
static esp_err_t wifi_config_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        const char* resp = "This endpoint accepts POST only for Wi-Fi configuration.";
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    } 
    else if (req->method == HTTP_POST) {
        char buf[128];
        int remaining = req->content_len;

        while (remaining > 0) {
            int received = httpd_req_recv(req, buf,
                                          remaining > sizeof(buf) ? sizeof(buf) : remaining);
            if (received <= 0) return ESP_FAIL;
            remaining -= received;
            buf[received] = '\0';
        }

        ESP_LOGI(TAG, "Received WiFi config: %s", buf);

        cJSON *json = cJSON_Parse(buf);
        if (!json) {
            ESP_LOGE(TAG, "Invalid JSON");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        cJSON *ssid_json = cJSON_GetObjectItemCaseSensitive(json, "ssid");
        cJSON *pass_json = cJSON_GetObjectItemCaseSensitive(json, "password");

        if (!cJSON_IsString(ssid_json) || !cJSON_IsString(pass_json)) {
            cJSON_Delete(json);
            ESP_LOGE(TAG, "Missing ssid or password");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        save_wifi_config(ssid_json->valuestring, pass_json->valuestring);
        cJSON_Delete(json);

        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_ERROR_CHECK(esp_wifi_deinit());
        wifi_init_softap();

        httpd_resp_sendstr(req, "WiFi config saved and AP restarted!");
        return ESP_OK;
    }

    return ESP_FAIL;
}

// ===============================
// NVS HANDLERS
// ===============================
void save_wifi_config(const char *ssid, const char *pass)
{
    nvs_handle_t nvs;
    if (nvs_open("wifi", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, "ssid", ssid);
        nvs_set_str(nvs, "pass", pass);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "WiFi config saved");
    }
}

bool load_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t nvs;
    if (nvs_open("wifi", NVS_READONLY, &nvs) != ESP_OK) return false;

    size_t s_len = ssid_len, p_len = pass_len;
    if (nvs_get_str(nvs, "ssid", ssid, &s_len) != ESP_OK) { nvs_close(nvs); return false; }
    if (nvs_get_str(nvs, "pass", pass, &p_len) != ESP_OK) { nvs_close(nvs); return false; }

    nvs_close(nvs);
    ESP_LOGI(TAG, "WiFi config loaded from NVS");
    return true;
}

void erase_wifi_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_OK || err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
        ESP_LOGI(TAG, "NVS erased");
    }
}

// ===============================
// WIFI INIT
// ===============================
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {0};
    char ssid[32], pass[64];
    if (load_wifi_config(ssid, sizeof(ssid), pass, sizeof(pass))) {
        strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
        strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
    } else {
        strncpy((char *)wifi_config.ap.ssid, WIFI_SSID, sizeof(wifi_config.ap.ssid));
        strncpy((char *)wifi_config.ap.password, WIFI_PASS, sizeof(wifi_config.ap.password));
    }

    wifi_config.ap.ssid_len = strlen((char *)wifi_config.ap.ssid);
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = strlen((char *)wifi_config.ap.password) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID:%s PASS:%s", wifi_config.ap.ssid, wifi_config.ap.password);
}

// ===============================
// WEB SERVER START
// ===============================
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.max_uri_handlers = 24;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uris[] = {
            { "/", HTTP_GET, root_get_handler, NULL },
            { "/servo", HTTP_POST, servo_post_handler, NULL },
            { "/sensors", HTTP_GET, sensors_get_handler, NULL },
            { "/status", HTTP_GET, status_get_handler, NULL },
            { "/settings", HTTP_GET, settings_get_handler, NULL },
            { "/web/style.css", HTTP_GET, style_get_handler, NULL },
            { "/upload", HTTP_POST, upload_post_handler, NULL },
            { "/move_xyz", HTTP_POST, move_xyz_post_handler, NULL },
            { "/wifi_reset", HTTP_POST, wifi_reset_post_handler, NULL },
            { "/wifi_config", HTTP_ANY, wifi_config_handler, NULL },
        };
        for (int i = 0; i < sizeof(uris)/sizeof(uris[0]); i++)
            httpd_register_uri_handler(server, &uris[i]);
    }
    return server;
}

// ===============================
// START FUNCTION
// ===============================
void wifi_servo_server_start(void)
{
    wifi_init_softap();
    start_webserver();
}