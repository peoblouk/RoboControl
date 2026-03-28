// ===============================
// wifi_server.c
// ===============================

#include "wifi_server.h"

static const char *TAG = "wifi_server";

static httpd_handle_t g_httpd = NULL;

void save_wifi_config(const char *ssid, const char *pass) {
    nvs_handle_t nvs;
    if (nvs_open("wifi", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, "ssid", ssid);
        nvs_set_str(nvs, "pass", pass);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

bool load_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    nvs_handle_t nvs;
    if (nvs_open("wifi", NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    size_t s_len = ssid_len, p_len = pass_len;
    if (nvs_get_str(nvs, "ssid", ssid, &s_len) != ESP_OK) {
        nvs_close(nvs);
        return false;
    }
    if (nvs_get_str(nvs, "pass", pass, &p_len) != ESP_OK) {
        nvs_close(nvs);
        return false;
    }
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

static void start_mdns(void) {
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

void wifi_init_softap(void) {
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
    start_mdns();
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 24;
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&g_httpd, &config) != ESP_OK) {
        return NULL;
    }

    if (http_handlers_register(g_httpd) != ESP_OK ||
        file_manager_register(g_httpd) != ESP_OK ||
        ws_handlers_register(g_httpd) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register one or more URI handlers");
        httpd_stop(g_httpd);
        g_httpd = NULL;
        return NULL;
    }

    return g_httpd;
}

void wifi_server_start(void) {
    wifi_init_softap();
    if (start_webserver()) {
        ws_handlers_start_task();
    }
}
