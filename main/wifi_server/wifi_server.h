// ===============================
// wifi_server.h
// ===============================

#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "file_manager.h"
#include "http_handlers.h"
#include "ws_handlers.h"
#include "esp_http_server.h"

#include "config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    char ssid[32];
    char pass[64];
} my_wifi_config_t;

void wifi_server_start(void);
void save_wifi_config(const char *ssid, const char *pass);
bool load_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
void wifi_init_softap(void);
void erase_wifi_nvs(void);

#endif // WIFI_SERVER_H
