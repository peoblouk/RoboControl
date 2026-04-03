// ===============================
// wifi_server.h
// ===============================

#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

// ===============================
// Dependencies
// ===============================

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

// ===============================
// Public Types
// ===============================

/** @brief Persistent Wi-Fi AP credentials stored in NVS. */
typedef struct {
    char ssid[32];
    char pass[64];
} my_wifi_config_t;

// ===============================
// Public API
// ===============================

/** @brief Initialize SoftAP + HTTP/WS server stack and start background services. */
void wifi_server_start(void);
/** @brief Save AP credentials into NVS namespace `wifi`. */
void save_wifi_config(const char *ssid, const char *pass);
/** @brief Load AP credentials from NVS; returns `false` when missing/invalid. */
bool load_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
/** @brief Initialize Wi-Fi in AP mode with configured/default credentials. */
void wifi_init_softap(void);
/** @brief Erase Wi-Fi related NVS data and reinitialize NVS partition. */
void erase_wifi_nvs(void);

#endif // WIFI_SERVER_H
