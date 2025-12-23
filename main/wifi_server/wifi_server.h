// ===============================
// wifi_server.h
// ===============================

#ifndef WIFI_SERVER
#define WIFI_SERVER

#define WIFI_SSID      "RoboControl"
#define WIFI_PASS      "Robo-Control123"   
#define MAX_STA_CONN   4 // max stable connections

typedef struct {
    char ssid[32];
    char pass[64];
} my_wifi_config_t;

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include "mdns.h"

#include <stdlib.h>
#include <inttypes.h>
#include "esp_spiffs.h"
#include "cJSON.h"

#include <dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "robot_io.h"  // Robot IO (servos, sensors)
#include "core_config.h"   // Core Configuration


// ===============================
// FUNCTION PROTOTYPES
// ===============================
void wifi_servo_server_start(void);
void save_wifi_config(const char *ssid, const char *pass);
bool load_wifi_config(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
void wifi_init_softap(void);
void erase_wifi_nvs(void);

#endif // WIFI_SERVO_SERVER