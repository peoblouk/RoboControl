// ===============================
// http_handlers.h
// ===============================

#ifndef HTTP_HANDLERS_H
#define HTTP_HANDLERS_H

#include "esp_http_server.h"
#include "wifi_server.h"

#include "config.h"
#include "cJSON.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

esp_err_t http_handlers_register(httpd_handle_t server);

#endif // HTTP_HANDLERS_H