// ===============================
// ws_handlers.h
// ===============================

#ifndef WS_HANDLERS_H
#define WS_HANDLERS_H

#include "esp_http_server.h"
#include "config.h"
#include "gcode.h"
#include "robot_io.h"

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

esp_err_t ws_handlers_register(httpd_handle_t server);
void ws_handlers_start_task(void);

#endif // WS_HANDLERS_H
