// ===============================
// ws_handlers.h
// ===============================

#ifndef WS_HANDLERS_H
#define WS_HANDLERS_H

// ===============================
// Dependencies
// ===============================

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

// ===============================
// Public API
// ===============================

/** @brief Register WebSocket endpoint (`/ws`) handlers on HTTP server. */
esp_err_t ws_handlers_register(httpd_handle_t server);
/** @brief Start periodic WebSocket broadcast task (robot/sensor telemetry). */
void ws_handlers_start_task(void);

#endif // WS_HANDLERS_H
