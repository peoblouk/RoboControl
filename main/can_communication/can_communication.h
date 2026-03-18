// ===============================
// can_communication.h
// ===============================

#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H
#include "config.h"   // Configuration
#include "robot_io.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/twai.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    bool started;
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_failures;
    uint32_t rx_timeouts;
    uint32_t alert_events;
    uint32_t last_alerts;
    uint32_t heartbeat_frames;
    uint8_t last_state_flags;
    twai_status_info_t driver_status;
} can_runtime_stats_t;

void can_init(void);
void can_start(void);
bool can_is_started(void);
esp_err_t can_send_raw(uint32_t identifier, const uint8_t *data, uint8_t len);
esp_err_t can_send_heartbeat_now(void);
esp_err_t can_request_recovery(void);
esp_err_t can_get_runtime_stats(can_runtime_stats_t *out);

#endif // CAN_COMMUNICATION_H