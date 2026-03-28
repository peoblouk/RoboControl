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

typedef enum {
    CAN_CMD_GET_STATUS = 0x01,
    CAN_CMD_ARM = 0x02,
    CAN_CMD_DISARM = 0x03,
    CAN_CMD_HOME = 0x04,
    CAN_CMD_STOP = 0x05,
    CAN_CMD_UPLOAD_BEGIN = 0x10,
    CAN_CMD_UPLOAD_DATA = 0x11,
    CAN_CMD_UPLOAD_END = 0x12,
    CAN_CMD_PROGRAM_RUN = 0x13,
    CAN_CMD_PROGRAM_DELETE = 0x14,
    CAN_CMD_PREPARE = 0x20,
    CAN_CMD_SYNC_START = 0x21,
} can_command_id_t;

typedef enum {
    CAN_PROTO_OK = 0x00,
    CAN_PROTO_ERR_INVALID_CMD = 0x01,
    CAN_PROTO_ERR_INVALID_LENGTH = 0x02,
    CAN_PROTO_ERR_INVALID_SLOT = 0x03,
    CAN_PROTO_ERR_UPLOAD_STATE = 0x04,
    CAN_PROTO_ERR_SEQUENCE = 0x05,
    CAN_PROTO_ERR_BUSY = 0x06,
    CAN_PROTO_ERR_NOT_READY = 0x07,
    CAN_PROTO_ERR_FILE = 0x08,
    CAN_PROTO_ERR_EXEC = 0x09,
} can_protocol_result_t;

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
    uint8_t last_protocol_error;
    uint8_t robot_state;
    uint8_t prepared_slot;
    uint8_t active_slot;
    bool upload_active;
    bool program_running;
    uint16_t upload_size_bytes;
    uint16_t upload_received_bytes;
    twai_status_info_t driver_status;
} can_runtime_stats_t;

void can_init(void);
void can_start(void);
bool can_is_started(void);
esp_err_t can_send_raw(uint32_t identifier, const uint8_t *data, uint8_t len);
esp_err_t can_send_heartbeat_now(void);
esp_err_t can_request_recovery(void);
esp_err_t can_get_runtime_stats(can_runtime_stats_t *out);
const char *can_protocol_result_to_str(can_protocol_result_t code);

#endif // CAN_COMMUNICATION_H