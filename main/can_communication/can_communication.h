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

// ===============================
// Public Types
// ===============================

/** @brief CAN protocol command IDs (`data[0]` in command frame). */
typedef enum {
    CAN_CMD_GET_STATUS     = 0x01, ///< Request current node status.
    CAN_CMD_ARM            = 0x02, ///< Arm robot outputs.
    CAN_CMD_DISARM         = 0x03, ///< Disarm robot outputs.
    CAN_CMD_HOME           = 0x04, ///< Start homing sequence.
    CAN_CMD_STOP           = 0x05, ///< Stop current motion/program.
    CAN_CMD_UPLOAD_BEGIN   = 0x10, ///< Start program upload.
    CAN_CMD_UPLOAD_DATA    = 0x11, ///< Continue program upload payload.
    CAN_CMD_UPLOAD_END     = 0x12, ///< Finish program upload.
    CAN_CMD_PROGRAM_RUN    = 0x13, ///< Run program from slot.
    CAN_CMD_PROGRAM_DELETE = 0x14, ///< Delete program slot.
    CAN_CMD_PREPARE        = 0x20, ///< Prepare slot/program for sync run.
    CAN_CMD_SYNC_START     = 0x21, ///< Start synchronized execution.
    CAN_CMD_JOG_XY         = 0x22, ///< Manual jog in XY plane.
    CAN_CMD_JOG_Z          = 0x23, ///< Manual jog in Z axis.
} can_command_id_t;

/** @brief Telemetry/info frame kinds sent on `CAN_INFO_ID_BASE + node_id`. */
typedef enum {
    CAN_INFO_WORK_OFFSET = 0x01, ///< Current work offset XYZ in 0.1 mm.
    CAN_INFO_TCP_WORK_XYZ = 0x02, ///< Current TCP estimate in work frame, 0.1 mm.
    CAN_INFO_TCP_META = 0x03, ///< Metadata for the accompanying value frames.
    CAN_INFO_VALUES_0_2 = 0x10, ///< Values 0..2 in 0.1 deg.
    CAN_INFO_VALUES_3_5 = 0x11, ///< Values 3..5 in 0.1 deg.
} can_info_frame_kind_t;

/** @brief Source used for `CAN_INFO_VALUES_*` telemetry values. */
typedef enum {
    CAN_INFO_SOURCE_EST_JOINTS = 0x00, ///< Estimated joint angles from the controller.
    CAN_INFO_SOURCE_ADC_SENSORS = 0x01, ///< Angles read from ADC sensors.
} can_info_value_source_t;

#define CAN_INFO_VALUE_INVALID_I16 ((int16_t)0x7FFF)

/** @brief Protocol-level command handling result codes. */
typedef enum {
    CAN_PROTO_OK                 = 0x00, ///< Command accepted and processed.
    CAN_PROTO_ERR_INVALID_CMD    = 0x01, ///< Unknown command ID.
    CAN_PROTO_ERR_INVALID_LENGTH = 0x02, ///< Invalid frame payload length.
    CAN_PROTO_ERR_INVALID_SLOT   = 0x03, ///< Program slot out of range.
    CAN_PROTO_ERR_UPLOAD_STATE   = 0x04, ///< Upload not in valid state.
    CAN_PROTO_ERR_SEQUENCE       = 0x05, ///< Upload sequence mismatch.
    CAN_PROTO_ERR_BUSY           = 0x06, ///< Node is busy.
    CAN_PROTO_ERR_NOT_READY      = 0x07, ///< Node is not ready.
    CAN_PROTO_ERR_FILE           = 0x08, ///< File operation failed.
    CAN_PROTO_ERR_EXEC           = 0x09, ///< Program execution failed.
} can_protocol_result_t;

/** @brief Snapshot of CAN runtime counters and node state. */
typedef struct {
    bool started;                     ///< CAN driver start state.
    uint32_t tx_frames;               ///< Number of transmitted frames.
    uint32_t rx_frames;               ///< Number of received frames.
    uint32_t tx_failures;             ///< Failed TX attempts.
    uint32_t rx_timeouts;             ///< RX timeout count.
    uint32_t alert_events;            ///< Number of TWAI alert events.
    uint32_t last_alerts;             ///< Last TWAI alert bitmask.
    uint32_t heartbeat_frames;        ///< Sent heartbeat frame count.
    uint8_t last_state_flags;         ///< Last published CAN state flags.
    uint8_t last_protocol_error;      ///< Last protocol result code.
    uint8_t robot_state;              ///< Current robot high-level state.
    uint8_t prepared_slot;            ///< Slot prepared for run (`0xFF` = none).
    uint8_t active_slot;              ///< Slot currently running (`0xFF` = none).
    bool upload_active;               ///< True while upload session is open.
    bool program_running;             ///< True while program is executing.
    uint16_t upload_size_bytes;       ///< Expected upload size.
    uint16_t upload_received_bytes;   ///< Bytes already received in upload.
    twai_status_info_t driver_status; ///< Raw TWAI driver status snapshot.
} can_runtime_stats_t;

// ===============================
// Public API
// ===============================

/** @brief Configure CAN/TWAI layer and create CAN tasks. */
void can_init(void);
/** @brief Start CAN/TWAI driver if not already started. */
void can_start(void);
/** @brief Return `true` if CAN/TWAI driver is currently started. */
bool can_is_started(void);
/** @brief Send raw standard CAN frame.
 *  @param identifier 11-bit CAN ID.
 *  @param data Pointer to payload data (can be `NULL` when `len == 0`).
 *  @param len Payload length in bytes (0..8).
 */
esp_err_t can_send_raw(uint32_t identifier, const uint8_t *data, uint8_t len);
/** @brief Send one heartbeat/status frame immediately. */
esp_err_t can_send_heartbeat_now(void);
/** @brief Request TWAI bus recovery (typically after bus-off). */
esp_err_t can_request_recovery(void);
/** @brief Fill runtime CAN statistics snapshot into `out`. */
esp_err_t can_get_runtime_stats(can_runtime_stats_t *out);
/** @brief Convert protocol result code to short text label. */
const char *can_protocol_result_to_str(can_protocol_result_t code);

#endif // CAN_COMMUNICATION_H
