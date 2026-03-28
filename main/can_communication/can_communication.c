// ===============================
// can_communication.c
// ===============================

#include "can_communication.h"

#include <errno.h>
#include <sys/stat.h>

static const char *TAG = "can";

typedef struct {
    bool active;
    uint8_t slot;
    uint16_t expected_size;
    uint16_t received_size;
    uint16_t next_seq;
    FILE *fp;
    char temp_path[CAN_SLOT_PATH_MAX];
    char final_path[CAN_SLOT_PATH_MAX];
} can_upload_session_t;

#define CAN_FLAG_ARMED            (1U << 0)
#define CAN_FLAG_OPERATING        (1U << 1)
#define CAN_FLAG_REFERENCED       (1U << 2)
#define CAN_FLAG_TCP_EST          (1U << 3)
#define CAN_FLAG_SYNC_READY       (1U << 4)
#define CAN_FLAG_UPLOAD_ACTIVE    (1U << 5)
#define CAN_FLAG_PROGRAM_RUNNING  (1U << 6)
#define CAN_FLAG_ERROR            (1U << 7)

static volatile bool s_can_started = false;
static volatile bool s_can_tasks_started = false;

static volatile uint32_t s_tx_frames = 0;
static volatile uint32_t s_rx_frames = 0;
static volatile uint32_t s_tx_failures = 0;
static volatile uint32_t s_rx_timeouts = 0;
static volatile uint32_t s_alert_events = 0;
static volatile uint32_t s_last_alerts = 0;
static volatile uint32_t s_heartbeat_frames = 0;
static volatile uint8_t s_last_state_flags = 0;
static volatile uint8_t s_last_protocol_error = CAN_PROTO_OK;

static portMUX_TYPE s_proto_mux = portMUX_INITIALIZER_UNLOCKED;
static can_upload_session_t s_upload = {0};
static int s_prepared_slot = -1;
static int s_active_slot = -1;

static uint8_t can_slot_to_wire(int slot)
{
    return (slot >= 0 && slot < CAN_PROGRAM_SLOT_COUNT) ? (uint8_t)slot : 0xFFU;
}

static void can_set_last_protocol_error(can_protocol_result_t code)
{
    portENTER_CRITICAL(&s_proto_mux);
    s_last_protocol_error = (uint8_t)code;
    portEXIT_CRITICAL(&s_proto_mux);
}

const char *can_protocol_result_to_str(can_protocol_result_t code)
{
    switch (code) {
        case CAN_PROTO_OK: return "ok";
        case CAN_PROTO_ERR_INVALID_CMD: return "invalid_cmd";
        case CAN_PROTO_ERR_INVALID_LENGTH: return "invalid_length";
        case CAN_PROTO_ERR_INVALID_SLOT: return "invalid_slot";
        case CAN_PROTO_ERR_UPLOAD_STATE: return "upload_state";
        case CAN_PROTO_ERR_SEQUENCE: return "sequence";
        case CAN_PROTO_ERR_BUSY: return "busy";
        case CAN_PROTO_ERR_NOT_READY: return "not_ready";
        case CAN_PROTO_ERR_FILE: return "file";
        case CAN_PROTO_ERR_EXEC: return "exec";
        default: return "unknown";
    }
}

static const char *can_mode_str(void)
{
#if CAN_NO_ACK_MODE
    return "no-ack";
#else
    return "normal";
#endif
}

static const char *can_state_str(twai_state_t state)
{
    switch (state) {
        case TWAI_STATE_STOPPED: return "stopped";
        case TWAI_STATE_RUNNING: return "running";
        case TWAI_STATE_BUS_OFF: return "bus-off";
        case TWAI_STATE_RECOVERING: return "recovering";
        default: return "unknown";
    }
}

static const char *can_cmd_str(uint8_t cmd)
{
    switch ((can_command_id_t)cmd) {
        case CAN_CMD_GET_STATUS: return "GET_STATUS";
        case CAN_CMD_ARM: return "ARM";
        case CAN_CMD_DISARM: return "DISARM";
        case CAN_CMD_HOME: return "HOME";
        case CAN_CMD_STOP: return "STOP";
        case CAN_CMD_UPLOAD_BEGIN: return "UPLOAD_BEGIN";
        case CAN_CMD_UPLOAD_DATA: return "UPLOAD_DATA";
        case CAN_CMD_UPLOAD_END: return "UPLOAD_END";
        case CAN_CMD_PROGRAM_RUN: return "PROGRAM_RUN";
        case CAN_CMD_PROGRAM_DELETE: return "PROGRAM_DELETE";
        case CAN_CMD_PREPARE: return "PREPARE";
        case CAN_CMD_SYNC_START: return "SYNC_START";
        default: return "UNKNOWN";
    }
}

static twai_timing_config_t can_get_timing_config(void)
{
#if CAN_BITRATE == 25000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
#elif CAN_BITRATE == 50000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
#elif CAN_BITRATE == 100000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
#elif CAN_BITRATE == 125000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
#elif CAN_BITRATE == 250000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
#elif CAN_BITRATE == 500000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE == 800000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
#elif CAN_BITRATE == 1000000
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
#else
#error Unsupported CAN_BITRATE value. Add a TWAI timing macro mapping in can_get_timing_config().
#endif
}

static void can_log_frame(const char *prefix, const twai_message_t *msg)
{
    char payload[3 * TWAI_FRAME_MAX_DLC + 1] = {0};
    char *p = payload;

    for (uint8_t i = 0; i < msg->data_length_code && i < TWAI_FRAME_MAX_DLC; i++) {
        int written = snprintf(p, (size_t)(payload + sizeof(payload) - p),
                               (i == 0) ? "%02X" : " %02X",
                               msg->data[i]);
        if (written <= 0) break;
        p += written;
    }

    ESP_LOGD(TAG, "%s id=0x%03" PRIX32 " dlc=%u extd=%u rtr=%u data=[%s]",
             prefix,
             msg->identifier,
             (unsigned)msg->data_length_code,
             (unsigned)msg->extd,
             (unsigned)msg->rtr,
             payload);
}

static bool can_slot_valid(uint8_t slot)
{
    return slot < CAN_PROGRAM_SLOT_COUNT;
}

static bool can_frame_targets_node(const twai_message_t *msg)
{
    if (msg->extd || msg->rtr) return false;
    if (msg->identifier == (CAN_CMD_ID_BASE + (CAN_NODE_ID & 0x7FU))) return true;
    if (msg->identifier == CAN_CMD_ID_BROADCAST) return true;
    return false;
}

static bool can_frame_is_broadcast(const twai_message_t *msg)
{
    return msg->identifier == CAN_CMD_ID_BROADCAST;
}

static void can_make_slot_path(uint8_t slot, bool temp_file, char *out, size_t out_sz)
{
    snprintf(out, out_sz,
             temp_file ? "%s/can_slot_%u.gcode.tmp" : "%s/can_slot_%u.gcode",
             FILE_STORAGE_PATH,
             (unsigned)slot);
}

static bool can_file_exists(const char *path)
{
    struct stat st = {0};
    return (path != NULL) && (stat(path, &st) == 0) && S_ISREG(st.st_mode);
}

static void can_upload_reset(bool discard_temp)
{
    if (s_upload.fp != NULL) {
        fclose(s_upload.fp);
        s_upload.fp = NULL;
    }

    if (discard_temp && s_upload.temp_path[0] != '\0') {
        remove(s_upload.temp_path);
    }

    memset(&s_upload, 0, sizeof(s_upload));
}

static void can_clear_prepared_slot(void)
{
    portENTER_CRITICAL(&s_proto_mux);
    s_prepared_slot = -1;
    portEXIT_CRITICAL(&s_proto_mux);
    robot_set_sync_ready(false);
}

static void can_set_prepared_slot(uint8_t slot)
{
    portENTER_CRITICAL(&s_proto_mux);
    s_prepared_slot = (int)slot;
    portEXIT_CRITICAL(&s_proto_mux);
    robot_set_sync_ready(true);
}

static void can_set_active_slot(int slot)
{
    portENTER_CRITICAL(&s_proto_mux);
    s_active_slot = slot;
    portEXIT_CRITICAL(&s_proto_mux);
}

static int can_get_prepared_slot(void)
{
    portENTER_CRITICAL(&s_proto_mux);
    int slot = s_prepared_slot;
    portEXIT_CRITICAL(&s_proto_mux);
    return slot;
}

static int can_get_active_slot(void)
{
    portENTER_CRITICAL(&s_proto_mux);
    int slot = s_active_slot;
    portEXIT_CRITICAL(&s_proto_mux);
    return slot;
}

static void can_housekeeping(void)
{
    if (!robot_is_program_running() && !robot_is_operating()) {
        can_set_active_slot(-1);
    }
}

static uint8_t can_build_state_flags(void)
{
    uint8_t flags = 0;

    if (robot_is_armed()) flags |= CAN_FLAG_ARMED;
    if (robot_is_operating()) flags |= CAN_FLAG_OPERATING;
    if (robot_is_referenced()) flags |= CAN_FLAG_REFERENCED;
    if (robot_has_tcp_estimate()) flags |= CAN_FLAG_TCP_EST;
    if (robot_is_sync_ready()) flags |= CAN_FLAG_SYNC_READY;
    if (s_upload.active) flags |= CAN_FLAG_UPLOAD_ACTIVE;
    if (robot_is_program_running()) flags |= CAN_FLAG_PROGRAM_RUNNING;
    if (robot_get_system_state() == ROBOT_STATE_ERROR) flags |= CAN_FLAG_ERROR;

    return flags;
}

static void can_fill_status_payload(uint8_t data[8])
{
    can_housekeeping();

    portENTER_CRITICAL(&s_proto_mux);
    const uint8_t last_proto_error = s_last_protocol_error;
    portEXIT_CRITICAL(&s_proto_mux);

    data[0] = (uint8_t)CAN_PROTOCOL_VERSION;
    data[1] = (uint8_t)robot_get_system_state();
    data[2] = can_build_state_flags();
    data[3] = can_slot_to_wire(can_get_prepared_slot());
    data[4] = can_slot_to_wire(can_get_active_slot());
    data[5] = last_proto_error;
    data[6] = (uint8_t)robot_get_last_error();
    data[7] = (uint8_t)CAN_NODE_ID;
}

static esp_err_t can_send_response(uint8_t cmd,
                                   can_protocol_result_t result,
                                   uint8_t detail0,
                                   uint8_t detail1,
                                   uint8_t detail2,
                                   uint8_t detail3)
{
    uint8_t data[8] = {0};
    data[0] = cmd;
    data[1] = (uint8_t)result;
    data[2] = (uint8_t)robot_get_system_state();
    data[3] = can_build_state_flags();
    data[4] = detail0;
    data[5] = detail1;
    data[6] = detail2;
    data[7] = detail3;

    can_set_last_protocol_error(result);
    return can_send_raw(CAN_RESP_ID_BASE + (CAN_NODE_ID & 0x7FU), data, sizeof(data));
}

static esp_err_t can_send_status_frame_internal(void)
{
    uint8_t data[8] = {0};
    can_fill_status_payload(data);

    esp_err_t err = can_send_raw(CAN_STATUS_ID_BASE + (CAN_NODE_ID & 0x7FU), data, sizeof(data));
    if (err == ESP_OK) {
        s_last_state_flags = data[2];
        s_heartbeat_frames++;
    }
    return err;
}

static can_protocol_result_t can_start_upload(uint8_t slot, uint16_t expected_size)
{
    if (!can_slot_valid(slot)) return CAN_PROTO_ERR_INVALID_SLOT;
    if (s_upload.active) return CAN_PROTO_ERR_UPLOAD_STATE;
    if (robot_is_program_running() || robot_is_operating()) return CAN_PROTO_ERR_BUSY;

    can_make_slot_path(slot, true, s_upload.temp_path, sizeof(s_upload.temp_path));
    can_make_slot_path(slot, false, s_upload.final_path, sizeof(s_upload.final_path));
    remove(s_upload.temp_path);

    s_upload.fp = fopen(s_upload.temp_path, "wb");
    if (s_upload.fp == NULL) {
        ESP_LOGE(TAG, "UPLOAD_BEGIN open failed for slot %u (errno=%d: %s)",
                 (unsigned)slot, errno, strerror(errno));
        memset(&s_upload, 0, sizeof(s_upload));
        return CAN_PROTO_ERR_FILE;
    }

    s_upload.active = true;
    s_upload.slot = slot;
    s_upload.expected_size = expected_size;
    s_upload.received_size = 0;
    s_upload.next_seq = 0;
    return CAN_PROTO_OK;
}

static can_protocol_result_t can_append_upload_chunk(uint16_t seq, const uint8_t *data, uint8_t len)
{
    if (!s_upload.active || s_upload.fp == NULL) return CAN_PROTO_ERR_UPLOAD_STATE;
    if (len == 0) return CAN_PROTO_ERR_INVALID_LENGTH;
    if (seq != s_upload.next_seq) return CAN_PROTO_ERR_SEQUENCE;
    if ((uint32_t)s_upload.received_size + len > s_upload.expected_size) return CAN_PROTO_ERR_UPLOAD_STATE;

    size_t written = fwrite(data, 1, len, s_upload.fp);
    if (written != len) {
        ESP_LOGE(TAG, "UPLOAD_DATA write failed slot=%u seq=%u", (unsigned)s_upload.slot, (unsigned)seq);
        can_upload_reset(true);
        return CAN_PROTO_ERR_FILE;
    }

    s_upload.received_size = (uint16_t)(s_upload.received_size + len);
    s_upload.next_seq = (uint16_t)(s_upload.next_seq + 1U);
    return CAN_PROTO_OK;
}

static can_protocol_result_t can_finish_upload(void)
{
    if (!s_upload.active || s_upload.fp == NULL) return CAN_PROTO_ERR_UPLOAD_STATE;
    if (s_upload.received_size != s_upload.expected_size) return CAN_PROTO_ERR_UPLOAD_STATE;

    fclose(s_upload.fp);
    s_upload.fp = NULL;

    remove(s_upload.final_path);
    if (rename(s_upload.temp_path, s_upload.final_path) != 0) {
        ESP_LOGE(TAG, "UPLOAD_END rename failed '%s' -> '%s' (errno=%d: %s)",
                 s_upload.temp_path,
                 s_upload.final_path,
                 errno,
                 strerror(errno));
        can_upload_reset(true);
        return CAN_PROTO_ERR_FILE;
    }

    can_upload_reset(false);
    return CAN_PROTO_OK;
}

static can_protocol_result_t can_delete_slot(uint8_t slot)
{
    char path[CAN_SLOT_PATH_MAX] = {0};

    if (!can_slot_valid(slot)) return CAN_PROTO_ERR_INVALID_SLOT;
    if (s_upload.active && s_upload.slot == slot) {
        can_upload_reset(true);
    }

    can_make_slot_path(slot, false, path, sizeof(path));
    if (!can_file_exists(path)) return CAN_PROTO_ERR_FILE;
    if (remove(path) != 0) return CAN_PROTO_ERR_FILE;

    if (can_get_prepared_slot() == slot) can_clear_prepared_slot();
    if (can_get_active_slot() == slot) can_set_active_slot(-1);
    return CAN_PROTO_OK;
}

static bool can_start_program_slot(uint8_t slot)
{
    char path[CAN_SLOT_PATH_MAX] = {0};

    if (!can_slot_valid(slot)) return false;
    if (!robot_is_armed() || !robot_is_referenced() || !robot_has_tcp_estimate()) return false;

    can_make_slot_path(slot, false, path, sizeof(path));
    if (!can_file_exists(path)) return false;

    if (!robot_core_run_gcode(path)) {
        return false;
    }

    can_clear_prepared_slot();
    can_set_active_slot(slot);
    return true;
}

static void can_reply_protocol_error(uint8_t cmd,
                                     can_protocol_result_t result,
                                     uint8_t detail0,
                                     uint8_t detail1)
{
    (void)can_send_response(cmd, result, detail0, detail1, 0, 0);
    ESP_LOGW(TAG, "%s failed: %s", can_cmd_str(cmd), can_protocol_result_to_str(result));
}

static void can_process_command_frame(const twai_message_t *msg)
{
    const bool broadcast = can_frame_is_broadcast(msg);
    const uint8_t cmd = msg->data[0];

    ESP_LOGI(TAG, "CMD %s (%02X) from id=0x%03" PRIX32 "%s",
             can_cmd_str(cmd),
             cmd,
             msg->identifier,
             broadcast ? " [broadcast]" : "");

    switch ((can_command_id_t)cmd) {
        case CAN_CMD_GET_STATUS:
            (void)can_send_response(cmd,
                                    CAN_PROTO_OK,
                                    can_slot_to_wire(can_get_prepared_slot()),
                                    can_slot_to_wire(can_get_active_slot()),
                                    (uint8_t)robot_get_last_error(),
                                    (uint8_t)CAN_NODE_ID);
            (void)can_send_status_frame_internal();
            return;

        case CAN_CMD_ARM:
            robot_arm();
            (void)can_send_response(cmd, CAN_PROTO_OK, 0, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;

        case CAN_CMD_DISARM:
            can_clear_prepared_slot();
            can_set_active_slot(-1);
            can_upload_reset(true);
            robot_disarm();
            (void)can_send_response(cmd, CAN_PROTO_OK, 0, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;

        case CAN_CMD_HOME: {
            bool ok = robot_cmd_home_reference();
            if (!ok) {
                can_reply_protocol_error(cmd,
                                         robot_is_armed() ? CAN_PROTO_ERR_BUSY : CAN_PROTO_ERR_NOT_READY,
                                         0,
                                         0);
                return;
            }
            can_clear_prepared_slot();
            can_set_active_slot(-1);
            (void)can_send_response(cmd, CAN_PROTO_OK, 0, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;
        }

        case CAN_CMD_STOP:
            can_clear_prepared_slot();
            can_set_active_slot(-1);
            robot_stop_all();
            (void)can_send_response(cmd, CAN_PROTO_OK, 0, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;

        case CAN_CMD_UPLOAD_BEGIN: {
            if (broadcast) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_CMD, 0, 0);
                return;
            }
            if (msg->data_length_code < 4) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_LENGTH, msg->data_length_code, 0);
                return;
            }

            const uint8_t slot = msg->data[1];
            const uint16_t expected_size = (uint16_t)msg->data[2] | ((uint16_t)msg->data[3] << 8);
            can_protocol_result_t result = can_start_upload(slot, expected_size);
            if (result != CAN_PROTO_OK) {
                can_reply_protocol_error(cmd, result, slot, 0);
                return;
            }

            (void)can_send_response(cmd,
                                    CAN_PROTO_OK,
                                    slot,
                                    (uint8_t)(expected_size & 0xFFU),
                                    (uint8_t)((expected_size >> 8) & 0xFFU),
                                    0);
            return;
        }

        case CAN_CMD_UPLOAD_DATA: {
            if (broadcast) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_CMD, 0, 0);
                return;
            }
            if (msg->data_length_code < 4) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_LENGTH, msg->data_length_code, 0);
                return;
            }

            const uint16_t seq = (uint16_t)msg->data[1] | ((uint16_t)msg->data[2] << 8);
            const uint8_t payload_len = (uint8_t)(msg->data_length_code - 3U);
            can_protocol_result_t result = can_append_upload_chunk(seq, &msg->data[3], payload_len);
            if (result != CAN_PROTO_OK) {
                uint16_t expected_seq = s_upload.next_seq;
                can_reply_protocol_error(cmd,
                                         result,
                                         (uint8_t)(expected_seq & 0xFFU),
                                         (uint8_t)((expected_seq >> 8) & 0xFFU));
                return;
            }

            (void)can_send_response(cmd,
                                    CAN_PROTO_OK,
                                    (uint8_t)(seq & 0xFFU),
                                    (uint8_t)((seq >> 8) & 0xFFU),
                                    (uint8_t)(s_upload.received_size & 0xFFU),
                                    (uint8_t)((s_upload.received_size >> 8) & 0xFFU));
            return;
        }

        case CAN_CMD_UPLOAD_END: {
            if (broadcast) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_CMD, 0, 0);
                return;
            }

            const uint8_t slot = s_upload.slot;
            const uint16_t bytes = s_upload.received_size;
            can_protocol_result_t result = can_finish_upload();
            if (result != CAN_PROTO_OK) {
                can_reply_protocol_error(cmd, result, slot, 0);
                return;
            }

            (void)can_send_response(cmd,
                                    CAN_PROTO_OK,
                                    slot,
                                    (uint8_t)(bytes & 0xFFU),
                                    (uint8_t)((bytes >> 8) & 0xFFU),
                                    0);
            (void)can_send_status_frame_internal();
            return;
        }

        case CAN_CMD_PROGRAM_RUN: {
            if (msg->data_length_code < 2) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_LENGTH, msg->data_length_code, 0);
                return;
            }

            const uint8_t slot = msg->data[1];
            if (!can_slot_valid(slot)) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_SLOT, slot, 0);
                return;
            }
            if (s_upload.active) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_BUSY, slot, 0);
                return;
            }
            if (!can_start_program_slot(slot)) {
                can_reply_protocol_error(cmd,
                                         (!robot_is_armed() || !robot_is_referenced() || !robot_has_tcp_estimate()) ? CAN_PROTO_ERR_NOT_READY : CAN_PROTO_ERR_EXEC,
                                         slot,
                                         0);
                return;
            }

            (void)can_send_response(cmd, CAN_PROTO_OK, slot, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;
        }

        case CAN_CMD_PROGRAM_DELETE: {
            if (msg->data_length_code < 2) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_LENGTH, msg->data_length_code, 0);
                return;
            }

            const uint8_t slot = msg->data[1];
            can_protocol_result_t result = can_delete_slot(slot);
            if (result != CAN_PROTO_OK) {
                can_reply_protocol_error(cmd, result, slot, 0);
                return;
            }

            (void)can_send_response(cmd, CAN_PROTO_OK, slot, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;
        }

        case CAN_CMD_PREPARE: {
            if (msg->data_length_code < 2) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_LENGTH, msg->data_length_code, 0);
                return;
            }

            const uint8_t slot = msg->data[1];
            char path[CAN_SLOT_PATH_MAX] = {0};

            if (!can_slot_valid(slot)) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_SLOT, slot, 0);
                return;
            }
            if (s_upload.active || robot_is_program_running() || robot_is_operating()) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_BUSY, slot, 0);
                return;
            }
            if (!robot_is_armed() || !robot_is_referenced() || !robot_has_tcp_estimate()) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_NOT_READY, slot, 0);
                return;
            }

            can_make_slot_path(slot, false, path, sizeof(path));
            if (!can_file_exists(path)) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_FILE, slot, 0);
                return;
            }

            can_set_prepared_slot(slot);
            (void)can_send_response(cmd, CAN_PROTO_OK, slot, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;
        }

        case CAN_CMD_SYNC_START: {
            const int prepared_slot = can_get_prepared_slot();
            if (prepared_slot < 0 || !robot_is_sync_ready()) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_NOT_READY, 0, 0);
                return;
            }
            if (!can_start_program_slot((uint8_t)prepared_slot)) {
                can_reply_protocol_error(cmd, CAN_PROTO_ERR_EXEC, (uint8_t)prepared_slot, 0);
                return;
            }

            (void)can_send_response(cmd, CAN_PROTO_OK, (uint8_t)prepared_slot, 0, 0, 0);
            (void)can_send_status_frame_internal();
            return;
        }

        default:
            can_reply_protocol_error(cmd, CAN_PROTO_ERR_INVALID_CMD, 0, 0);
            return;
    }
}

static void can_receive_task(void *arg)
{
    (void)arg;
    const TickType_t rx_wait = pdMS_TO_TICKS(250);
    TickType_t poll_delay = pdMS_TO_TICKS(1);
    if (poll_delay == 0) {
        poll_delay = 1;
    }

    for (;;) {
        twai_message_t msg = {0};
        esp_err_t err = twai_receive(&msg, rx_wait);
        if (err == ESP_OK) {
            s_rx_frames++;
            can_log_frame("RX", &msg);
            if (can_frame_targets_node(&msg) && msg.data_length_code > 0) {
                can_process_command_frame(&msg);
            }
        } else if (err == ESP_ERR_TIMEOUT) {
            s_rx_timeouts++;
        } else {
            ESP_LOGW(TAG, "twai_receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Yield at least one tick so other TWAI users (e.g. CLI probe/status TX) can acquire the driver lock.
        vTaskDelay(poll_delay);
    }
}

static void can_alert_task(void *arg)
{
    (void)arg;
    uint32_t tx_fail_suppressed = 0;
    TickType_t tx_fail_last_log_tick = 0;
    const TickType_t tx_fail_summary_period = pdMS_TO_TICKS(10000);

    for (;;) {
        uint32_t alerts = 0;
        esp_err_t err = twai_read_alerts(&alerts, pdMS_TO_TICKS(500));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "twai_read_alerts failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        s_alert_events++;
        s_last_alerts = alerts;

        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGW(TAG, "TWAI RX queue full");
        }
        if (alerts & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGW(TAG, "TWAI bus error");
        }
        if (alerts & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI entered error-passive state");
        }
        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            (void)twai_initiate_recovery();
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGW(TAG, "TWAI bus recovered, restarting controller");
            esp_err_t start_err = twai_start();
            if (start_err != ESP_OK && start_err != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "twai_start after recovery failed: %s", esp_err_to_name(start_err));
            }
        }
        if (alerts & TWAI_ALERT_TX_FAILED) {
            s_tx_failures++;
            TickType_t now = xTaskGetTickCount();
            if (tx_fail_last_log_tick == 0 || (now - tx_fail_last_log_tick) >= tx_fail_summary_period) {
                if (tx_fail_suppressed > 0) {
                    ESP_LOGW(TAG, "TWAI TX failed (suppressed=%lu)", (unsigned long)tx_fail_suppressed);
                    tx_fail_suppressed = 0;
                } else {
                    ESP_LOGW(TAG, "TWAI TX failed");
                }
                tx_fail_last_log_tick = now;
            } else {
                tx_fail_suppressed++;
            }
        }
        if (alerts & TWAI_ALERT_TX_SUCCESS) {
            ESP_LOGD(TAG, "TWAI TX success");
        }
    }
}

static void can_status_task(void *arg)
{
    (void)arg;
    esp_err_t last_err = ESP_OK;
    uint32_t suppressed = 0;
    TickType_t last_log_tick = 0;
    const TickType_t summary_period = pdMS_TO_TICKS(10000);

    for (;;) {
        if (s_can_started) {
            esp_err_t err = can_send_status_frame_internal();

            if (err == ESP_OK) {
                if (last_err != ESP_OK) {
                    ESP_LOGI(TAG, "Status frame TX recovered");
                }
                last_err = ESP_OK;
                suppressed = 0;
                last_log_tick = xTaskGetTickCount();
            } else if (err != last_err) {
                ESP_LOGW(TAG, "Status frame TX failed: %s", esp_err_to_name(err));
                last_err = err;
                suppressed = 0;
                last_log_tick = xTaskGetTickCount();
            } else {
                suppressed++;
                TickType_t now = xTaskGetTickCount();
                if ((now - last_log_tick) >= summary_period) {
                    ESP_LOGW(TAG, "Status frame TX still failing: %s (suppressed=%lu)",
                             esp_err_to_name(err),
                             (unsigned long)suppressed);
                    suppressed = 0;
                    last_log_tick = now;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CAN_STATUS_PERIOD_MS));
    }
}

void can_init(void)
{
    if (s_can_started) {
        return;
    }

#if CAN_NO_ACK_MODE
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);
#else
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
#endif
    g_config.tx_queue_len = CAN_TX_QUEUE_LEN;
    g_config.rx_queue_len = CAN_RX_QUEUE_LEN;
    g_config.alerts_enabled = TWAI_ALERT_TX_SUCCESS |
                              TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_RX_QUEUE_FULL |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_BUS_ERROR |
                              TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED;

    twai_timing_config_t t_config = can_get_timing_config();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "TWAI driver already installed");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    err = twai_start();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "TWAI already running");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        (void)twai_driver_uninstall();
        return;
    }

    s_can_started = true;
    ESP_LOGI(TAG,
             "TWAI ready tx=%d rx=%d bitrate=%d mode=%s node=%u cmd_id=0x%03X resp_id=0x%03X",
             (int)CAN_TX_GPIO,
             (int)CAN_RX_GPIO,
             CAN_BITRATE,
             can_mode_str(),
             (unsigned)CAN_NODE_ID,
             (unsigned)(CAN_CMD_ID_BASE + (CAN_NODE_ID & 0x7FU)),
             (unsigned)(CAN_RESP_ID_BASE + (CAN_NODE_ID & 0x7FU)));
}

void can_start(void)
{
    can_init();
    if (!s_can_started || s_can_tasks_started) {
        return;
    }

    BaseType_t ok_rx = xTaskCreatePinnedToCore(can_receive_task, "can_rx", 4096, NULL, 6, NULL, CORE_COMM);
    BaseType_t ok_alert = xTaskCreatePinnedToCore(can_alert_task, "can_alert", 3072, NULL, 6, NULL, CORE_COMM);
    BaseType_t ok_status = xTaskCreatePinnedToCore(can_status_task, "can_status", 3072, NULL, 5, NULL, CORE_COMM);

    if (ok_rx != pdPASS || ok_alert != pdPASS || ok_status != pdPASS) {
        ESP_LOGE(TAG, "Failed to create one or more CAN tasks");
        return;
    }

    s_can_tasks_started = true;
}

bool can_is_started(void)
{
    return s_can_started;
}

esp_err_t can_send_raw(uint32_t identifier, const uint8_t *data, uint8_t len)
{
    if (!s_can_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > TWAI_FRAME_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t msg = {0};
    msg.identifier = identifier;
    msg.extd = (identifier > TWAI_STD_ID_MASK);
    msg.data_length_code = len;

    if (len > 0 && data != NULL) {
        memcpy(msg.data, data, len);
    }

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        s_tx_frames++;
    } else {
        s_tx_failures++;
    }
    return err;
}

esp_err_t can_send_heartbeat_now(void)
{
    return can_send_status_frame_internal();
}

esp_err_t can_request_recovery(void)
{
    if (!s_can_started) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_status_info_t info = {0};
    esp_err_t err = twai_get_status_info(&info);
    if (err != ESP_OK) {
        return err;
    }
    if (info.state != TWAI_STATE_BUS_OFF) {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_initiate_recovery();
}

esp_err_t can_get_runtime_stats(can_runtime_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_housekeeping();
    memset(out, 0, sizeof(*out));
    out->started = s_can_started;
    out->tx_frames = s_tx_frames;
    out->rx_frames = s_rx_frames;
    out->tx_failures = s_tx_failures;
    out->rx_timeouts = s_rx_timeouts;
    out->alert_events = s_alert_events;
    out->last_alerts = s_last_alerts;
    out->heartbeat_frames = s_heartbeat_frames;
    out->last_state_flags = s_last_state_flags;
    out->last_protocol_error = s_last_protocol_error;
    out->robot_state = (uint8_t)robot_get_system_state();
    out->prepared_slot = can_slot_to_wire(can_get_prepared_slot());
    out->active_slot = can_slot_to_wire(can_get_active_slot());
    out->upload_active = s_upload.active;
    out->program_running = robot_is_program_running();
    out->upload_size_bytes = s_upload.expected_size;
    out->upload_received_bytes = s_upload.received_size;

    if (!s_can_started) {
        out->driver_status.state = TWAI_STATE_STOPPED;
        return ESP_OK;
    }

    esp_err_t err = twai_get_status_info(&out->driver_status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "twai_get_status_info failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "TWAI stats state=%s tx=%" PRIu32 " rx=%" PRIu32 " proto=%s",
             can_state_str(out->driver_status.state),
             out->tx_frames,
             out->rx_frames,
             can_protocol_result_to_str((can_protocol_result_t)out->last_protocol_error));
    return ESP_OK;
}
