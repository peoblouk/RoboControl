// ===============================
// can_communication.c
// ===============================

#include "can_communication.h"

static const char *TAG = "can";

#define CAN_STD_ID_HEARTBEAT_BASE  0x700U

#define CAN_FLAG_ARMED        (1U << 0)
#define CAN_FLAG_OPERATING    (1U << 1)
#define CAN_FLAG_REFERENCED   (1U << 2)
#define CAN_FLAG_TCP_EST      (1U << 3)

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

static uint8_t can_build_state_flags(void)
{
    uint8_t flags = 0;
    if (robot_is_armed()) flags |= CAN_FLAG_ARMED;
    if (robot_is_operating()) flags |= CAN_FLAG_OPERATING;
    if (robot_is_referenced()) flags |= CAN_FLAG_REFERENCED;
    if (robot_has_tcp_estimate()) flags |= CAN_FLAG_TCP_EST;
    return flags;
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

    ESP_LOGI(TAG, "%s id=0x%03" PRIX32 " dlc=%u extd=%u rtr=%u data=[%s]",
             prefix,
             msg->identifier,
             (unsigned)msg->data_length_code,
             (unsigned)msg->extd,
             (unsigned)msg->rtr,
             payload);
}

static esp_err_t can_send_status_frame_internal(void)
{
    twai_status_info_t info = {0};
    (void)twai_get_status_info(&info);

    uint8_t data[8] = {0};
    data[0] = 0xA5;
    data[1] = can_build_state_flags();
    data[2] = (uint8_t)CAN_NODE_ID;
    data[3] = (uint8_t)(s_heartbeat_frames & 0xFFU);
    data[4] = (uint8_t)(info.msgs_to_tx & 0xFFU);
    data[5] = (uint8_t)(info.msgs_to_rx & 0xFFU);
    data[6] = (uint8_t)(info.tx_error_counter & 0xFFU);
    data[7] = (uint8_t)(info.rx_error_counter & 0xFFU);

    esp_err_t err = can_send_raw(CAN_STD_ID_HEARTBEAT_BASE + (CAN_NODE_ID & 0x7FU), data, sizeof(data));
    if (err == ESP_OK) {
        s_last_state_flags = data[1];
        s_heartbeat_frames++;
    }
    return err;
}

static void can_receive_task(void *arg)
{
    (void)arg;
    for (;;) {
        twai_message_t msg = {0};
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(250));
        if (err == ESP_OK) {
            s_rx_frames++;
            can_log_frame("RX", &msg);
        } else if (err == ESP_ERR_TIMEOUT) {
            s_rx_timeouts++;
        } else {
            ESP_LOGW(TAG, "twai_receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static void can_alert_task(void *arg)
{
    (void)arg;
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
            ESP_LOGW(TAG, "TWAI TX failed");
        }
        if (alerts & TWAI_ALERT_TX_SUCCESS) {
            ESP_LOGD(TAG, "TWAI TX success");
        }
    }
}

static void can_status_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_can_started) {
            esp_err_t err = can_send_status_frame_internal();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Status frame TX failed: %s", esp_err_to_name(err));
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
             "TWAI ready tx=%d rx=%d bitrate=%d mode=%s node=%u",
             (int)CAN_TX_GPIO,
             (int)CAN_RX_GPIO,
             CAN_BITRATE,
             can_mode_str(),
             (unsigned)CAN_NODE_ID);
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

    if (!s_can_started) {
        out->driver_status.state = TWAI_STATE_STOPPED;
        return ESP_OK;
    }

    esp_err_t err = twai_get_status_info(&out->driver_status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "twai_get_status_info failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "TWAI stats state=%s tx=%" PRIu32 " rx=%" PRIu32,
             can_state_str(out->driver_status.state), out->tx_frames, out->rx_frames);
    return ESP_OK;
}
