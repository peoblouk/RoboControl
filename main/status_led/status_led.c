#include "status_led.h"

static const char *TAG = "status_led";
static led_strip_handle_t s_led_strip = NULL;

static robot_led_state_t status_led_get_state(void)
{
    if (!robot_is_armed()) return ROBOT_LED_DISARM;
    if (robot_is_operating()) return ROBOT_LED_OPERATING;
    return ROBOT_LED_ARMED;
}

static esp_err_t status_led_apply_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_strip == NULL) return ESP_ERR_INVALID_STATE;

    for (int i = 0; i < STATUS_LED_COUNT; i++) {
        esp_err_t err = led_strip_set_pixel(s_led_strip, i, r, g, b);
        if (err != ESP_OK) return err;
    }
    return led_strip_refresh(s_led_strip);
}

static void status_led_task(void *arg)
{
    (void)arg;

    robot_led_state_t state = ROBOT_LED_DISARM;
    robot_led_state_t prev_state = (robot_led_state_t)(-1);
    bool blink_on = true;
    TickType_t last_blink_tick = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        state = status_led_get_state();

        bool refresh_needed = false;

        if (state != prev_state) {
            prev_state = state;
            blink_on = true;
            last_blink_tick = now;
            refresh_needed = true;
        }

        if (state == ROBOT_LED_OPERATING) {
            if ((now - last_blink_tick) >= pdMS_TO_TICKS(STATUS_LED_OPER_BLINK_MS)) {
                blink_on = !blink_on;
                last_blink_tick = now;
                refresh_needed = true;
            }
        } else if (!blink_on) {
            blink_on = true;
            refresh_needed = true;
        }

        if (refresh_needed) {
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;

            if (state == ROBOT_LED_DISARM) {
                r = STATUS_LED_DISARM_R;
                g = STATUS_LED_DISARM_G;
                b = STATUS_LED_DISARM_B;
            } else if (state == ROBOT_LED_ARMED) {
                r = STATUS_LED_ARM_R;
                g = STATUS_LED_ARM_G;
                b = STATUS_LED_ARM_B;
            } else if (state == ROBOT_LED_OPERATING) {
                if (blink_on) {
                    r = STATUS_LED_OPER_R;
                    g = STATUS_LED_OPER_G;
                    b = STATUS_LED_OPER_B;
                }
            }

            esp_err_t err = status_led_apply_color(r, g, b);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "LED refresh failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_TASK_PERIOD_MS));
    }
}

void status_led_start(void)
{
    if (s_led_strip != NULL) return;

    led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_WS2812_GPIO,
        .max_leds = STATUS_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = STATUS_LED_RMT_RES_HZ,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = STATUS_LED_USE_DMA,
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED init failed on GPIO %d: %s", STATUS_LED_WS2812_GPIO, esp_err_to_name(err));
        return;
    }

    err = led_strip_clear(s_led_strip);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED clear failed: %s", esp_err_to_name(err));
    }

    BaseType_t res = xTaskCreatePinnedToCore(status_led_task,
                                             "status_led",
                                             3072,
                                             NULL,
                                             1,
                                             NULL,
                                             CORE_COMM);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status LED task");
        s_led_strip = NULL;
        return;
    }

    ESP_LOGI(TAG, "Status LED started on GPIO %d", STATUS_LED_WS2812_GPIO);
}