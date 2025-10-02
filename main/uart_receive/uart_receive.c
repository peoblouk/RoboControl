#include "uart_receive.h"

static const char *TAG = "uart_receive";
// ===============================
// UART INIT
// ===============================
void uart1_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Driver + buffer
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE*2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART initialized on TX pin %d, RX pin %d", UART_TX_PIN, UART_RX_PIN);
}

// ===============================
// UART COMMAND TASK
// ===============================
#define LINE_BUF_SIZE 128
static char line_buf[LINE_BUF_SIZE];
static int line_pos = 0;

void uart_cmd_task(void *arg) {
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE-1, pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];

                if (c == '\n' || c == '\r') {
                    // konec příkazu -> zpracuj
                    line_buf[line_pos] = '\0';
                    if (line_pos > 0) {
                        ESP_LOGI(TAG, "UART CMD: %s", line_buf);

                        // --- SERVO ---
                        if (strncmp(line_buf, "SERVO", 5) == 0) {
                            int id; float angle;
                            if (sscanf(line_buf, "SERVO %d %f", &id, &angle) == 2) {
                                servo_set_angle(id, angle);
                                char resp[64];
                                snprintf(resp, sizeof(resp), "OK SERVO %d %.1f\n", id, angle);
                                uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
                            }
                        }
                        // --- MOVE ---
                        else if (strncmp(line_buf, "MOVE", 4) == 0) {
                            float x, y, z;
                            if (sscanf(line_buf, "MOVE %f %f %f", &x, &y, &z) == 3) {
                                float q_target[SERVO_COUNT];
                                inverse_kinematics(x, y, z, q_target);
                                move_to_position(q_target);
                                uart_write_bytes(UART_PORT_NUM, "OK MOVE\n", 8);
                            }
                        }
                        // --- SENSORS ---
                        else if (strcmp(line_buf, "SENSORS") == 0) {
                            char resp[128];
                            int offset = snprintf(resp, sizeof(resp), "SENSORS ");
                            for (int j = 0; j < SENSOR_COUNT; j++) {
                                offset += snprintf(resp + offset, sizeof(resp) - offset,
                                                    "%d:%.1f ", j, sensor_read_angle(j));
                            }
                            snprintf(resp + offset, sizeof(resp) - offset, "\n");
                            uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
                        }
                        else {
                            uart_write_bytes(UART_PORT_NUM, "ERR Unknown command\n", 20);
                        }
                    }
                    line_pos = 0; // reset bufferu
                }
                else {
                    if (line_pos < LINE_BUF_SIZE-1) {
                        line_buf[line_pos++] = c;
                    }
                }
            }
        }
    }
}