// ===============================
// main.c
// ===============================

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "wifi_servo_server.h" // Wi-Fi and Servo Control Server
#include "robot_io.h"          // Robotic Arm Kinematics
#include "esp_spiffs.h"
#include <string.h>

static void init_spiffs(void); // Initialize SPIFFS (File System)
void uart1_init(void);
void uart_cmd_task(void *arg);

void app_main(void)
{
    init_spiffs();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_servo_server_start(); // Start Wi-Fi and Servo Control Server
    
    servos_init(); // Initialize servos
    sensors_init(); // Initialize sensors

    uart1_init();
    xTaskCreate(uart_cmd_task, "uart_cmd_task", 4096, NULL, 5, NULL);
}

static void init_spiffs(void) { // Init File System
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        } 
        else {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
}

/*
// ===============================
// TEST SERVO MOVEMENT
#include "robot_io.h"
static void servo_test_task(void) {
    while (1) {
        servo_set_angle(0, 180);
        vTaskDelay(pdMS_TO_TICKS(2000));
        servo_set_angle(0, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
*/


// ===============================
// UART COMMAND INTERFACE TASK
#include "driver/uart.h"

static const char *TAG = "MAIN_UART";

#define UART_PORT_NUM      UART_NUM_1
#define UART_TX_PIN        GPIO_NUM_10   // libovolný pin s funkcí TX
#define UART_RX_PIN        GPIO_NUM_9   // libovolný pin s funkcí RX
#define UART_BAUD_RATE     115200
#define BUF_SIZE           256

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

void uart_cmd_task(void *arg) {
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE-1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "RX: %s", (char*)data);

            // --- Parsování příkazů ---
            if (strncmp((char*)data, "SERVO", 5) == 0) {
                int id; float angle;
                if (sscanf((char*)data, "SERVO %d %f", &id, &angle) == 2) {
                    servo_set_angle(id, angle);
                    char resp[64];
                    snprintf(resp, sizeof(resp), "OK SERVO %d %.1f\n", id, angle);
                    uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
                }
            }
            else if (strncmp((char*)data, "MOVE", 4) == 0) {
                float x, y, z;
                if (sscanf((char*)data, "MOVE %f %f %f", &x, &y, &z) == 3) {
                    float q_target[SERVO_COUNT];
                    inverse_kinematics(x, y, z, q_target);
                    move_to_position(q_target);

                    uart_write_bytes(UART_PORT_NUM, "OK MOVE\n", 8);
                }
            }
            else if (strncmp((char*)data, "SENSORS?", 8) == 0) {
                char resp[128];
                int offset = 0;
                offset += snprintf(resp + offset, sizeof(resp) - offset, "SENSORS ");
                for (int i = 0; i < SENSOR_COUNT; i++) {
                    offset += snprintf(resp + offset, sizeof(resp) - offset,
                                        "%d:%.1f ", i, sensor_read_angle(i));
                }
                offset += snprintf(resp + offset, sizeof(resp) - offset, "\n");
                uart_write_bytes(UART_PORT_NUM, resp, strlen(resp));
            }
            else {
                uart_write_bytes(UART_PORT_NUM, "ERR Unknown command\n", 20);
            }
        }
    }
}