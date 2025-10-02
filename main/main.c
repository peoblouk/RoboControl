// ===============================
// main.c
// ===============================

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "wifi_server.h" // Wi-Fi and Servo Control Server
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
    uart1_init(); // Initialize UART1

    xTaskCreate(uart_cmd_task, "uart_cmd_task", 4096, NULL, 5, NULL);
}


// ===============================
// INIT SPIFFS
// ===============================
static void init_spiffs(void) {
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
// ===============================
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