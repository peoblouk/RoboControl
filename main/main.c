// ===============================
// main.c
// ===============================

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "wifi_server.h"    // Wi-Fi and Servo Control Server
#include "robot_io.h"       // Robotic Arm Kinematics
#include "core_config.h"    // Core Configuration
#include "cmd_control.h"    // Command Control
#include "rt_stats.h"       // Real-time statistics

//! UART for receiving commands from another ESP32
//#include "uart_receive.h" // UART Receive
//void uart_cmd_task(void *arg);

static void init_spiffs(void); // Initialize SPIFFS (File System)

void app_main(void)
{
    init_spiffs();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    servos_init();             // Initialize servos
    sensors_init();            // Initialize sensors

    wifi_servo_server_start(); // Start Wi-Fi and Servo Control Server
    robot_control_start();     // Start robot control task
    cmd_control_start();       // Start command control task

    //! UART for receiving commands from another ESP32
    //uart1_init(); // Initialize UART1
    //xTaskCreate(uart_cmd_task, "uart_cmd_task", 4096, NULL, 5, NULL);
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