// ===============================
// main.c
// ===============================

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "wifi_server.h"    // Wi-Fi and Servo Control Server
#include "robot_io.h"       // Robotic Arm Kinematics
#include "config.h"         // Configuration
#include "cmd_control.h"    // Command Control
#include "rt_stats.h"       // Real-time statistics

void test_gpio39_blink(void);

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

    // while (1) {
    //     test_gpio39_blink(); // Blink GPIO39 for testing
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
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

void test_gpio39_blink(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << GPIO_NUM_39,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    while (1) {
        gpio_set_level(GPIO_NUM_39, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(GPIO_NUM_39, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}