// ===============================
// main.c
// ===============================

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <string.h>
#include "wifi_server.h"    // Wi-Fi and Servo Control Server
#include "robot_io.h"       // Robotic Arm Kinematics
//#include "uart_receive.h" // UART Receive
#include "core_config.h"   // Core Configuration

#define CMD_BUF_SIZE 128

static void init_spiffs(void); // Initialize SPIFFS (File System)
void uart_cmd_task(void *arg);
void console_task(void *arg);

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

    //! UART for receiving commands from another ESP32
    //uart1_init(); // Initialize UART1
    //xTaskCreate(uart_cmd_task, "uart_cmd_task", 4096, NULL, 5, NULL);

    //xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(console_task, "console_task", 4096, NULL, 5, NULL, CORE_ROBOT);
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

// ===============================
// CONSOLE TASK
// ===============================
void console_task(void *arg) {
    char buf[CMD_BUF_SIZE];
    int pos = 0;

    while (1) {
        int c = getchar();   // čeká na znak z UART0
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (c == '\r' || c == '\n') {
            putchar('\n');  // odřádkujeme
            buf[pos] = '\0';

            if (pos > 0) {
                // === Parsování příkazů ===
                if (strncmp(buf, "SERVO", 5) == 0) {
                    int id; float angle;
                    if (sscanf(buf, "SERVO %d %f", &id, &angle) == 2) {
                        servo_set_angle(id, angle);
                        printf("OK: Servo %d -> %.1f°\n", id, angle);
                    }
                } else if (strncmp(buf, "MOVE", 4) == 0) {
                    float x,y,z;
                    if (sscanf(buf, "MOVE %f %f %f", &x,&y,&z) == 3) {
                        float q_target[SERVO_COUNT];
                        inverse_kinematics(x,y,z,q_target);
                        move_to_position(q_target);
                        printf("OK: Move to X=%.1f Y=%.1f Z=%.1f\n", x,y,z);
                    }
                } else if (strcmp(buf,"SENSORS")==0) {
                    for (int i=0;i<SENSOR_COUNT;i++) {
                        printf("Sensor %d: %.1f°\n", i, sensor_read_angle(i));
                    }
                } else {
                    printf("ERR: Unknown command '%s'\n", buf);
                }
            }

            pos = 0; // reset bufferu
            printf("> "); // prompt
        }
        else if (c == 0x08 || c == 0x7F) { 
            // Backspace
            if (pos > 0) {
                pos--;
                printf("\b \b");
            }
        }
        else if (pos < CMD_BUF_SIZE-1) {
            buf[pos++] = (char)c;
            putchar(c); // echo znak
        }
    }
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