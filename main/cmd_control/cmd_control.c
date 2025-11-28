// ===============================
// cmd_control.c
// ===============================

#include "cmd_control.h"

static const char *TAG = "cmd_control";


// ===============================
// COMMAND HANDLERS 
// ===============================
static int cmd_servo(int argc, char **argv) // servo <id> <angle>
{
    if (argc != 3) {
        printf("Usage: servo <id> <angle>\n");
        return 0;
    }

    int   id    = atoi(argv[1]);
    float angle = strtof(argv[2], NULL);

    servo_set_angle(id, angle);
    printf("OK: Servo %d -> %.1f°\n", id, angle);
    return 0;
}

static int cmd_move(int argc, char **argv) // move <x> <y> <z>
{
    if (argc != 4) {
        printf("Usage: move <x> <y> <z>\n");
        return 0;
    }

    float x = strtof(argv[1], NULL);
    float y = strtof(argv[2], NULL);
    float z = strtof(argv[3], NULL);

    if (robot_cmd_move_xyz(x, y, z)) {
        printf("OK: Move to X=%.1f Y=%.1f Z=%.1f (queued)\n", x, y, z);
    } else {
        printf("ERR: Robot queue full or not started\n");
    }
    return 0;
}

static int cmd_sensors(int argc, char **argv) // sensors
{
    (void)argc;
    (void)argv;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        printf("Sensor %d: %.1f°\n", i, sensor_read_angle(i));
    }
    return 0;
}

static int cmd_test(int argc, char **argv) // test
{
    (void)argc;
    (void)argv;

    int ok = 1;

    ok &= robot_cmd_move_xyz(100.0,   0.0,  80.0);
    ok &= robot_cmd_move_xyz( 80.0,  80.0,  80.0);
    ok &= robot_cmd_move_xyz( 80.0, -80.0,  80.0);
    ok &= robot_cmd_move_xyz(100.0,   0.0,  80.0);

    if (ok) {
        printf("OK: Test sequence queued (4 positions)\n");
    } else {
        printf("ERR: Failed to queue all test moves\n");
    }
    return 0;
}

// ===============================
// COMMAND REGISTRATION
// ===============================
static void register_commands(void)
{
    const esp_console_cmd_t servo_cmd = {
        .command  = "servo",
        .help     = "Set servo angle: servo <id> <angle>",
        .hint     = NULL,
        .func     = &cmd_servo,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&servo_cmd));

    const esp_console_cmd_t move_cmd = {
        .command  = "move",
        .help     = "Move to XYZ: move <x> <y> <z>",
        .hint     = NULL,
        .func     = &cmd_move,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&move_cmd));

    const esp_console_cmd_t sensors_cmd = {
        .command  = "sensors",
        .help     = "Print joint angles from sensors",
        .hint     = NULL,
        .func     = &cmd_sensors,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sensors_cmd));

    const esp_console_cmd_t test_cmd = {
        .command  = "test",
        .help     = "Run test motion sequence",
        .hint     = NULL,
        .func     = &cmd_test,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&test_cmd));
}

// ===============================
// UART CONSOLE (getchar)
// ===============================
static void console_task(void *arg)
{
    (void)arg;
    // Unbuffered I/O for immediate echo
    setvbuf(stdin,  NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    esp_console_config_t console_cfg = {
        .max_cmdline_args   = 8,
        .max_cmdline_length = CMD_BUF_SIZE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .hint_color = 0,
        .hint_bold  = false,
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_cfg));

    register_commands();

    ESP_LOGI(TAG, "console_task running on core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Console ready. Type 'help' to list commands.");

    char buf[CMD_BUF_SIZE];
    int  pos = 0;

    printf("> ");

    while (1) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ENTER (CR or LF)
        if (c == '\r' || c == '\n') {
            putchar('\n');
            buf[pos] = '\0';

            if (pos > 0) {
                int ret = 0;
                esp_err_t err = esp_console_run(buf, &ret);
                if (err == ESP_ERR_NOT_FOUND) {
                    printf("ERR: Unknown command '%s'\n", buf);
                } else if (err == ESP_ERR_INVALID_ARG) {
                    printf("ERR: Parse error\n");
                }
            }

            pos = 0;
            printf("> ");
            continue;
        }

        // BACKSPACE (BS or DEL)
        if (c == 0x08 || c == 0x7F) {
            if (pos > 0) {
                pos--;
                printf("\b \b");  // erase last char on terminal
            }
            continue;
        }

        // Printable characters only
        if (c >= 32 && c < 127) {
            if (pos < CMD_BUF_SIZE - 1) {
                buf[pos++] = (char)c;
                putchar(c);
            }
            continue;
        }

        // Other control characters are ignored
    }
}

// ===============================
// PUBLIC API
// ===============================
void cmd_control_start(void)
{
    BaseType_t res = xTaskCreatePinnedToCore(
        console_task,
        "console_task",
        4096,
        NULL,
        5,
        NULL,
        CORE_ROBOT
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create console_task");
    } else {
        ESP_LOGI(TAG, "console_task started on core %d", CORE_ROBOT);
    }
}