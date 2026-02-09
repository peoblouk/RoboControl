// ===============================
// cmd_control.c
// ===============================

#include "cmd_control.h"

static const char *TAG = "cmd_control";
static rt_stats_t g_sensors_cmd_stats;
static rt_stats_t g_servo_cmd_stats;
static rt_stats_t g_move_cmd_stats;

// ===============================
// COMMAND HANDLERS 
// ===============================
static int cmd_joint(int argc, char **argv) // joint <id> <angle>
{
    if (argc != 3) {
        printf("Usage: joint <id> <angle>\n");
        return 0;
    }

    int   id    = atoi(argv[1]);
    float angle = strtof(argv[2], NULL);

    if (id < 0 || id >= JOINT_COUNT) {
        printf("ERR: invalid id\n");
        return 0;
    }

    float lo = 0.0f, hi = 180.0f;

    if (id == 1) {
        // J1 = servo 1 (master) + servo 2 (follower)
        lo = g_joint_limits[1].min_deg;
        hi = g_joint_limits[1].max_deg;

        if (g_joint_limits[2].min_deg > lo) lo = g_joint_limits[2].min_deg;
        if (g_joint_limits[2].max_deg < hi) hi = g_joint_limits[2].max_deg;
    } else {
        static const int joint_to_servo[JOINT_COUNT] = { 0, 1, 3, 4, 5, 6 };
        int s = joint_to_servo[id];
        lo = g_joint_limits[s].min_deg;
        hi = g_joint_limits[s].max_deg;
    }

    if (angle < lo || angle > hi) {
        printf("ERR: angle out of range (allowed %.1f - %.1f)\n", lo, hi);
        return 0;
    }

    int64_t t_start_us = esp_timer_get_time();
    joint_set_angle(id, angle);
    int64_t t_end_us = esp_timer_get_time();

    int64_t dt_us = t_end_us - t_start_us;
    rt_stats_add_sample(&g_servo_cmd_stats, dt_us);

#ifdef STATS_PRINT
    printf("OK: Joint %d -> %.1f° (time: %lld us)\n", id, angle, (long long)dt_us);
    if (g_servo_cmd_stats.count % 10 == 0) {
        rt_stats_print("JOINT_CMD", &g_servo_cmd_stats);
    }
#endif

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

    // start measuring
    int64_t t_start_us = esp_timer_get_time();
    bool res = robot_cmd_move_xyz(x, y, z);

    // end measuring
    int64_t t_end_us = esp_timer_get_time();
    int64_t dt_us    = t_end_us - t_start_us;
    rt_stats_add_sample(&g_move_cmd_stats, dt_us);

    #ifdef STATS_PRINT
    if (res) {
        printf("OK: Move to X=%.1f Y=%.1f Z=%.1f (queued) (time: %lld us)\n", 
               x, y, z, (long long)dt_us);
    } else {
        printf("ERR: Robot queue full or not started (time: %lld us)\n", 
               (long long)dt_us);
    }

    if (g_move_cmd_stats.count % 10 == 0) {
        rt_stats_print("MOVE_CMD", &g_move_cmd_stats);
    }
    #endif

    return 0;
}

static int cmd_gcode(int argc, char **argv) // gcode <subcommand>
{
    if (argc < 2) {
        printf("Usage:\n");
        printf("  gcode reset\n");
        printf("  gcode stop\n");
        printf("  gcode run <file.gcode>\n");
        printf("  gcode line <G1 ...>\n");
        return 0;
    }

    if (strcmp(argv[1],"reset")==0) { gcode_reset(); return 0; }
    if (strcmp(argv[1],"stop")==0)  { gcode_stop();  return 0; }

    if (strcmp(argv[1], "run") == 0 && argc >= 3) {
            robot_core_run_gcode(argv[2]);
            printf("G-Code execution started in background.\n");
            return 0;
        }

    if (strcmp(argv[1],"line")==0 && argc >= 3) {
        char line[200] = {0};
        for (int i = 2; i < argc; i++) {
            if (strlen(line) + strlen(argv[i]) + 2 >= sizeof(line)) break;
            if (i > 2) strcat(line, " ");
            strcat(line, argv[i]);
        }
        bool ok = gcode_push_line(line);
        printf(ok ? "OK\n" : "ERR\n");
        return 0;
    }

    printf("Unknown gcode subcommand\n");
    return 0;
}

static int cmd_sensors(int argc, char **argv) // sensors
{
    (void)argc;
    (void)argv;

    // start measuring
    int64_t t_start_us = esp_timer_get_time();

    for (int i = 0; i < SENSOR_COUNT; i++) {
        printf("Sensor %d: %.1f°\n", i, sensor_read_angle(i));
    }

    // end measuring
    int64_t t_end_us = esp_timer_get_time();
    int64_t dt_us    = t_end_us - t_start_us;
    rt_stats_add_sample(&g_sensors_cmd_stats, dt_us);

    #ifdef STATS_PRINT
    printf("SENSORS time: %lld us (%.3f ms)\n",
           (long long)dt_us,
           dt_us / 1000.0f);

    if (g_sensors_cmd_stats.count % 20 == 0) {
        rt_stats_print("SENSORS_CMD", &g_sensors_cmd_stats);
    }
    #endif

    return 0;
}

static int cmd_print_file(int argc, char **argv) // print <filename>
{
    if (argc != 2) {
        printf("Usage: print <filename>\n");
        printf("Example: print test.gcode\n");
        return 0;
    }

    const char *filename_arg = argv[1];
    char full_path[128];

    if (filename_arg[0] == '/') {
        snprintf(full_path, sizeof(full_path), "%s", filename_arg);
    } else {
        snprintf(full_path, sizeof(full_path), "%s/%s", FILE_STORAGE_PATH, filename_arg);
    }

    FILE *file = fopen(full_path, "r");
    if (!file) {
        printf("ERR: Cannot open file '%s'\n", full_path);
        return 0;
    }

    printf("\n=== FILE CONTENT: %s ===\n", full_path);
    char line[128];
    int line_number = 1;

    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\r\n")] = 0;
        printf("%4d: %s\n", line_number, line);
        line_number++;
    }

    printf("=== END OF FILE ===\n\n");
    fclose(file);
    return 0;
}

static int cmd_ls(int argc, char **argv) // ls
{
    const char *path = FILE_STORAGE_PATH; 
    
    DIR *dir = opendir(path);
    if (!dir) {
        printf("ERR: Cannot open directory '%s'. Is SPIFFS mounted?\n", path);
        return 0;
    }

    printf("\nListing directory: %s\n", path);
    printf("----------------------------------------\n");
    printf("%-24s | %s\n", "Filename", "Size (bytes)");
    printf("----------------------------------------\n");

    struct dirent *ent;
    struct stat st;
    char fullpath[500];
    int count = 0;

    while ((ent = readdir(dir)) != NULL) {
        snprintf(fullpath, sizeof(fullpath), "%s/%s", path, ent->d_name);
        
        if (stat(fullpath, &st) == 0) {
            printf("%-24s | %ld\n", ent->d_name, (long)st.st_size);
        } else {
            printf("%-24s | ?\n", ent->d_name);
        }
        count++;
    }
    
    closedir(dir);
    printf("----------------------------------------\n");
    printf("Total files: %d\n\n", count);
    return 0;
}

static int cmd_stats(int argc, char **argv) // stats
{
    (void)argc;
    (void)argv;

    if (g_sensors_cmd_stats.count == 0) {
        printf("SENSORS_CMD: no samples yet. Run 'sensors' a few times.\n");
        return 0;
    }
    if (g_servo_cmd_stats.count == 0) {
        printf("SERVO_CMD: no samples yet. Run 'servo' a few times.\n");
        return 0;
    }
    if (g_move_cmd_stats.count == 0) {
        printf("MOVE_CMD: no samples yet. Run 'move' a few times.\n");
        return 0;
    }

    rt_stats_print("SENSORS_CMD", &g_sensors_cmd_stats);
    rt_stats_print("SERVO_CMD", &g_servo_cmd_stats);
    rt_stats_print("MOVE_CMD", &g_move_cmd_stats);
    return 0;
}

/////
static int cmd_tasks(int argc, char **argv) // tasks
{
    (void)argc;
    (void)argv;

    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();

    TaskStatus_t *ts = malloc(num_tasks * sizeof(TaskStatus_t));
    if (!ts) {
        printf("ERR: malloc failed\n");
        return 0;
    }

    uint32_t total_run_time = 0;
    num_tasks = uxTaskGetSystemState(ts, num_tasks, &total_run_time);

    printf("Name            State Prio Stack Core Num\n");

    for (UBaseType_t i = 0; i < num_tasks; i++) {
        char state_ch;
        switch (ts[i].eCurrentState) {
            case eRunning:   state_ch = 'R'; break;
            case eReady:     state_ch = 'R'; break;
            case eBlocked:   state_ch = 'B'; break;
            case eSuspended: state_ch = 'S'; break;
            case eDeleted:   state_ch = 'D'; break;
            default:         state_ch = '?'; break;
        }

        BaseType_t core = xTaskGetCoreID(ts[i].xHandle);
        const char *core_str;
        if (core == 0 || core == 1) {
            core_str = (core == 0) ? "0" : "1";
        } 
        else { 
            core_str = "-"; // for unpinned
        }

        printf("%-15s %c     %2u   %5u   %3s  %3u\n",
               ts[i].pcTaskName,
               state_ch,
               (unsigned)ts[i].uxCurrentPriority,
               (unsigned)ts[i].usStackHighWaterMark,
               core_str,
               (unsigned)ts[i].xTaskNumber);
    }

    free(ts);
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
/////

// ===============================
// COMMAND REGISTRATION
// ===============================
static void register_commands(void)
{
    const esp_console_cmd_t servo_cmd = {
        .command  = "joint",
        .help     = "Set joint angle: joint <id> <angle>",
        .hint     = NULL,
        .func     = &cmd_joint,
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

    const esp_console_cmd_t print_cmd = {
        .command  = "print",
        .help     = "Print file content: print <path>",
        .hint     = NULL,
        .func     = &cmd_print_file,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&print_cmd));

    const esp_console_cmd_t ls_cmd = {
        .command  = "ls",
        .help     = "List files in storage",
        .hint     = NULL,
        .func     = &cmd_ls,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ls_cmd));

    const esp_console_cmd_t stats_cmd = {
        .command  = "stats",
        .help     = "Print timing stats for 'sensors' command",
        .hint     = NULL,
        .func     = &cmd_stats,
        .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stats_cmd));

    const esp_console_cmd_t tasks_cmd = {
        .command = "tasks",
        .help    = "Print FreeRTOS task list",
        .hint    = NULL,
        .func    = &cmd_tasks,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tasks_cmd));

    const esp_console_cmd_t gcode_cmd = {
    .command  = "gcode",
    .help     = "G-code: gcode run <file>, gcode line <...>, gcode stop, gcode reset",
    .hint     = NULL,
    .func     = &cmd_gcode,
    .argtable = NULL,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&gcode_cmd));
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
        .max_cmdline_args   = 16,
        .max_cmdline_length = CMD_BUF_SIZE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .hint_color = 0,
        .hint_bold  = false,
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_cfg));

    register_commands();

    ESP_LOGI(TAG, "console_task running on core %d", xPortGetCoreID());

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
        rt_stats_reset(&g_sensors_cmd_stats);
    }
}