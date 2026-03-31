// ===============================
// cmd_control.c
// ===============================

#include "cmd_control.h"

static const char *TAG = "cmd_control";
static rt_stats_t g_sensors_cmd_stats;
static rt_stats_t g_servo_cmd_stats;
static rt_stats_t g_move_cmd_stats;
static esp_console_repl_t *s_repl = NULL;

static void print_robot_state(void)
{
    float wx, wy, wz;
    robot_get_work_offset(&wx, &wy, &wz);

    printf("state=%s last_error=%u\n",
           robot_get_system_state_name(),
           (unsigned)robot_get_last_error());
    printf("armed=%s operating=%s program=%s referenced=%s tcp_est=%s sync_ready=%s\n",
           robot_is_armed() ? "yes" : "no",
           robot_is_operating() ? "yes" : "no",
           robot_is_program_running() ? "yes" : "no",
           robot_is_referenced() ? "yes" : "no",
           robot_has_tcp_estimate() ? "yes" : "no",
           robot_is_sync_ready() ? "yes" : "no");
    printf("work_offset: X=%.1f Y=%.1f Z=%.1f\n", wx, wy, wz);

    robot_pose_t pose;
    if (robot_get_tcp_estimate_work(&pose)) {
        printf("tcp_work_est: X=%.1f Y=%.1f Z=%.1f P=%.1f\n",
               pose.x, pose.y, pose.z, pose.pitch_deg);
    } else {
        printf("tcp_work_est: unknown\n");
    }
}

static const char *twai_state_to_str_(twai_state_t state)
{
    switch (state) {
        case TWAI_STATE_STOPPED: return "stopped";
        case TWAI_STATE_RUNNING: return "running";
        case TWAI_STATE_BUS_OFF: return "bus-off";
        case TWAI_STATE_RECOVERING: return "recovering";
        default: return "unknown";
    }
}

static void print_can_flags_(uint8_t flags)
{
    printf("flags: armed=%s operating=%s referenced=%s tcp_est=%s sync_ready=%s upload=%s program=%s error=%s\n",
           (flags & (1U << 0)) ? "yes" : "no",
           (flags & (1U << 1)) ? "yes" : "no",
           (flags & (1U << 2)) ? "yes" : "no",
           (flags & (1U << 3)) ? "yes" : "no",
           (flags & (1U << 4)) ? "yes" : "no",
           (flags & (1U << 5)) ? "yes" : "no",
           (flags & (1U << 6)) ? "yes" : "no",
           (flags & (1U << 7)) ? "yes" : "no");
}

static int cmd_can(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage:\n");
        printf("  can up\n");
        printf("  can status\n");
        printf("  can hb\n");
        printf("  can probe [count] [gap_ms]\n");
        printf("  can tx <id> [b0 ... b7]\n");
        printf("  can recover\n");
        return 0;
    }

    if (strcmp(argv[1], "up") == 0) {
        if (can_is_started()) {
            printf("OK: CAN already started\n");
            return 0;
        }
        can_start();
        printf(can_is_started() ? "OK: CAN started\n" : "ERR: CAN start failed\n");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        can_runtime_stats_t st = {0};
        esp_err_t err = can_get_runtime_stats(&st);
        if (err != ESP_OK) {
            printf("ERR: cannot read CAN status (%s)\n", esp_err_to_name(err));
            return 0;
        }

        printf("started=%s twai=%s tx_frames=%" PRIu32 " rx_frames=%" PRIu32 " tx_fail=%" PRIu32 " hb=%" PRIu32 "\n",
               st.started ? "yes" : "no",
               twai_state_to_str_(st.driver_status.state),
               st.tx_frames,
               st.rx_frames,
               st.tx_failures,
               st.heartbeat_frames);
        printf("robot_state=%u prepared_slot=%u active_slot=%u upload=%s program=%s upload_bytes=%u/%u proto_err=%s\n",
               (unsigned)st.robot_state,
               (unsigned)st.prepared_slot,
               (unsigned)st.active_slot,
               st.upload_active ? "yes" : "no",
               st.program_running ? "yes" : "no",
               (unsigned)st.upload_received_bytes,
               (unsigned)st.upload_size_bytes,
               can_protocol_result_to_str((can_protocol_result_t)st.last_protocol_error));
        printf("driver: queued_tx=%" PRIu32 " queued_rx=%" PRIu32 " tx_err=%" PRIu32 " rx_err=%" PRIu32 " bus_err=%" PRIu32 "\n",
               st.driver_status.msgs_to_tx,
               st.driver_status.msgs_to_rx,
               st.driver_status.tx_error_counter,
               st.driver_status.rx_error_counter,
               st.driver_status.bus_error_count);
        printf("alerts=%" PRIu32 " last_alerts=0x%08" PRIX32 " rx_timeouts=%" PRIu32 "\n",
               st.alert_events,
               st.last_alerts,
               st.rx_timeouts);
        print_can_flags_(st.last_state_flags);
        return 0;
    }

    if (strcmp(argv[1], "hb") == 0) {
        esp_err_t err = can_send_heartbeat_now();
        if (err == ESP_OK) printf("OK: heartbeat sent\n");
        else printf("ERR: heartbeat failed (%s)\n", esp_err_to_name(err));
        return 0;
    }

    if (strcmp(argv[1], "recover") == 0) {
        esp_err_t err = can_request_recovery();
        if (err == ESP_OK) printf("OK: recovery requested\n");
        else printf("ERR: recovery failed (%s)\n", esp_err_to_name(err));
        return 0;
    }

    if (strcmp(argv[1], "probe") == 0) {
        if (argc > 4) {
            printf("Usage: can probe [count] [gap_ms]\n");
            return 0;
        }

        int count = 5;
        int gap_ms = 20;
        if (argc >= 3) count = atoi(argv[2]);
        if (argc >= 4) gap_ms = atoi(argv[3]);
        if (count < 1 || count > 100) {
            printf("ERR: count must be 1..100\n");
            return 0;
        }
        if (gap_ms < 0 || gap_ms > 2000) {
            printf("ERR: gap_ms must be 0..2000\n");
            return 0;
        }

        can_runtime_stats_t before = {0};
        can_runtime_stats_t after = {0};
        esp_err_t st_err = can_get_runtime_stats(&before);
        if (st_err != ESP_OK) {
            printf("ERR: cannot read CAN status (%s)\n", esp_err_to_name(st_err));
            return 0;
        }

        printf("probe-start: twai=%s tx=%" PRIu32 " fail=%" PRIu32 " tx_err=%" PRIu32 " rx_err=%" PRIu32 " bus_err=%" PRIu32 "\n",
               twai_state_to_str_(before.driver_status.state),
               before.tx_frames,
               before.tx_failures,
               before.driver_status.tx_error_counter,
               before.driver_status.rx_error_counter,
               before.driver_status.bus_error_count);

        const uint32_t probe_id = CAN_RESP_ID_BASE + (CAN_NODE_ID & 0x7FU);
        const TickType_t gap_ticks = pdMS_TO_TICKS((uint32_t)gap_ms);
        int ok = 0;
        for (int i = 0; i < count; i++) {
            uint8_t payload[8] = { 0xC0, (uint8_t)i, (uint8_t)CAN_NODE_ID, 0x55, 0xAA, 0, 0, 0 };
            esp_err_t err = can_send_raw(probe_id, payload, sizeof(payload));
            if (err == ESP_OK) {
                ok++;
                printf("probe %d/%d: OK\n", i + 1, count);
            } else {
                printf("probe %d/%d: ERR (%s)\n", i + 1, count, esp_err_to_name(err));
            }

            if (i + 1 < count) {
                if (gap_ticks > 0) vTaskDelay(gap_ticks);
                else taskYIELD();
            }
        }

        st_err = can_get_runtime_stats(&after);
        if (st_err == ESP_OK) {
            printf("probe-end: ok=%d/%d delta_tx=%" PRIu32 " delta_fail=%" PRIu32 " delta_tx_err=%" PRIu32 " delta_bus_err=%" PRIu32 " twai=%s\n",
                   ok,
                   count,
                   after.tx_frames - before.tx_frames,
                   after.tx_failures - before.tx_failures,
                   after.driver_status.tx_error_counter - before.driver_status.tx_error_counter,
                   after.driver_status.bus_error_count - before.driver_status.bus_error_count,
                   twai_state_to_str_(after.driver_status.state));
        } else {
            printf("probe-end: ok=%d/%d (status unavailable: %s)\n", ok, count, esp_err_to_name(st_err));
        }
        return 0;
    }

    if (strcmp(argv[1], "tx") == 0) {
        if (argc < 3 || argc > 11) {
            printf("Usage: can tx <id> [b0 ... b7]\n");
            return 0;
        }

        uint32_t id = strtoul(argv[2], NULL, 0);
        uint8_t data[8] = {0};
        uint8_t len = (uint8_t)(argc - 3);

        for (int i = 0; i < len; i++) {
            char *end = NULL;
            unsigned long v = strtoul(argv[3 + i], &end, 0);
            if (end == argv[3 + i] || *end != '\0' || v > 0xFFUL) {
                printf("ERR: invalid byte '%s'\n", argv[3 + i]);
                return 0;
            }
            data[i] = (uint8_t)v;
        }

        esp_err_t err = can_send_raw(id, data, len);
        if (err == ESP_OK) printf("OK: frame queued\n");
        else printf("ERR: tx failed (%s)\n", esp_err_to_name(err));
        return 0;
    }

    printf("Unknown can subcommand '%s'\n", argv[1]);
    printf("Try: can status | can probe | can hb | can tx ...\n");
    return 0;
}

static int cmd_joint(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: joint <id> <angle>\n");
        return 0;
    }

    if (!robot_is_armed()) {
        printf("ERR: joint rejected (robot is DISARMED)\n");
        return 0;
    }

    int id = atoi(argv[1]);
    float angle = strtof(argv[2], NULL);

    if (id < 0 || id >= JOINT_COUNT) {
        printf("ERR: invalid id\n");
        return 0;
    }

    float lo = 0.0f, hi = 180.0f;
    if (id == 1) {
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
    if (g_servo_cmd_stats.count % 10 == 0) rt_stats_print("JOINT_CMD", &g_servo_cmd_stats);
#endif
    return 0;
}

static int cmd_move(int argc, char **argv)
{
    if (argc != 4 && argc != 5) {
        printf("Usage: move <x> <y> <z> [pitch]\n");
        return 0;
    }

    if (!robot_is_armed()) {
        printf("ERR: move rejected (robot is DISARMED)\n");
        return 0;
    }

    float x = strtof(argv[1], NULL);
    float y = strtof(argv[2], NULL);
    float z = strtof(argv[3], NULL);
    const bool pitch_explicit = (argc == 5);
    float pitch = pitch_explicit ? strtof(argv[4], NULL) : ROBOT_DEFAULT_PITCH_DEG;

    float ox, oy, oz;
    robot_get_work_offset(&ox, &oy, &oz);
    float xb = x + ox;
    float yb = y + oy;
    float zb = z + oz;
    float q_test[SERVO_COUNT];
    bool ik_ok = robot_ik_tcp(xb, yb, zb, pitch, q_test);

    if (!ik_ok && !pitch_explicit) {
        const float base_pitch = pitch;
        for (int d = 1; d <= 89 && !ik_ok; d++) {
            const float p_up = base_pitch + (float)d;
            if (p_up <= 89.0f) {
                ik_ok = robot_ik_tcp(xb, yb, zb, p_up, q_test);
                if (ik_ok) {
                    pitch = p_up;
                    break;
                }
            }

            const float p_dn = base_pitch - (float)d;
            if (p_dn >= -89.0f) {
                ik_ok = robot_ik_tcp(xb, yb, zb, p_dn, q_test);
                if (ik_ok) {
                    pitch = p_dn;
                    break;
                }
            }
        }

        if (ik_ok) {
            printf("INFO: auto pitch adjusted to %.1f deg for IK\n", pitch);
        }
    }

    if (!ik_ok) {
        printf("ERR: IK TCP no solution for WORK X=%.1f Y=%.1f Z=%.1f pitch=%.1f (BASE X=%.1f Y=%.1f Z=%.1f)\n",
               x, y, z, pitch, xb, yb, zb);
        return 0;
    }

    int64_t t_start_us = esp_timer_get_time();
    bool res = robot_cmd_move_xyz_work(x, y, z, pitch);
    int64_t t_end_us = esp_timer_get_time();
    int64_t dt_us = t_end_us - t_start_us;
    rt_stats_add_sample(&g_move_cmd_stats, dt_us);

#ifdef STATS_PRINT
    if (res) {
        printf("OK: Move(work) to X=%.1f Y=%.1f Z=%.1f queued (pitch %.1f) (time: %lld us)\n", x, y, z, pitch, (long long)dt_us);
    } else {
        printf("ERR: move rejected (not referenced / queue full / not started) (time: %lld us)\n", (long long)dt_us);
    }
    if (g_move_cmd_stats.count % 10 == 0) rt_stats_print("MOVE_CMD", &g_move_cmd_stats);
#endif
    return 0;
}

static int cmd_gcode(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage:\n  gcode reset\n  gcode stop\n  gcode run <file.gcode>\n  gcode line <G1 ...>\n  gcode sync\n");
        return 0;
    }

    if (strcmp(argv[1], "reset") == 0) { gcode_reset(); return 0; }
    if (strcmp(argv[1], "stop") == 0)  { gcode_stop(); return 0; }
    if (strcmp(argv[1], "sync") == 0)  {
        printf(gcode_sync_to_robot_pose() ? "OK: gcode synced to robot pose\n" : "ERR: robot pose unknown\n");
        return 0;
    }
    if (strcmp(argv[1], "run") == 0 && argc >= 3) {
        bool ok = robot_core_run_gcode(argv[2]);
        printf(ok ? "G-Code execution started.\n" : "ERR: G-Code start rejected.\n");
        return 0;
    }
    if (strcmp(argv[1], "line") == 0 && argc >= 3) {
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

static int cmd_sensors(int argc, char **argv)
{
    (void)argc; (void)argv;
    int64_t t_start_us = esp_timer_get_time();

    const int joint_to_servo[JOINT_COUNT] = {
        JOINT0_SERVO, JOINT1_SERVO, JOINT2_SERVO, JOINT3_SERVO, JOINT4_SERVO, JOINT5_SERVO
    };

    bool use_adc_sensors = (SENSOR_COUNT > 0) && robot_sensors_initialized();
    if (use_adc_sensors) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            printf("Sensor %d: %.1f deg\n", i, sensor_read_angle(i));
        }
    } else {
        printf("Sensors unavailable, showing estimated joint angles:\n");
        for (int j = 0; j < JOINT_COUNT; j++) {
            int s = joint_to_servo[j];
            printf("Joint %d (servo %d): %.1f deg\n", j, s, robot_get_est_angle(s));
        }
    }

    int64_t t_end_us = esp_timer_get_time();
    int64_t dt_us = t_end_us - t_start_us;
    rt_stats_add_sample(&g_sensors_cmd_stats, dt_us);
#ifdef STATS_PRINT
    printf("SENSORS time: %lld us (%.3f ms)\n", (long long)dt_us, dt_us / 1000.0f);
    if (g_sensors_cmd_stats.count % 20 == 0) rt_stats_print("SENSORS_CMD", &g_sensors_cmd_stats);
#endif
    return 0;
}

static int cmd_print_file(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: print <filename>\nExample: print test.gcode\n");
        return 0;
    }

    const char *filename_arg = argv[1];
    char full_path[128];
    if (filename_arg[0] == '/') snprintf(full_path, sizeof(full_path), "%s", filename_arg);
    else snprintf(full_path, sizeof(full_path), "%s/%s", FILE_STORAGE_PATH, filename_arg);

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

static int cmd_ls(int argc, char **argv)
{
    (void)argc; (void)argv;
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
        if (stat(fullpath, &st) == 0) printf("%-24s | %ld\n", ent->d_name, (long)st.st_size);
        else printf("%-24s | ?\n", ent->d_name);
        count++;
    }
    closedir(dir);
    printf("----------------------------------------\nTotal files: %d\n\n", count);
    return 0;
}

static int cmd_stats(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (g_sensors_cmd_stats.count == 0) { printf("SENSORS_CMD: no samples yet. Run 'sensors' a few times.\n"); return 0; }
    if (g_servo_cmd_stats.count == 0) { printf("SERVO_CMD: no samples yet. Run 'servo' a few times.\n"); return 0; }
    if (g_move_cmd_stats.count == 0) { printf("MOVE_CMD: no samples yet. Run 'move' a few times.\n"); return 0; }
    rt_stats_print("SENSORS_CMD", &g_sensors_cmd_stats);
    rt_stats_print("SERVO_CMD", &g_servo_cmd_stats);
    rt_stats_print("MOVE_CMD", &g_move_cmd_stats);
    return 0;
}

static int cmd_tasks(int argc, char **argv)
{
    (void)argc; (void)argv;
#if CONFIG_FREERTOS_USE_TRACE_FACILITY
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *ts = malloc(num_tasks * sizeof(TaskStatus_t));
    if (!ts) { printf("ERR: malloc failed\n"); return 0; }
    uint32_t total_run_time = 0;
    num_tasks = uxTaskGetSystemState(ts, num_tasks, &total_run_time);
    (void)total_run_time;

    printf("Name            State Prio Stack Core Num\n");
    for (UBaseType_t i = 0; i < num_tasks; i++) {
        char state_ch;
        switch (ts[i].eCurrentState) {
            case eRunning: case eReady: state_ch = 'R'; break;
            case eBlocked: state_ch = 'B'; break;
            case eSuspended: state_ch = 'S'; break;
            case eDeleted: state_ch = 'D'; break;
            default: state_ch = '?'; break;
        }
        BaseType_t core = xTaskGetCoreID(ts[i].xHandle);
        const char *core_str = (core == 0) ? "0" : (core == 1) ? "1" : "-";
        printf("%-15s %c     %2u   %5u   %3s  %3u\n", ts[i].pcTaskName, state_ch,
               (unsigned)ts[i].uxCurrentPriority, (unsigned)ts[i].usStackHighWaterMark,
               core_str, (unsigned)ts[i].xTaskNumber);
    }
    free(ts);
#else
    printf("ERR: 'tasks' requires CONFIG_FREERTOS_USE_TRACE_FACILITY=y (menuconfig)\n");
#endif
    return 0;
}

static int cmd_test(int argc, char **argv)
{
    (void)argc; (void)argv;
    int ok = 1;
    robot_cmd_queue_flush();

    const float X0 = 0.0f;
    const float Y0 = 0.0f;
    const float Z0 = 0.0f;
    const float ZS = ROBOT_HOME_Z_BASE_DEFAULT - ROBOT_WORK_OFFSET_Z_DEFAULT;
    const float P0 = 0.0f;
    const float DX = 40.0f;
    const float DY = 40.0f;
    const float DZ = 40.0f;
    // const float DP = 20.0f;   // kept for later TCP / pitch tests
    const TickType_t W = pdMS_TO_TICKS(2500);

    ok &= robot_cmd_move_xyz_work(X0, Y0, ZS, P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0, Z0,     P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0, Z0+DZ,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0, Z0,     P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0-DX, Y0, Z0,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0+DX, Y0, Z0,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0,    Y0, Z0,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0+DY, Z0,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0-DY, Z0,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0,    Z0,  P0); vTaskDelay(W);
    // Optional pitch sweep in test sequence (kept disabled by default)
    // ok &= robot_cmd_move_xyz_work(X0, Y0, Z0, +DP); vTaskDelay(W);
    // ok &= robot_cmd_move_xyz_work(X0, Y0, Z0, -DP); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0, Z0,  P0); vTaskDelay(W);
    ok &= robot_cmd_move_xyz_work(X0, Y0, ZS, P0); vTaskDelay(W);

    printf(ok ? "OK: Simple work-axis sweep queued\n" : "ERR: Queue failed\n");
    return 0;
}

static int cmd_disarm(int argc, char **argv)
{
    (void)argc; (void)argv;
    robot_disarm();
    printf("OK: disarmed\n");
    return 0;
}

static int cmd_arm(int argc, char **argv)
{
    (void)argc; (void)argv;
    robot_arm();
    printf("OK: armed\n");
    return 0;
}

static int cmd_home(int argc, char **argv)
{
    (void)argc; (void)argv;
    bool ok = robot_cmd_home_reference();
    printf(ok ? "OK: home queued, reference will be set on finish\n" : "ERR: home not queued\n");
    return 0;
}

static int cmd_wcofs(int argc, char **argv)
{
    if (argc == 1) {
        float x, y, z;
        robot_get_work_offset(&x, &y, &z);
        printf("Work offset: X=%.1f Y=%.1f Z=%.1f\n", x, y, z);
        return 0;
    }
    if (argc != 4) {
        printf("Usage: wcofs <x> <y> <z>\n");
        return 0;
    }
    robot_set_work_offset(strtof(argv[1], NULL), strtof(argv[2], NULL), strtof(argv[3], NULL));
    print_robot_state();
    return 0;
}

static int cmd_state(int argc, char **argv)
{
    (void)argc; (void)argv;
    print_robot_state();
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "joint",  .help = "Set joint angle: joint <id> <angle>", .func = &cmd_joint },
        { .command = "move",   .help = "Move in WORK frame: move <x> <y> <z> [pitch] (pitch omitted => auto nearest IK pitch)", .func = &cmd_move },
        { .command = "sensors",.help = "Print sensor angles or estimated joint angles when sensors are unavailable", .func = &cmd_sensors },
        { .command = "test",   .help = "Run simple XYZ test motion sequence in WORK frame", .func = &cmd_test },
        { .command = "print",  .help = "Print file content: print <path>", .func = &cmd_print_file },
        { .command = "ls",     .help = "List files in storage", .func = &cmd_ls },
        { .command = "stats",  .help = "Print timing stats for control commands", .func = &cmd_stats },
        { .command = "tasks",  .help = "Print FreeRTOS task list", .func = &cmd_tasks },
        { .command = "gcode",  .help = "G-code: gcode run <file>, gcode line <...>, gcode stop, gcode reset, gcode sync", .func = &cmd_gcode },
        { .command = "can",    .help = "CAN/TWAI: can up | can status | can probe [count] [gap_ms] | can hb | can tx <id> [b0..b7] | can recover", .func = &cmd_can },
        { .command = "disarm", .help = "Disable servo PWM outputs", .func = &cmd_disarm },
        { .command = "arm",    .help = "Enable servo PWM outputs", .func = &cmd_arm },
        { .command = "home",   .help = "Move robot to HOME position and establish reference", .func = &cmd_home },
        { .command = "wcofs",  .help = "Show/set work offset: wcofs [x y z]", .func = &cmd_wcofs },
        { .command = "state",  .help = "Show robot reference, work offset and TCP estimate", .func = &cmd_state },
    };

    for (size_t i = 0; i < sizeof(cmds)/sizeof(cmds[0]); i++) {
        esp_console_cmd_t c = cmds[i];
        c.hint = NULL;
        c.argtable = NULL;
        ESP_ERROR_CHECK(esp_console_cmd_register(&c));
    }
}

void cmd_control_start(void)
{
    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_cfg.prompt = ">> ";
    repl_cfg.max_history_len = 1;
    repl_cfg.max_cmdline_length = CMD_BUF_SIZE;
    repl_cfg.task_stack_size = 4096;
    repl_cfg.task_priority = 5;

    esp_err_t err = esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &s_repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create console REPL: %s", esp_err_to_name(err));
        return;
    }

    register_commands();

    linenoiseSetDumbMode(1);
    linenoiseSetMultiLine(0);

    err = esp_console_start_repl(s_repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start console REPL: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "console REPL started on core %d", CORE_COMM);
    rt_stats_reset(&g_sensors_cmd_stats);
}
