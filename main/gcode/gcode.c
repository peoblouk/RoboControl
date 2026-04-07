// ===============================
// gcode.c
// ===============================

#include "gcode.h"

#include "esp_log.h"
#include <ctype.h>
#include <errno.h>

static const char *TAG = "gcode";

static volatile bool s_stop = false;
static volatile bool s_error = false;
static gcode_state_t st;

static float to_mm(float v) { return st.units_mm ? v : (v * 25.4f); }

void gcode_set_current_position(float x, float y, float z, float pitch_deg)
{
    st.x = x;
    st.y = y;
    st.z = z;
    st.pitch_deg = pitch_deg;
    st.pose_known = true;
}

bool gcode_sync_to_robot_pose(void)
{
    robot_pose_t pose;
    if (!robot_get_tcp_estimate_work(&pose)) {
        return false;
    }

    gcode_set_current_position(pose.x, pose.y, pose.z, pose.pitch_deg);
    return true;
}

void gcode_reset(void)
{
    st.absolute = true;
    st.units_mm = true;
    st.pose_known = false;
    st.feed_mm_s = 50.0f;
    st.pitch_deg = ROBOT_DEFAULT_PITCH_DEG;
    st.x = st.y = st.z = 0.0f;
    s_stop = false;
    s_error = false;

    if (gcode_sync_to_robot_pose()) {
        ESP_LOGI(TAG, "G-code synced to robot pose: x=%.1f y=%.1f z=%.1f pitch=%.1f",
                 st.x, st.y, st.z, st.pitch_deg);
    } else {
        ESP_LOGW(TAG, "G-code state reset without TCP sync (pose unknown)");
    }
}

void gcode_stop(void)
{
    s_stop = true;
    robot_cmd_queue_flush();
}

void gcode_fail_external(const char *msg)
{
    if (msg && msg[0] != '\0') {
        ESP_LOGE(TAG, "%s", msg);
    }
    s_error = true;
    s_stop = true;
}

static void strip_comment(char *s)
{
    char *c = strchr(s, ';');
    if (c) *c = '\0';

    size_t n = strlen(s);
    while (n && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
}

static bool parse_word(const char *p, char key, float *out)
{
    const char *k = strchr(p, key);
    if (!k) return false;
    *out = strtof(k + 1, NULL);
    return true;
}

static void gcode_set_feed_from_word(float f_word)
{
    float v_mm_min = to_mm(f_word);
    st.feed_mm_s = v_mm_min / 60.0f;
    if (!isfinite(st.feed_mm_s) || st.feed_mm_s < MIN_V_MM_S) {
        st.feed_mm_s = MIN_V_MM_S;
    }
}

static float compute_move_duration_s(float x0, float y0, float z0,
                                     float x1, float y1, float z1,
                                     float v_mm_s)
{
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float dz = z1 - z0;
    const float dist = sqrtf(dx * dx + dy * dy + dz * dz);

    if (!isfinite(v_mm_s) || v_mm_s < MIN_V_MM_S) v_mm_s = MIN_V_MM_S;
    if (dist < 1e-4f) return MIN_SEG_T;
    return dist / v_mm_s;
}

static TickType_t timeout_from_duration_s(float duration_s, float min_timeout_s, float extra_timeout_s)
{
    if (!isfinite(duration_s) || duration_s < 0.0f) duration_s = 0.0f;
    float timeout_s = duration_s + extra_timeout_s;
    if (timeout_s < min_timeout_s) timeout_s = min_timeout_s;

    uint32_t timeout_ms = (uint32_t)lroundf(timeout_s * 1000.0f);
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    return (timeout > 0) ? timeout : 1;
}

static bool send_xyz_blocking(float x, float y, float z, float pitch_deg, float duration_s, TickType_t timeout)
{
    TickType_t t0 = xTaskGetTickCount();

    while (!s_stop && (xTaskGetTickCount() - t0) < timeout) {
        if (robot_cmd_move_xyz_work_t(x, y, z, pitch_deg, duration_s, 0)) {
            const TickType_t settle = pdMS_TO_TICKS(60);
            const TickType_t t_cmd = xTaskGetTickCount();

            vTaskDelay(settle);
            while (!s_stop && (xTaskGetTickCount() - t_cmd) < timeout) {
                if (!robot_is_operating()) {
                    return true;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return false;
}

static bool send_dwell_blocking(uint32_t dwell_ms, TickType_t timeout)
{
    TickType_t t0 = xTaskGetTickCount();

    while (!s_stop && (xTaskGetTickCount() - t0) < timeout) {
        if (robot_cmd_dwell_ms(dwell_ms, 0)) {
            const TickType_t settle = pdMS_TO_TICKS(20);
            const TickType_t t_cmd = xTaskGetTickCount();

            vTaskDelay(settle);
            while (!s_stop && (xTaskGetTickCount() - t_cmd) < timeout) {
                if (!robot_is_operating()) {
                    return true;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return false;
}

static bool send_gripper_blocking(float gripper_deg, TickType_t timeout)
{
    TickType_t t0 = xTaskGetTickCount();

    while (!s_stop && (xTaskGetTickCount() - t0) < timeout) {
        if (robot_cmd_gripper_set(gripper_deg, 0)) {
            const TickType_t settle = pdMS_TO_TICKS(20);
            const TickType_t t_cmd = xTaskGetTickCount();

            vTaskDelay(settle);
            while (!s_stop && (xTaskGetTickCount() - t_cmd) < timeout) {
                if (!robot_is_operating()) {
                    return true;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return false;
}

static bool gcode_abort(const char *msg)
{
    ESP_LOGE(TAG, "%s", msg);
    s_error = true;
    s_stop = true;
    robot_cmd_queue_flush();
    return false;
}

bool gcode_push_line(const char *line_in)
{
    char line[200];
    strncpy(line, line_in, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    strip_comment(line);
    if (line[0] == '\0') return true;

    for (char *c = line; *c != '\0'; ++c) {
        *c = (char)toupper((unsigned char)*c);
    }

    float g = -1.0f, m = -1.0f, f = 0.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    float p = 0.0f, s = 0.0f;

    bool hasG = parse_word(line, 'G', &g);
    bool hasM = parse_word(line, 'M', &m);
    bool hasF = parse_word(line, 'F', &f);
    bool hasX = parse_word(line, 'X', &x);
    bool hasY = parse_word(line, 'Y', &y);
    bool hasZ = parse_word(line, 'Z', &z);
    bool hasP = parse_word(line, 'P', &p);
    bool hasS = parse_word(line, 'S', &s);

    if (hasG && hasM) {
        return gcode_abort("Use one command per line (either G or M).");
    }

    if (hasF && !hasG && !hasM) {
        gcode_set_feed_from_word(f);
        return true;
    }

    if (hasM) {
        const int mi = (int)lroundf(m);

        if (mi == 2 || mi == 30) {
            s_stop = true;
            return true;
        }

        if (mi == 10) {
            if (!send_gripper_blocking(GRIPPER_OPEN_DEG, pdMS_TO_TICKS(5000))) {
                if (s_error || s_stop) return false;
                return gcode_abort("Queue timeout sending GRIPPER OPEN");
            }
            return true;
        }

        if (mi == 11) {
            if (!send_gripper_blocking(GRIPPER_CLOSE_DEG, pdMS_TO_TICKS(5000))) {
                if (s_error || s_stop) return false;
                return gcode_abort("Queue timeout sending GRIPPER CLOSE");
            }
            return true;
        }

        if (mi == 280) {
            if (!hasS) {
                return gcode_abort("M280 requires S (gripper angle in degrees).");
            }
            if (!send_gripper_blocking(s, pdMS_TO_TICKS(5000))) {
                if (s_error || s_stop) return false;
                return gcode_abort("Queue timeout sending GRIPPER S-angle");
            }
            return true;
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "Unsupported M-code M%d", mi);
        return gcode_abort(msg);
    }

    if (!hasG) return true;

    int gi = (int)lroundf(g);
    if (gi == 90) { st.absolute = true; return true; }
    if (gi == 91) { st.absolute = false; return true; }
    if (gi == 20) { st.units_mm = false; return true; }
    if (gi == 21) { st.units_mm = true; return true; }
    if (gi == 4) {
        float dwell_ms = 0.0f;
        if (hasP) dwell_ms = p;
        else if (hasS) dwell_ms = s * 1000.0f;
        else return gcode_abort("G4 requires P (ms) or S (s).");

        if (dwell_ms < 0.0f) return gcode_abort("G4 dwell must be non-negative.");
        if (dwell_ms < 0.5f) return true;

        TickType_t dwell_timeout = timeout_from_duration_s(dwell_ms / 1000.0f, 8.0f, 2.0f);
        if (!send_dwell_blocking((uint32_t)lroundf(dwell_ms), dwell_timeout)) {
            if (s_error || s_stop) return false;
            return gcode_abort("Queue timeout sending DWELL");
        }
        return true;
    }

    if (gi == 0 || gi == 1) {
        if (!robot_is_referenced()) {
            return gcode_abort("Robot is not referenced. Run HOME/reference first.");
        }
        if (!st.pose_known) {
            return gcode_abort("Robot TCP pose is unknown. Use HOME or gcode sync before motion.");
        }

        if (hasF) {
            gcode_set_feed_from_word(f);
        }

        float pitch_deg = hasP ? p : st.pitch_deg;
        float tx = st.x, ty = st.y, tz = st.z;
        if (hasX) tx = st.absolute ? to_mm(x) : (st.x + to_mm(x));
        if (hasY) ty = st.absolute ? to_mm(y) : (st.y + to_mm(y));
        if (hasZ) tz = st.absolute ? to_mm(z) : (st.z + to_mm(z));

        if (!robot_tcp_reachable_work(tx, ty, tz, pitch_deg)) {
            char msg[96];
            snprintf(msg, sizeof(msg), "Unreachable WORK XYZ: x=%.2f y=%.2f z=%.2f pitch=%.2f",
                     tx, ty, tz, pitch_deg);
            return gcode_abort(msg);
        }

        const float v_mm_s = (gi == 0) ? RAPID_MM_S : st.feed_mm_s;
        const float duration_s = compute_move_duration_s(st.x, st.y, st.z, tx, ty, tz, v_mm_s);

        TickType_t move_timeout = timeout_from_duration_s(duration_s, 8.0f, 8.0f);
        if (!send_xyz_blocking(tx, ty, tz, pitch_deg, duration_s, move_timeout)) {
            if (s_error || s_stop) return false;
            return gcode_abort("Queue timeout sending WORK XYZ");
        }

        st.x = tx;
        st.y = ty;
        st.z = tz;
        st.pitch_deg = pitch_deg;
        return true;
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "Unsupported G-code G%d", gi);
    return gcode_abort(msg);
}

bool gcode_run_file(const char *filename)
{
    gcode_reset();
    s_stop = false;
    s_error = false;

    if (!robot_is_referenced()) {
        return gcode_abort("Cannot run G-code: robot is not referenced");
    }

    if (!robot_has_tcp_estimate()) {
        return gcode_abort("Cannot run G-code: current TCP pose is unknown");
    }

    if (!filename || filename[0] == '\0') {
        return gcode_abort("Empty filename");
    }

    char path1[256];
    char path2[256];
    FILE *fp = NULL;

    if (filename[0] == '/') {
        snprintf(path1, sizeof(path1), "%s", filename);
        fp = fopen(path1, "r");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot open '%s' (errno=%d: %s)", path1, errno, strerror(errno));
            return false;
        }
    } else {
        snprintf(path1, sizeof(path1), "%s/%s", FILE_STORAGE_PATH, filename);
        fp = fopen(path1, "r");

        if (!fp) {
            snprintf(path2, sizeof(path2), "/spiffs/%s", filename);
            fp = fopen(path2, "r");

            if (!fp) {
                ESP_LOGE(TAG, "Cannot open '%s' nor '%s' (errno=%d: %s)",
                         path1, path2, errno, strerror(errno));
                return false;
            } else {
                ESP_LOGW(TAG, "Opened via fallback path: %s", path2);
            }
        }
    }

    char line[200];
    while (!s_stop && fgets(line, sizeof(line), fp)) {
        if (!gcode_push_line(line)) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    fclose(fp);
    return !s_error;
}
