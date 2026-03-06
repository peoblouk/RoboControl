// ===============================
// gcode.c
// ===============================

#include "gcode.h"

#include "esp_log.h"
#include <errno.h>

static const char *TAG = "gcode";

static volatile bool s_stop = false;
static volatile bool s_error = false;
static gcode_state_t st;

static float to_mm(float v) { return st.units_mm ? v : (v * 25.4f); }

void gcode_reset(void)
{
    st.absolute = true;
    st.units_mm = true;
    st.feed_mm_s = 50.0f;
    st.x = st.y = st.z = 0.0f;
    s_stop = false;
    s_error = false;
}

void gcode_stop(void)
{
    s_stop = true;
    robot_cmd_queue_flush();
}

static void strip_comment(char *s)
{
    char *c = strchr(s, ';');
    if (c) *c = 0;

    size_t n = strlen(s);
    while (n && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = 0;
}

static bool parse_word(const char *p, char key, float *out)
{
    const char *k = strchr(p, key);
    if (!k) return false;
    *out = strtof(k + 1, NULL);
    return true;
}

static bool send_xyz_blocking(float x, float y, float z, float pitch_deg, TickType_t timeout)
{
    TickType_t t0 = xTaskGetTickCount();

    while (!s_stop && (xTaskGetTickCount() - t0) < timeout) {
        if (robot_cmd_move_xyz(x, y, z, pitch_deg)) {
            vTaskDelay(pdMS_TO_TICKS(60));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return false;
}

bool gcode_push_line(const char *line_in)
{
    char line[200];
    strncpy(line, line_in, sizeof(line) - 1);
    line[sizeof(line) - 1] = 0;
    strip_comment(line);
    if (line[0] == 0) return true;

    float g = -1, f = 0, x = 0, y = 0, z = 0;
    float p = 0.0f;

    bool hasG = parse_word(line, 'G', &g);
    bool hasF = parse_word(line, 'F', &f);
    bool hasX = parse_word(line, 'X', &x);
    bool hasY = parse_word(line, 'Y', &y);
    bool hasZ = parse_word(line, 'Z', &z);
    bool hasP = parse_word(line, 'P', &p);

    float pitch_deg = hasP ? p : 0.0f;

    if (hasG) {
        int gi = (int)lroundf(g);

        if (gi == 90) { st.absolute = true; return true; }
        if (gi == 91) { st.absolute = false; return true; }
        if (gi == 20) { st.units_mm = false; return true; }
        if (gi == 21) { st.units_mm = true; return true; }

        if (gi == 0 || gi == 1) {
            if (hasF) {
                float v_mm_min = to_mm(f);
                st.feed_mm_s = v_mm_min / 60.0f;
                if (st.feed_mm_s < MIN_V_MM_S) st.feed_mm_s = MIN_V_MM_S;
            }

            float tx = st.x, ty = st.y, tz = st.z;
            if (hasX) tx = st.absolute ? to_mm(x) : (st.x + to_mm(x));
            if (hasY) ty = st.absolute ? to_mm(y) : (st.y + to_mm(y));
            if (hasZ) tz = st.absolute ? to_mm(z) : (st.z + to_mm(z));

            if (!robot_tcp_reachable(tx, ty, tz, pitch_deg)) {
                ESP_LOGE(TAG, "Unreachable XYZ: x=%.2f y=%.2f z=%.2f pitch=%.2f", tx, ty, tz, pitch_deg);
                s_error = true;
                s_stop = true;
                robot_cmd_queue_flush();
                return false;
            }

            float pitch_deg = hasP ? p : NAN; // NAN => default pitch v robot_io.c

            if (!send_xyz_blocking(tx, ty, tz, pitch_deg, pdMS_TO_TICKS(8000))) {
                ESP_LOGE(TAG, "Queue timeout sending XYZ");
                s_error = true;
                s_stop = true;
                return false;
            }

            st.x = tx;
            st.y = ty;
            st.z = tz;
            return true;
        }
    }

    if (hasF && !hasG) {
        float v_mm_min = to_mm(f);
        st.feed_mm_s = v_mm_min / 60.0f;
        if (st.feed_mm_s < MIN_V_MM_S) st.feed_mm_s = MIN_V_MM_S;
    }

    return true;
}

bool gcode_run_file(const char *filename)
{
    gcode_reset();
    s_stop = false;
    s_error = false;

    if (!filename || filename[0] == 0) {
        ESP_LOGE(TAG, "Empty filename");
        return false;
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