// ===============================
// gcode.c
// ===============================

#include "gcode.h"

static volatile bool s_stop = false;
static gcode_state_t st;

static float to_mm(float v) { return st.units_mm ? v : (v * 25.4f); }

void gcode_reset(void)
{
    st.absolute = true;
    st.units_mm = true;
    st.feed_mm_s = 50.0f;
    st.x = st.y = st.z = 0.0f;
    s_stop = false;
}

void gcode_stop(void) { s_stop = true; robot_cmd_queue_flush(); }

static void strip_comment(char *s)
{
    // ';' comment
    char *c = strchr(s, ';');
    if (c) *c = 0;     // trim \r\n
    size_t n = strlen(s);
    while (n && (s[n-1]=='\n' || s[n-1]=='\r' || s[n-1]==' ' || s[n-1]=='\t')) s[--n]=0;
}

static bool parse_word(const char *p, char key, float *out)
{
    const char *k = strchr(p, key);
    if (!k) return false;
    *out = strtof(k+1, NULL);
    return true;
}

bool gcode_push_line(const char *line_in)
{
    if (!st.units_mm && !st.absolute && st.feed_mm_s==0) {}

    char line[200];
    strncpy(line, line_in, sizeof(line)-1);
    line[sizeof(line)-1] = 0;
    strip_comment(line);
    if (line[0] == 0) return true;

    float g=-1, f=0, x=0, y=0, z=0;
    bool hasG = parse_word(line,'G',&g);
    bool hasF = parse_word(line,'F',&f);
    bool hasX = parse_word(line,'X',&x);
    bool hasY = parse_word(line,'Y',&y);
    bool hasZ = parse_word(line,'Z',&z);

    if (hasG) {
        int gi = (int)lroundf(g);

        if (gi == 90) { st.absolute = true; return true; }
        if (gi == 91) { st.absolute = false; return true; }
        if (gi == 20) { st.units_mm = false; return true; }
        if (gi == 21) { st.units_mm = true; return true; }

        if (gi == 0 || gi == 1) {
            // update feed if provided (F is in units/min)
            if (hasF) {
                float v_mm_min = to_mm(f);
                st.feed_mm_s = v_mm_min / 60.0f;
                if (st.feed_mm_s < MIN_V_MM_S) st.feed_mm_s = MIN_V_MM_S;
            }

            float tx = st.x, ty = st.y, tz = st.z;
            if (hasX) tx = st.absolute ? to_mm(x) : (st.x + to_mm(x));
            if (hasY) ty = st.absolute ? to_mm(y) : (st.y + to_mm(y));
            if (hasZ) tz = st.absolute ? to_mm(z) : (st.z + to_mm(z));

            float dx = tx - st.x, dy = ty - st.y, dz = tz - st.z;
            float dist = sqrtf(dx*dx + dy*dy + dz*dz);

            float v = (gi==0) ? RAPID_MM_S : st.feed_mm_s;
            if (v < MIN_V_MM_S) v = MIN_V_MM_S;

            float T = (dist > 1e-6f) ? (dist / v) : 0.05f;
            if (T < 0.05f) T = 0.05f;

            float q_target[SERVO_COUNT];
            inverse_kinematics(tx, ty, tz, q_target);

            // block up to 2s to avoid losing lines when queue is full
            if (!robot_cmd_move_joints_t(q_target, T, pdMS_TO_TICKS(2000))) return false;

            st.x = tx; st.y = ty; st.z = tz;
            return true;
        }
    }

    // update feed if provided (F is in units/min)
    if (hasF && !hasG) {
        float v_mm_min = to_mm(f);
        st.feed_mm_s = v_mm_min / 60.0f;
        if (st.feed_mm_s < MIN_V_MM_S) st.feed_mm_s = MIN_V_MM_S;
    }

    return true;
}

bool gcode_run_file(const char *filename)
{
    s_stop = false;

    char path[256];
    snprintf(path, sizeof(path), "%s/%s", FILE_STORAGE_PATH, filename);

    FILE *fp = fopen(path, "r");
    if (!fp) return false;

    char line[200];
    while (!s_stop && fgets(line, sizeof(line), fp)) {
        if (!gcode_push_line(line)) {
            // queue full or error
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    fclose(fp);
    return true;
}