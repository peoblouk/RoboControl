// ===============================
// robot_io.c
// ===============================

#include "robot_io.h"
#include "gcode.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "robot_io";
static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_oneshot_unit_handle_t s_adc2 = NULL;

#define SERVO_DUTY_MAX ((1U << 14) - 1)   // timer is LEDC_TIMER_14_BIT

static const int s_joint_master_servo[JOINT_COUNT] = {
    JOINT0_SERVO, JOINT1_SERVO, JOINT2_SERVO, JOINT3_SERVO, JOINT4_SERVO, JOINT5_SERVO
};

static const int s_joint_sensor_idx[JOINT_COUNT] = { 0, 1, 2, 3, 4, 5 };

servo_t servos[SERVO_COUNT] = {
    { .gpio_num = SERVO0_GPIO, .channel = SERVO0_CH }, // J0
    { .gpio_num = SERVO1_GPIO, .channel = SERVO1_CH }, // J1_A
    { .gpio_num = SERVO2_GPIO, .channel = SERVO2_CH }, // J1_B (follower)
    { .gpio_num = SERVO3_GPIO, .channel = SERVO3_CH }, // J2
    { .gpio_num = SERVO4_GPIO, .channel = SERVO4_CH }, // J3
    { .gpio_num = SERVO5_GPIO, .channel = SERVO5_CH }, // J4
    { .gpio_num = SERVO6_GPIO, .channel = SERVO6_CH }, // J5 (gripper)
};

sensor_t sensors[SENSOR_COUNT] = {
    { .unit = S0_ADC_UNIT, .channel = S0_ADC_CH },
    { .unit = S1_ADC_UNIT, .channel = S1_ADC_CH },
    { .unit = S2_ADC_UNIT, .channel = S2_ADC_CH },
    { .unit = S3_ADC_UNIT, .channel = S3_ADC_CH },
    { .unit = S4_ADC_UNIT, .channel = S4_ADC_CH },
    { .unit = S5_ADC_UNIT, .channel = S5_ADC_CH },
};

const joint_limits_t g_joint_limits[SERVO_COUNT] = {
    { .min_deg = J0_MIN, .max_deg = J0_MAX, .max_deg_s = J0_V }, // servo 0 (J0)
    { .min_deg = J1_MIN, .max_deg = J1_MAX, .max_deg_s = J1_V }, // servo 1 (J1 master)
    { .min_deg = J1_MIN, .max_deg = J1_MAX, .max_deg_s = J1_V }, // servo 2 (J1 follower)
    { .min_deg = J2_MIN, .max_deg = J2_MAX, .max_deg_s = J2_V }, // servo 3 (J2)
    { .min_deg = J3_MIN, .max_deg = J3_MAX, .max_deg_s = J3_V }, // servo 4 (J3)
    { .min_deg = J4_MIN, .max_deg = J4_MAX, .max_deg_s = J4_V }, // servo 5 (J4)
    { .min_deg = J5_MIN, .max_deg = J5_MAX, .max_deg_s = J5_V }, // servo 6 (gripper)
};

typedef struct {
    char filename[64];
} gcode_task_params_t;

static QueueHandle_t s_robot_queue = NULL;
static traj_seg_t s_seg_buf[SEG_BUF_LEN];
static int s_seg_w = 0, s_seg_r = 0;
static traj_seg_t s_cur = {0};
static float s_last_q[SERVO_COUNT] = {0};
static volatile bool s_armed = true;
static volatile bool s_referenced = false;
static volatile bool s_tcp_est_valid = false;
static robot_pose_t s_tcp_est_base = {
    .x = ROBOT_HOME_X_BASE_DEFAULT,
    .y = ROBOT_HOME_Y_BASE_DEFAULT,
    .z = ROBOT_HOME_Z_BASE_DEFAULT,
    .pitch_deg = ROBOT_HOME_PITCH_DEG_DEFAULT,
};
static const float s_home_q_init[SERVO_COUNT] = HOME_Q_INIT;

static float s_work_offset_xyz[3] = {
    ROBOT_WORK_OFFSET_X_DEFAULT,
    ROBOT_WORK_OFFSET_Y_DEFAULT,
    ROBOT_WORK_OFFSET_Z_DEFAULT,
};

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int servo_master(int servo_id) {
    return (servo_id == J1_B_SERVO) ? J1_A_SERVO : servo_id;
}

static inline int servo_follower(int servo_id) {
    servo_id = servo_master(servo_id);
    return (servo_id == J1_A_SERVO) ? J1_B_SERVO : -1;
}

static inline void work_to_base_xyz(float xw, float yw, float zw, float *xb, float *yb, float *zb)
{
    if (xb) *xb = xw + s_work_offset_xyz[0];
    if (yb) *yb = yw + s_work_offset_xyz[1];
    if (zb) *zb = zw + s_work_offset_xyz[2];
}

static inline void base_to_work_xyz(float xb, float yb, float zb, float *xw, float *yw, float *zw)
{
    if (xw) *xw = xb - s_work_offset_xyz[0];
    if (yw) *yw = yb - s_work_offset_xyz[1];
    if (zw) *zw = zb - s_work_offset_xyz[2];
}

static inline void robot_tcp_estimate_invalidate(void)
{
    s_tcp_est_valid = false;
}

static inline void robot_tcp_estimate_set_base(float x, float y, float z, float pitch_deg)
{
    s_tcp_est_base.x = x;
    s_tcp_est_base.y = y;
    s_tcp_est_base.z = z;
    s_tcp_est_base.pitch_deg = pitch_deg;
    s_tcp_est_valid = true;
}

float robot_get_est_angle(int id)
{
    if (id < 0 || id >= SERVO_COUNT) return 0.0f;
    return s_last_q[id];
}

void robot_set_work_offset(float x, float y, float z)
{
    s_work_offset_xyz[0] = x;
    s_work_offset_xyz[1] = y;
    s_work_offset_xyz[2] = z;
    ESP_LOGI(TAG, "Work offset set: x=%.1f y=%.1f z=%.1f", x, y, z);
}

void robot_get_work_offset(float *x, float *y, float *z)
{
    if (x) *x = s_work_offset_xyz[0];
    if (y) *y = s_work_offset_xyz[1];
    if (z) *z = s_work_offset_xyz[2];
}

bool robot_is_referenced(void)
{
    return s_referenced;
}

bool robot_has_tcp_estimate(void)
{
    return s_tcp_est_valid;
}

void robot_clear_reference(void)
{
    s_referenced = false;
    robot_tcp_estimate_invalidate();
}

bool robot_get_tcp_estimate_base(robot_pose_t *pose)
{
    if (!pose || !s_tcp_est_valid) return false;
    *pose = s_tcp_est_base;
    return true;
}

bool robot_get_tcp_estimate_work(robot_pose_t *pose)
{
    if (!pose || !s_tcp_est_valid) return false;
    *pose = s_tcp_est_base;
    base_to_work_xyz(s_tcp_est_base.x, s_tcp_est_base.y, s_tcp_est_base.z, &pose->x, &pose->y, &pose->z);
    return true;
}

// ===============================
// SERVO PWM RANGES
// ===============================
static servo_pwm_range_t s_servo_pwm[SERVO_COUNT] = SERVO_PWM_RANGES_INIT;
static portMUX_TYPE s_pwm_mux = portMUX_INITIALIZER_UNLOCKED;

bool servo_pwm_set_range_us(int servo_id, int min_us, int max_us)
{
    if (servo_id < 0 || servo_id >= SERVO_COUNT) return false;
    if (min_us < 0 || max_us < 0) return false;
    if (min_us >= max_us) return false;

    const int period_us = (int)lroundf(1000000.0f / (float)SERVO_PWM_FREQ);
    if (min_us > period_us || max_us > period_us) return false;

    portENTER_CRITICAL(&s_pwm_mux);
    s_servo_pwm[servo_id].min_us = (uint16_t)min_us;
    s_servo_pwm[servo_id].max_us = (uint16_t)max_us;
    portEXIT_CRITICAL(&s_pwm_mux);
    return true;
}

void servo_pwm_get_range_us(int servo_id, int *min_us, int *max_us)
{
    if (min_us) *min_us = 0;
    if (max_us) *max_us = 0;
    if (servo_id < 0 || servo_id >= SERVO_COUNT) return;

    portENTER_CRITICAL(&s_pwm_mux);
    uint16_t mn = s_servo_pwm[servo_id].min_us;
    uint16_t mx = s_servo_pwm[servo_id].max_us;
    portEXIT_CRITICAL(&s_pwm_mux);

    if (min_us) *min_us = (int)mn;
    if (max_us) *max_us = (int)mx;
}

static inline uint32_t angle_to_duty(int servo_id, float angle_deg)
{
    const float period_us = 1000000.0f / (float)SERVO_PWM_FREQ;

    uint16_t mn, mx;
    portENTER_CRITICAL(&s_pwm_mux);
    mn = s_servo_pwm[servo_id].min_us;
    mx = s_servo_pwm[servo_id].max_us;
    portEXIT_CRITICAL(&s_pwm_mux);

    const float duty_min_f = ((float)mn / period_us) * (float)SERVO_DUTY_MAX;
    const float duty_max_f = ((float)mx / period_us) * (float)SERVO_DUTY_MAX;

    float t = angle_deg / 180.0f;
    if (t < 0) t = 0;
    if (t > 1) t = 1;

    float duty_f = duty_min_f + (duty_max_f - duty_min_f) * t;
    if (duty_f < 0) duty_f = 0;
    if (duty_f > (float)SERVO_DUTY_MAX) duty_f = (float)SERVO_DUTY_MAX;

    return (uint32_t)(duty_f + 0.5f);
}

// ===============================
// SERVO/KINEMATICS MAPPING
// ===============================
static float OFF[SERVO_COUNT] = SERVO_OFF_INIT;
static float DIR[SERVO_COUNT] = SERVO_DIR_INIT;

static inline float map_servo(int sid, float joint_deg_math)
{
    return OFF[sid] + DIR[sid] * joint_deg_math;
}

static inline float j1_b_from_j1_a(float a_deg)
{
    return a_deg + J1_B_TRIM_DEG;
}

static inline void j1_a_allowed_range(float *lo, float *hi)
{
    float a_lo = g_joint_limits[J1_A_SERVO].min_deg;
    float a_hi = g_joint_limits[J1_A_SERVO].max_deg;

    const float b_lo = g_joint_limits[J1_B_SERVO].min_deg;
    const float b_hi = g_joint_limits[J1_B_SERVO].max_deg;

    const float a_from_b_lo = b_lo - J1_B_TRIM_DEG;
    const float a_from_b_hi = b_hi - J1_B_TRIM_DEG;

    if (a_from_b_lo > a_lo) a_lo = a_from_b_lo;
    if (a_from_b_hi < a_hi) a_hi = a_from_b_hi;

    *lo = a_lo;
    *hi = a_hi;
}

// ===============================
// JOINT LIMITS
// ===============================
static void joint_limits_get(int joint_id, float *lo, float *hi, float *vmax)
{
    int s = s_joint_master_servo[joint_id];
    float l = g_joint_limits[s].min_deg;
    float h = g_joint_limits[s].max_deg;
    float v = g_joint_limits[s].max_deg_s;

    if (s == J1_A_SERVO) {
        j1_a_allowed_range(&l, &h);

        float v2 = g_joint_limits[J1_B_SERVO].max_deg_s;
        if (v2 < v) v = v2;
    }

    *lo = l;
    *hi = h;
    *vmax = v;
}

// ===============================
// INITIALIZATION OF SERVOS
// ===============================
void servos_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = SERVO_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t channel = {
            .gpio_num   = servos[i].gpio_num,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = servos[i].channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }

    ESP_LOGI(TAG, "Servos initialized");
}

// ===============================
// INITIALIZATION OF SENSORS
// ===============================
void sensors_init(void)
{
    bool need_adc1 = false;
    bool need_adc2 = false;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensors[i].unit == ADC_UNIT_1) need_adc1 = true;
        else if (sensors[i].unit == ADC_UNIT_2) need_adc2 = true;
    }

    if (need_adc1) {
        adc_oneshot_unit_init_cfg_t cfg1 = {
            .unit_id  = ADC_UNIT_1,
            .clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &s_adc1));
    }

    if (need_adc2) {
        adc_oneshot_unit_init_cfg_t cfg2 = {
            .unit_id  = ADC_UNIT_2,
            .clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg2, &s_adc2));
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < SENSOR_COUNT; i++) {
        adc_oneshot_unit_handle_t unit =
            (sensors[i].unit == ADC_UNIT_1) ? s_adc1 : s_adc2;

        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            unit,
            sensors[i].channel,
            &chan_cfg
        ));
    }

    ESP_LOGI(TAG, "Sensors initialized (ADC1:%s, ADC2:%s)",
             need_adc1 ? "yes" : "no",
             need_adc2 ? "yes" : "no");
}

// ===============================
// SENSOR READ RAW DATA (0–4095)
// ===============================
int sensor_read_raw(int id)
{
    if (id < 0 || id >= SENSOR_COUNT) return -1;

    adc_oneshot_unit_handle_t unit =
        (sensors[id].unit == ADC_UNIT_1) ? s_adc1 : s_adc2;

    if (!unit) {
        ESP_LOGW(TAG, "ADC unit not init for sensor %d", id);
        return -1;
    }

    int raw = 0;
    esp_err_t err = adc_oneshot_read(unit, sensors[id].channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: id=%d unit=%d ch=%d err=%s",
                 id, sensors[id].unit, sensors[id].channel,
                 esp_err_to_name(err));
        return -1;
    }
    return raw;
}

// ===============================
// SENSOR READ ANGLE (0-180°)
// ===============================
float sensor_read_angle(int id)
{
    int raw = sensor_read_raw(id);
    if (raw < 0) return -1;
    return (raw / 4095.0f) * 180.0f;
}

// ===============================
// SET SERVO ANGLE (master + follower)
// ===============================
void servo_set_angle(int servo_id, float angle)
{
    if (servo_id < 0 || servo_id >= SERVO_COUNT) {
        ESP_LOGW(TAG, "Invalid servo ID: %d", servo_id);
        return;
    }

    int master = servo_master(servo_id);
    int other  = servo_follower(master);

    float lo = g_joint_limits[master].min_deg;
    float hi = g_joint_limits[master].max_deg;

    if (master == J1_A_SERVO) {
        j1_a_allowed_range(&lo, &hi);
    }

    angle = clampf(angle, lo, hi);
    robot_tcp_estimate_invalidate();

    if (!s_armed) {
        ledc_stop(LEDC_LOW_SPEED_MODE, servos[master].channel, 0);
        s_last_q[master] = angle;

        if (other >= 0) {
            float other_angle = j1_b_from_j1_a(angle);

            float lo2 = g_joint_limits[other].min_deg;
            float hi2 = g_joint_limits[other].max_deg;
            other_angle = clampf(other_angle, lo2, hi2);

            ledc_stop(LEDC_LOW_SPEED_MODE, servos[other].channel, 0);
            s_last_q[other] = other_angle;
        }
        return;
    }

    uint32_t duty = angle_to_duty(master, angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[master].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[master].channel);
    s_last_q[master] = angle;

    if (other >= 0) {
        float other_angle = j1_b_from_j1_a(angle);

        float lo2 = g_joint_limits[other].min_deg;
        float hi2 = g_joint_limits[other].max_deg;
        other_angle = clampf(other_angle, lo2, hi2);

        uint32_t duty2 = angle_to_duty(other, other_angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[other].channel, duty2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[other].channel);

        s_last_q[other] = other_angle;
    }
}

static void seed_last_q_from_home(void)
{
    for (int i = 0; i < SERVO_COUNT; i++) {
        s_last_q[i] = s_home_q_init[i];
    }

    robot_validate_and_prepare_q(s_last_q, true);
}

// ===============================
// SET JOINT ANGLE
// ===============================
void joint_set_angle(int joint_id, float angle)
{
    if (joint_id < 0 || joint_id >= JOINT_COUNT) {
        ESP_LOGW(TAG, "Invalid joint ID: %d", joint_id);
        return;
    }

    int s = s_joint_master_servo[joint_id];
    servo_set_angle(s, angle);
}

// ===============================
// VALIDATE / CLAMP q[SERVO_COUNT]
// ===============================
bool robot_validate_and_prepare_q(float q[SERVO_COUNT], bool clamp)
{
    bool ok = true;

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i == J1_B_SERVO) continue;

        float lo = g_joint_limits[i].min_deg;
        float hi = g_joint_limits[i].max_deg;

        if (i == J1_A_SERVO) {
            j1_a_allowed_range(&lo, &hi);
        }

        if (q[i] < lo || q[i] > hi) ok = false;
        if (clamp) q[i] = clampf(q[i], lo, hi);
    }

    float b = j1_b_from_j1_a(q[J1_A_SERVO]);
    float blo = g_joint_limits[J1_B_SERVO].min_deg;
    float bhi = g_joint_limits[J1_B_SERVO].max_deg;

    if (b < blo || b > bhi) ok = false;
    if (clamp) b = clampf(b, blo, bhi);

    q[J1_B_SERVO] = b;
    return ok;
}

bool robot_tcp_reachable(float x, float y, float z, float pitch_deg)
{
    float r = sqrtf(x*x + y*y);
    float phi = DEG2RAD(pitch_deg);

    float r_w  = r - L_TOOL * cosf(phi);
    float z_w  = z - L_TOOL * sinf(phi);
    float z_sh = z_w - L0;

    float d = sqrtf(r_w*r_w + z_sh*z_sh);

    if (d > (L1 + L2)) return false;
    if (d < fabsf(L1 - L2)) return false;
    return true;
}

bool robot_tcp_reachable_work(float x, float y, float z, float pitch_deg)
{
    float xb, yb, zb;
    work_to_base_xyz(x, y, z, &xb, &yb, &zb);
    return robot_tcp_reachable(xb, yb, zb, pitch_deg);
}

float robot_min_time_for_move(const float q0[SERVO_COUNT], const float q1[SERVO_COUNT])
{
    float Tmin = 0.0f;

    for (int j = 0; j < JOINT_COUNT; j++) {
        int s = s_joint_master_servo[j];
        float lo, hi, vmax;
        (void)lo; (void)hi;
        joint_limits_get(j, &lo, &hi, &vmax);

        float diff = fabsf(q1[s] - q0[s]);
        if (vmax > 1e-6f) {
            float t = diff / vmax;
            if (t > Tmin) Tmin = t;
        }
    }

    return Tmin;
}

// ===============================
// INVERSE KINEMATICS
// ===============================
static bool inverse_kinematics_tcp(float x, float y, float z,
                                   float tool_pitch_deg,
                                   float q_target[SERVO_COUNT])
{
    float r = sqrtf(x*x + y*y);

    // base angle, keep last when x=y ~ 0
    float q0;
    if (r < 1e-6f) {
        float d0 = DIR[0];
        q0 = (fabsf(d0) < 1e-6f) ? 0.0f : DEG2RAD((s_last_q[0] - OFF[0]) / d0);
    } else {
        q0 = atan2f(y, x);
    }

    float phi = DEG2RAD(tool_pitch_deg);

    // wrist (pitch pivot) from TCP
    float r_w  = r - (float)L_TOOL * cosf(phi);
    float z_w  = z - (float)L_TOOL * sinf(phi);
    float z_sh = z_w - L0;

    float d2 = r_w*r_w + z_sh*z_sh;
    float d  = sqrtf(d2);

    if (d > (L1 + L2)) return false;
    if (d < fabsf(L1 - L2)) return false;

    float cos_q2 = (d2 - L1*L1 - L2*L2) / (2.0f*L1*L2);
    if (cos_q2 >  1.0f) cos_q2 =  1.0f;
    if (cos_q2 < -1.0f) cos_q2 = -1.0f;

    // elbow angle in [0..pi]
    float q2 = acosf(cos_q2);

    float phi2 = atan2f(z_sh, r_w);
    float psi  = atan2f(L2*sinf(q2), L1 + L2*cosf(q2));

    // two elbow configurations: q1 = phi2 ± psi
    float q1_opts[2] = { (phi2 - psi), (phi2 + psi) };

    float best_q[SERVO_COUNT];
    float best_cost = 1e30f;
    bool  best_ok = false;

    for (int e = 0; e < 2; e++) {
        float q1 = q1_opts[e];

        // raw q3
        float q3_raw = phi - q1 - q2;

        // try q3 wrapped by ±2π to fit limits / reduce motion
        for (int k = -1; k <= 1; k++) {
            float q3 = q3_raw + (float)k * 2.0f * (float)M_PI;

            float j0 = RAD2DEG(q0);
            float j1 = RAD2DEG(q1);
            float j2 = RAD2DEG(q2);
            float j3 = RAD2DEG(q3);

            float cand[SERVO_COUNT];
            for (int i = 0; i < SERVO_COUNT; i++) cand[i] = s_last_q[i];

            cand[0]          = map_servo(0, j0);
            cand[J1_A_SERVO] = map_servo(J1_A_SERVO, j1);
            cand[3]          = map_servo(3, j2);
            cand[4]          = map_servo(4, j3);

            // reject if any joint out of range (no clamping here)
            if (!robot_validate_and_prepare_q(cand, false)) continue;

            // cost = minimal change from current pose (servo space)
            float cost = 0.0f;
            cost += fabsf(cand[0]          - s_last_q[0]);
            cost += fabsf(cand[J1_A_SERVO] - s_last_q[J1_A_SERVO]);
            cost += fabsf(cand[3]          - s_last_q[3]);
            cost += fabsf(cand[4]          - s_last_q[4]);

            if (cost < best_cost) {
                best_cost = cost;
                for (int i = 0; i < SERVO_COUNT; i++) best_q[i] = cand[i];
                best_ok = true;
            }
        }
    }

    if (!best_ok) return false;

    for (int i = 0; i < SERVO_COUNT; i++) q_target[i] = best_q[i];
    return true;
}

// void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT])
// {
//     float q0 = atan2f(y, x);

//     float R    = sqrtf(x*x + y*y);
//     float z_sh = z - L0;
//     float d    = sqrtf(R*R + z_sh*z_sh);

//     float cos_q2 = (d*d - L1*L1 - L2*L2) / (2.0f*L1*L2);
//     if (cos_q2 > 1.0f)  cos_q2 = 1.0f;
//     if (cos_q2 < -1.0f) cos_q2 = -1.0f;

//     float q2 = acosf(cos_q2);

//     float phi = atan2f(z_sh, R);
//     float psi = atan2f(L2*sinf(q2), L1 + L2*cosf(q2));
//     float q1  = phi - psi;

//     float j0 = RAD2DEG(q0);
//     float j1 = RAD2DEG(q1);
//     float j2 = RAD2DEG(q2);

//     q_target[0]          = map_servo(0, j0);
//     q_target[J1_A_SERVO] = map_servo(J1_A_SERVO, j1);
//     q_target[3]          = map_servo(3, j2);
// }

// ===============================
// SEGMENT BUFFER
// ===============================
static bool seg_full(void)  { return ((s_seg_w + 1) % SEG_BUF_LEN) == s_seg_r; }
static bool seg_empty(void) { return s_seg_w == s_seg_r; }

static void seg_flush(void) {
    s_seg_w = s_seg_r = 0;
    s_cur.active = false;
}

static bool seg_push(const traj_seg_t *s) {
    if (seg_full()) return false;
    s_seg_buf[s_seg_w] = *s;
    s_seg_w = (s_seg_w + 1) % SEG_BUF_LEN;
    return true;
}

static bool seg_pop(traj_seg_t *s) {
    if (seg_empty()) return false;
    *s = s_seg_buf[s_seg_r];
    s_seg_r = (s_seg_r + 1) % SEG_BUF_LEN;
    return true;
}

static float smoothstep(float s) {
    if (s < 0) s = 0;
    if (s > 1) s = 1;
    return 3.0f*s*s - 2.0f*s*s*s;
}

static void apply_joints(const float q[SERVO_COUNT])
{
    for (int j = 0; j < JOINT_COUNT; j++) {
        int s = s_joint_master_servo[j];
        joint_set_angle(j, q[s]);
    }
}

// ===============================
// ROBOT CONTROL TASK
// ===============================
static void robot_control_task(void *arg)
{
    (void)arg;

    for (int i = 0; i < SERVO_COUNT; i++) s_last_q[i] = 90.0f;
    
    seed_last_q_from_home();
    robot_clear_reference();

    robot_cmd_t cmd;
    static bool disarmed_latched = false;

    for (;;) {

        if (!s_armed) {
            if (!disarmed_latched) {
                seg_flush();
                for (int i = 0; i < SERVO_COUNT; i++) {
                    ledc_stop(LEDC_LOW_SPEED_MODE, servos[i].channel, 0);
                }
                disarmed_latched = true;
            }

            while (xQueueReceive(s_robot_queue, &cmd, 0) == pdTRUE) {
            }

            vTaskDelay(pdMS_TO_TICKS(EXEC_DT_MS));
            continue;
        }

        disarmed_latched = false;

        if (xQueueReceive(s_robot_queue, &cmd, pdMS_TO_TICKS(EXEC_DT_MS)) == pdTRUE) {

            if (cmd.type == ROBOT_CMD_QUEUE_FLUSH) {
                seg_flush();
                continue;
            }

            if (seg_full()) {
                ESP_LOGW(TAG, "Planner full, dropping command");
                continue;
            }

            traj_seg_t seg = {0};

            for (int i = 0; i < SERVO_COUNT; i++) seg.q0[i] = s_last_q[i];
            seg.tcp_target_valid = false;
            seg.mark_referenced_on_finish = false;

            if (cmd.type == ROBOT_CMD_MOVE_JOINTS) {
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = cmd.q_target[i];
                robot_validate_and_prepare_q(seg.q1, true);
                seg.T = 1.0f;
                seg.tcp_target_valid = cmd.tcp_target_valid;
                seg.mark_referenced_on_finish = cmd.mark_referenced_on_finish;
                seg.tcp_target_base.x = cmd.x;
                seg.tcp_target_base.y = cmd.y;
                seg.tcp_target_base.z = cmd.z;
                seg.tcp_target_base.pitch_deg = cmd.pitch_deg;
            }
            else if (cmd.type == ROBOT_CMD_MOVE_XYZ) {
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = seg.q0[i];

                float pitch = cmd.pitch_deg;
                if (!isfinite(pitch)) pitch = ROBOT_DEFAULT_PITCH_DEG;
                if (pitch > 89.0f)  pitch = 89.0f;
                if (pitch < -89.0f) pitch = -89.0f;

                ESP_LOGW(TAG, "IK TCP(base): x=%.1f y=%.1f z=%.1f pitch=%.1f",
                         cmd.x, cmd.y, cmd.z, pitch);

                bool ok = inverse_kinematics_tcp(cmd.x, cmd.y, cmd.z, pitch, seg.q1);
                if (!ok) { ESP_LOGW(TAG, "IK TCP failed"); continue; }

                ESP_LOGW(TAG, "IK servo: s0(J0)=%.1f s1(J1)=%.1f s3(J2)=%.1f s4(J3)=%.1f",
                         seg.q1[0], seg.q1[1], seg.q1[3], seg.q1[4]);

                robot_validate_and_prepare_q(seg.q1, true);
                seg.T = 1.0f;
                seg.tcp_target_valid = true;
                seg.tcp_target_base.x = cmd.x;
                seg.tcp_target_base.y = cmd.y;
                seg.tcp_target_base.z = cmd.z;
                seg.tcp_target_base.pitch_deg = pitch;
            }
            else if (cmd.type == ROBOT_CMD_MOVE_JOINTS_T) {
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = cmd.q_target[i];
                robot_validate_and_prepare_q(seg.q1, true);
                seg.T = cmd.duration_s;
                seg.tcp_target_valid = cmd.tcp_target_valid;
                seg.mark_referenced_on_finish = cmd.mark_referenced_on_finish;
                seg.tcp_target_base.x = cmd.x;
                seg.tcp_target_base.y = cmd.y;
                seg.tcp_target_base.z = cmd.z;
                seg.tcp_target_base.pitch_deg = cmd.pitch_deg;
            }
            else {
                ESP_LOGW(TAG, "Unknown robot command: %d", cmd.type);
                continue;
            }

            float Tmin = robot_min_time_for_move(seg.q0, seg.q1);
            if (seg.T < Tmin) seg.T = Tmin;
            if (seg.T < MIN_SEG_T) seg.T = MIN_SEG_T;

            seg.t = 0;
            seg.active = false;

            seg_push(&seg);
        }

        if (!s_cur.active) {
            if (!seg_pop(&s_cur)) continue;
            s_cur.t = 0;
            s_cur.active = true;
        }

        s_cur.t += EXEC_DT_S;
        float s = (s_cur.T > 1e-6f) ? (s_cur.t / s_cur.T) : 1.0f;
        float a = smoothstep(s);

        float q[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) {
            q[i] = s_cur.q0[i] + (s_cur.q1[i] - s_cur.q0[i]) * a;
        }

        robot_validate_and_prepare_q(q, true);
        apply_joints(q);

        if (s >= 1.0f) {
            for (int i = 0; i < SERVO_COUNT; i++) s_last_q[i] = s_cur.q1[i];

            if (s_cur.tcp_target_valid) {
                robot_tcp_estimate_set_base(s_cur.tcp_target_base.x,
                                            s_cur.tcp_target_base.y,
                                            s_cur.tcp_target_base.z,
                                            s_cur.tcp_target_base.pitch_deg);
            } else {
                robot_tcp_estimate_invalidate();
            }

            if (s_cur.mark_referenced_on_finish) {
                s_referenced = true;
                ESP_LOGI(TAG, "Robot reference established");
            }

            s_cur.active = false;
        }
    }
}

bool robot_is_armed(void)
{
    return s_armed;
}

void robot_disarm(void)
{
    s_armed = false;
    robot_cmd_queue_flush();

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, servos[i].channel, 0);
    }
}

void robot_arm(void)
{
    s_armed = true;
}

// ===============================
// START ROBOT CONTROL
// ===============================
void robot_control_start(void)
{
    s_robot_queue = xQueueCreate(8, sizeof(robot_cmd_t));
    if (s_robot_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create robot queue");
        return;
    }

    BaseType_t res = xTaskCreatePinnedToCore(robot_control_task, "robot_ctrl", 4096, NULL, 6, NULL, CORE_ROBOT);
    if (res != pdPASS) ESP_LOGE(TAG, "Failed to create robot_control_task");
}

bool robot_cmd_move_joints(const float q_target[SERVO_COUNT])
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_JOINTS;
    for (int i = 0; i < SERVO_COUNT; i++) cmd.q_target[i] = q_target[i];

    return xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE;
}

bool robot_cmd_move_joints_home(const float q_target[SERVO_COUNT],
                                float home_x_base,
                                float home_y_base,
                                float home_z_base,
                                float home_pitch_deg)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_JOINTS;
    for (int i = 0; i < SERVO_COUNT; i++) cmd.q_target[i] = q_target[i];
    cmd.tcp_target_valid = true;
    cmd.mark_referenced_on_finish = true;
    cmd.x = home_x_base;
    cmd.y = home_y_base;
    cmd.z = home_z_base;
    cmd.pitch_deg = home_pitch_deg;

    return xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE;
}

bool robot_cmd_move_xyz(float x, float y, float z, float pitch_deg)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_XYZ;
    cmd.x = x; cmd.y = y; cmd.z = z;
    cmd.pitch_deg = pitch_deg;

    return xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE;
}

bool robot_cmd_move_xyz_work(float x, float y, float z, float pitch_deg)
{
    if (!s_referenced) {
        ESP_LOGW(TAG, "Rejecting work-frame move: robot not referenced");
        return false;
    }

    float xb, yb, zb;
    work_to_base_xyz(x, y, z, &xb, &yb, &zb);
    return robot_cmd_move_xyz(xb, yb, zb, pitch_deg);
}

bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_JOINTS_T;
    for (int i = 0; i < SERVO_COUNT; i++) cmd.q_target[i] = q_target[i];
    cmd.duration_s = duration_s;

    return xQueueSend(s_robot_queue, &cmd, timeout) == pdTRUE;
}

void robot_cmd_queue_flush(void)
{
    if (s_robot_queue == NULL) return;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_QUEUE_FLUSH;
    (void)xQueueSend(s_robot_queue, &cmd, 0);
}

// ===============================
// RUN GCODE FILE
// ===============================
static void gcode_executor_task(void *arg)
{
    gcode_task_params_t *params = (gcode_task_params_t *)arg;

    ESP_LOGI(TAG, "Starting G-Code task for file: %s", params->filename);
    bool res = gcode_run_file(params->filename);

    if (res) ESP_LOGI(TAG, "G-Code finished successfully");
    else     ESP_LOGE(TAG, "G-Code failed");

    free(params);
    vTaskDelete(NULL);
}

bool robot_ik_tcp(float x, float y, float z, float pitch_deg, float q_target[SERVO_COUNT])
{
    for (int i = 0; i < SERVO_COUNT; i++) q_target[i] = robot_get_est_angle(i);

    if (!inverse_kinematics_tcp(x, y, z, pitch_deg, q_target)) return false;

    robot_validate_and_prepare_q(q_target, true);
    return true;
}

void robot_core_run_gcode(const char *filename)
{
    if (!s_armed) return;

    gcode_task_params_t *params = malloc(sizeof(gcode_task_params_t));
    if (!params) {
        ESP_LOGE(TAG, "Failed to allocate memory for G-code task");
        return;
    }

    strncpy(params->filename, filename, sizeof(params->filename) - 1);
    params->filename[sizeof(params->filename) - 1] = 0;

    BaseType_t res = xTaskCreatePinnedToCore(
        gcode_executor_task,
        "gcode_exec",
        4096,
        params,
        4,
        NULL,
        CORE_ROBOT
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create G-code task");
        free(params);
    }
}