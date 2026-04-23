// ===============================
// robot_io.c
// ===============================

#include "robot_io.h"
#include "gcode.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "robot_io";
#if SENSOR_COUNT > 0
static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_oneshot_unit_handle_t s_adc2 = NULL;
#endif

#define SERVO_DUTY_MAX ((1U << 14) - 1)   // timer is LEDC_TIMER_14_BIT

typedef struct {
    char filename[64];
} gcode_task_params_t;

static const int s_joint_master_servo[JOINT_COUNT] = {
    JOINT0_SERVO, JOINT1_SERVO, JOINT2_SERVO, JOINT3_SERVO, JOINT4_SERVO, JOINT5_SERVO
};

servo_t servos[SERVO_COUNT] = {
    { .gpio_num = SERVO0_GPIO, .channel = SERVO0_CH }, // J0
    { .gpio_num = SERVO1_GPIO, .channel = SERVO1_CH }, // J1_A
    { .gpio_num = SERVO2_GPIO, .channel = SERVO2_CH }, // J1_B (follower)
    { .gpio_num = SERVO3_GPIO, .channel = SERVO3_CH }, // J2
    { .gpio_num = SERVO4_GPIO, .channel = SERVO4_CH }, // J3
    { .gpio_num = SERVO5_GPIO, .channel = SERVO5_CH }, // J4
    { .gpio_num = SERVO6_GPIO, .channel = SERVO6_CH }, // J5 (gripper)
};

#if SENSOR_COUNT > 0
sensor_t sensors[SENSOR_COUNT] = {
    { .unit = S0_ADC_UNIT, .channel = S0_ADC_CH },
    { .unit = S1_ADC_UNIT, .channel = S1_ADC_CH },
    { .unit = S2_ADC_UNIT, .channel = S2_ADC_CH },
    { .unit = S3_ADC_UNIT, .channel = S3_ADC_CH },
    { .unit = S4_ADC_UNIT, .channel = S4_ADC_CH },
    { .unit = S5_ADC_UNIT, .channel = S5_ADC_CH },
};
#endif

const joint_limits_t g_joint_limits[SERVO_COUNT] = {
    { .min_deg = J0_MIN, .max_deg = J0_MAX, .max_deg_s = J0_V }, // servo 0 (J0)
    { .min_deg = J1_MIN, .max_deg = J1_MAX, .max_deg_s = J1_V }, // servo 1 (J1 master)
    { .min_deg = J1_MIN, .max_deg = J1_MAX, .max_deg_s = J1_V }, // servo 2 (J1 follower)
    { .min_deg = J2_MIN, .max_deg = J2_MAX, .max_deg_s = J2_V }, // servo 3 (J2)
    { .min_deg = J3_MIN, .max_deg = J3_MAX, .max_deg_s = J3_V }, // servo 4 (J3)
    { .min_deg = J4_MIN, .max_deg = J4_MAX, .max_deg_s = J4_V }, // servo 5 (J4)
    { .min_deg = J5_MIN, .max_deg = J5_MAX, .max_deg_s = J5_V }, // servo 6 (gripper)
};

static float OFF[SERVO_COUNT] = SERVO_OFF_INIT;
static float DIR[SERVO_COUNT] = SERVO_DIR_INIT;
static servo_pwm_range_t s_servo_pwm[SERVO_COUNT] = SERVO_PWM_RANGES_INIT;
static portMUX_TYPE s_pwm_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t s_robot_queue = NULL;
static traj_seg_t s_seg_buf[SEG_BUF_LEN];
static int s_seg_w = 0, s_seg_r = 0;
static traj_seg_t s_current_segment = {0};
static float s_last_q[SERVO_COUNT] = {0};
static volatile bool s_armed = false;
static volatile bool s_operating = false;
static volatile bool s_referenced = false;
static volatile bool s_tcp_estimate_is_valid = false;
static volatile bool s_sync_ready = false;
static volatile bool s_gcode_running = false;
static volatile bool s_program_stop_requested = false;
static volatile robot_error_t s_last_error = ROBOT_ERROR_NONE;
static robot_pose_t s_tcp_est_base = {
    .x = ROBOT_HOME_X_BASE_DEFAULT,
    .y = ROBOT_HOME_Y_BASE_DEFAULT,
    .z = ROBOT_HOME_Z_BASE_DEFAULT,
    .pitch_deg = ROBOT_HOME_PITCH_DEG_DEFAULT,
};
static const float s_home_q_init[SERVO_COUNT] = HOME_Q_INIT;
static const float s_home_pre_q_init[SERVO_COUNT] = HOME_PRE_Q_INIT;

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

static inline float clamp_pitch_deg(float pitch_deg)
{
    if (!isfinite(pitch_deg)) pitch_deg = ROBOT_DEFAULT_PITCH_DEG;
    return clampf(pitch_deg, -89.0f, 89.0f);
}

static inline float tcp_ik_pitch_rad(float pitch_deg, float sign)
{
    return DEG2RAD(sign * clamp_pitch_deg(pitch_deg));
}

static inline int servo_master(int servo_id) {
    return (servo_id == SERVO_J1_B) ? SERVO_J1_A : servo_id;
}

static inline int servo_follower(int servo_id) {
    servo_id = servo_master(servo_id);
    return (servo_id == SERVO_J1_A) ? SERVO_J1_B : -1;
}

static inline float planar_radius_from_base(float x, float y)
{
    return sqrtf(x*x + y*y);
}

static inline float planar_radius_from_j1(float x, float y)
{
    return planar_radius_from_base(x, y) - J1_X_OFFSET;
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

// ===============================
// ROBOT STATE / ESTIMATION
// ===============================
static void robot_set_error_internal(robot_error_t err)
{
    portENTER_CRITICAL(&s_state_mux);
    s_last_error = err;
    portEXIT_CRITICAL(&s_state_mux);
}

void robot_clear_error(void)
{
    robot_set_error_internal(ROBOT_ERROR_NONE);
}

robot_error_t robot_get_last_error(void)
{
    portENTER_CRITICAL(&s_state_mux);
    robot_error_t err = s_last_error;
    portEXIT_CRITICAL(&s_state_mux);
    return err;
}

void robot_set_sync_ready(bool ready)
{
    portENTER_CRITICAL(&s_state_mux);
    s_sync_ready = ready;
    portEXIT_CRITICAL(&s_state_mux);
}

bool robot_is_sync_ready(void)
{
    portENTER_CRITICAL(&s_state_mux);
    bool ready = s_sync_ready;
    portEXIT_CRITICAL(&s_state_mux);
    return ready;
}

bool robot_is_program_running(void)
{
    portENTER_CRITICAL(&s_state_mux);
    bool running = s_gcode_running;
    portEXIT_CRITICAL(&s_state_mux);
    return running;
}

bool robot_is_program_stop_requested(void)
{
    portENTER_CRITICAL(&s_state_mux);
    bool requested = s_program_stop_requested;
    portEXIT_CRITICAL(&s_state_mux);
    return requested;
}

void robot_set_program_stop_requested(bool requested)
{
    portENTER_CRITICAL(&s_state_mux);
    s_program_stop_requested = requested;
    portEXIT_CRITICAL(&s_state_mux);
}

robot_system_state_t robot_get_system_state(void)
{
    portENTER_CRITICAL(&s_state_mux);
    const bool armed = s_armed;
    const bool referenced = s_referenced;
    const bool operating = s_operating;
    const bool sync_ready = s_sync_ready;
    const bool gcode_running = s_gcode_running;
    const bool stop_requested = s_program_stop_requested;
    const robot_error_t last_error = s_last_error;
    portEXIT_CRITICAL(&s_state_mux);

    if (!armed) return ROBOT_STATE_DISARMED;
    if (last_error != ROBOT_ERROR_NONE) return ROBOT_STATE_ERROR;
    if (sync_ready) return ROBOT_STATE_READY_FOR_SYNC;
    if (stop_requested && (operating || gcode_running)) return ROBOT_STATE_STOPPING;
    if (operating || gcode_running) return ROBOT_STATE_RUNNING;
    if (!referenced) return ROBOT_STATE_UNREFERENCED;
    return ROBOT_STATE_READY;
}

const char *robot_get_system_state_name(void)
{
    switch (robot_get_system_state()) {
        case ROBOT_STATE_DISARMED: return "DISARMED";
        case ROBOT_STATE_UNREFERENCED: return "UNREFERENCED";
        case ROBOT_STATE_READY: return "READY";
        case ROBOT_STATE_RUNNING: return "RUNNING";
        case ROBOT_STATE_READY_FOR_SYNC: return "READY_FOR_SYNC";
        case ROBOT_STATE_ERROR: return "ERROR";
        case ROBOT_STATE_STOPPING: return "STOPPING";
        default: return "UNKNOWN";
    }
}

static inline void robot_tcp_estimate_invalidate(void)
{
    portENTER_CRITICAL(&s_state_mux);
    s_tcp_estimate_is_valid = false;
    portEXIT_CRITICAL(&s_state_mux);
}

static inline void robot_tcp_estimate_set_base(float x, float y, float z, float pitch_deg)
{
    portENTER_CRITICAL(&s_state_mux);
    s_tcp_est_base.x = x;
    s_tcp_est_base.y = y;
    s_tcp_est_base.z = z;
    s_tcp_est_base.pitch_deg = pitch_deg;
    s_tcp_estimate_is_valid = true;
    portEXIT_CRITICAL(&s_state_mux);
}

float robot_get_est_angle(int id)
{
    if (id < 0 || id >= SERVO_COUNT) return 0.0f;
    portENTER_CRITICAL(&s_state_mux);
    float v = s_last_q[id];
    portEXIT_CRITICAL(&s_state_mux);
    return v;
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
    portENTER_CRITICAL(&s_state_mux);
    bool v = s_referenced;
    portEXIT_CRITICAL(&s_state_mux);
    return v;
}

bool robot_has_tcp_estimate(void)
{
    portENTER_CRITICAL(&s_state_mux);
    bool v = s_tcp_estimate_is_valid;
    portEXIT_CRITICAL(&s_state_mux);
    return v;
}

void robot_clear_reference(void)
{
    portENTER_CRITICAL(&s_state_mux);
    s_referenced = false;
    s_tcp_estimate_is_valid = false;
    s_sync_ready = false;
    portEXIT_CRITICAL(&s_state_mux);
}

bool robot_get_tcp_estimate_base(robot_pose_t *pose)
{
    if (!pose) return false;
    portENTER_CRITICAL(&s_state_mux);
    if (!s_tcp_estimate_is_valid) {
        portEXIT_CRITICAL(&s_state_mux);
        return false;
    }
    *pose = s_tcp_est_base;
    portEXIT_CRITICAL(&s_state_mux);
    return true;
}

bool robot_get_tcp_estimate_work(robot_pose_t *pose)
{
    if (!pose) return false;
    portENTER_CRITICAL(&s_state_mux);
    if (!s_tcp_estimate_is_valid) {
        portEXIT_CRITICAL(&s_state_mux);
        return false;
    }
    *pose = s_tcp_est_base;
    portEXIT_CRITICAL(&s_state_mux);
    base_to_work_xyz(pose->x, pose->y, pose->z, &pose->x, &pose->y, &pose->z);
    return true;
}

// ===============================
// JOINT / SERVO / SENSOR I/O
// ===============================

// ===============================
// SERVO / SERVO MAPPING
// ===============================
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

// Servo and joint mapping helpers
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
    float a_lo = g_joint_limits[SERVO_J1_A].min_deg;
    float a_hi = g_joint_limits[SERVO_J1_A].max_deg;

    const float b_lo = g_joint_limits[SERVO_J1_B].min_deg;
    const float b_hi = g_joint_limits[SERVO_J1_B].max_deg;

    const float a_from_b_lo = b_lo - J1_B_TRIM_DEG;
    const float a_from_b_hi = b_hi - J1_B_TRIM_DEG;

    if (a_from_b_lo > a_lo) a_lo = a_from_b_lo;
    if (a_from_b_hi < a_hi) a_hi = a_from_b_hi;

    *lo = a_lo;
    *hi = a_hi;
}

// Joint limits helpers
static void joint_limits_get(int joint_id, float *lo, float *hi, float *vmax)
{
    int s = s_joint_master_servo[joint_id];
    float l = g_joint_limits[s].min_deg;
    float h = g_joint_limits[s].max_deg;
    float v = g_joint_limits[s].max_deg_s;

    if (s == SERVO_J1_A) {
        j1_a_allowed_range(&l, &h);

        float v2 = g_joint_limits[SERVO_J1_B].max_deg_s;
        if (v2 < v) v = v2;
    }

    *lo = l;
    *hi = h;
    *vmax = v;
}

// Initialization
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

// Low-level servo write
static void servo_write_angle_hw(int master, float angle, int other, float other_angle)
{
    uint32_t duty = angle_to_duty(master, angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[master].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[master].channel);

    if (other >= 0) {
        uint32_t duty2 = angle_to_duty(other, other_angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[other].channel, duty2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[other].channel);
    }
}

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

    if (master == SERVO_J1_A) {
        j1_a_allowed_range(&lo, &hi);
    }

    angle = clampf(angle, lo, hi);

    float other_angle = -1.0f;
    if (other >= 0) {
        float lo2 = g_joint_limits[other].min_deg;
        float hi2 = g_joint_limits[other].max_deg;
        other_angle = clampf(j1_b_from_j1_a(angle), lo2, hi2);
    }

    robot_tcp_estimate_invalidate();

    if (!s_armed) {
        portENTER_CRITICAL(&s_state_mux);
        s_last_q[master] = angle;
        if (other >= 0) s_last_q[other] = other_angle;
        portEXIT_CRITICAL(&s_state_mux);

        ledc_stop(LEDC_LOW_SPEED_MODE, servos[master].channel, 0);
        if (other >= 0) ledc_stop(LEDC_LOW_SPEED_MODE, servos[other].channel, 0);
        return;
    }

    servo_write_angle_hw(master, angle, other, other_angle);

    portENTER_CRITICAL(&s_state_mux);
    s_last_q[master] = angle;
    if (other >= 0) s_last_q[other] = other_angle;
    portEXIT_CRITICAL(&s_state_mux);
}

static void seed_last_q_from_pose(const float q_seed[SERVO_COUNT])
{
    float q[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) q[i] = q_seed[i];
    robot_validate_and_prepare_q(q, true);

    portENTER_CRITICAL(&s_state_mux);
    for (int i = 0; i < SERVO_COUNT; i++) s_last_q[i] = q[i];
    portEXIT_CRITICAL(&s_state_mux);
}

static void seed_last_q_from_home(void)
{
    seed_last_q_from_pose(s_home_q_init);
}

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
// SENSORS / SENSOR INIT
// ===============================
void sensors_init(void)
{
#if SENSOR_COUNT <= 0
    ESP_LOGI(TAG, "Sensorless mode: SENSOR_COUNT=0");
    return;
#else
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
#endif
}

// Sensor helpers
bool robot_sensors_initialized(void)
{
#if SENSOR_COUNT <= 0
    return false;
#else
    bool need_adc1 = false;
    bool need_adc2 = false;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensors[i].unit == ADC_UNIT_1) need_adc1 = true;
        else if (sensors[i].unit == ADC_UNIT_2) need_adc2 = true;
    }

    if (need_adc1 && !s_adc1) return false;
    if (need_adc2 && !s_adc2) return false;
    return true;
#endif
}

int sensor_read_raw(int id)
{
#if SENSOR_COUNT <= 0
    (void)id;
    return -1;
#else
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
#endif
}

float sensor_read_angle(int id)
{
    int raw = sensor_read_raw(id);
    if (raw < 0) return -1;
    return (raw / 4095.0f) * 180.0f;
}

// ===============================
// JOINT / MOTION VALIDATION
// ===============================
bool robot_validate_and_prepare_q(float q[SERVO_COUNT], bool clamp)
{
    bool ok = true;

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i == SERVO_J1_B) continue;

        float lo = g_joint_limits[i].min_deg;
        float hi = g_joint_limits[i].max_deg;

        if (i == SERVO_J1_A) {
            j1_a_allowed_range(&lo, &hi);
        }

        if (q[i] < lo || q[i] > hi) ok = false;
        if (clamp) q[i] = clampf(q[i], lo, hi);
    }

    float b = j1_b_from_j1_a(q[SERVO_J1_A]);
    float blo = g_joint_limits[SERVO_J1_B].min_deg;
    float bhi = g_joint_limits[SERVO_J1_B].max_deg;

    if (b < blo || b > bhi) ok = false;
    if (clamp) b = clampf(b, blo, bhi);

    q[SERVO_J1_B] = b;
    return ok;
}

bool robot_tcp_reachable(float x, float y, float z, float pitch_deg)
{
    const float r_j1 = planar_radius_from_j1(x, y);
    const float tool_len_cfg = ROBOT_TOOL_LEN;
    const float tool_len = (isfinite(tool_len_cfg) && tool_len_cfg > 0.0f) ? tool_len_cfg : 0.0f;
    const float pitch_signs[2] = { +1.0f, -1.0f };

    for (int i = 0; i < 2; i++) {
        const float phi = tcp_ik_pitch_rad(pitch_deg, pitch_signs[i]);
        const float r_w = r_j1 - tool_len * cosf(phi);
        const float z_w = z - tool_len * sinf(phi);
        const float z_sh = z_w - L0;
        const float d = sqrtf(r_w*r_w + z_sh*z_sh);

        if (!isfinite(d)) continue;
        if (d > (L1 + L2)) continue;
        if (d < fabsf(L1 - L2)) continue;
        return true;
    }

    return false;
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
static bool inverse_kinematics(float x, float y, float z,
                                   float tool_pitch_deg,
                                   float q_target[SERVO_COUNT])
{
    float last_q[SERVO_COUNT];
    portENTER_CRITICAL(&s_state_mux);
    for (int i = 0; i < SERVO_COUNT; i++) last_q[i] = s_last_q[i];
    portEXIT_CRITICAL(&s_state_mux);

    const float r_base = planar_radius_from_base(x, y);
    const float r_j1 = planar_radius_from_j1(x, y);

    float q0;
    if (r_base < 1e-6f) {
        const float d0 = DIR[SERVO_J0];
        q0 = (fabsf(d0) < 1e-6f) ? 0.0f
                                 : DEG2RAD((last_q[SERVO_J0] - OFF[SERVO_J0]) / d0);
    } else {
        q0 = atan2f(y, x); // base angle
    }

    const float tool_len_cfg = ROBOT_TOOL_LEN;
    const float tool_len = (isfinite(tool_len_cfg) && tool_len_cfg > 0.0f) ? tool_len_cfg : 0.0f;
    const float pitch_signs[2] = { +1.0f, -1.0f }; // <-89..+89>

    float best_q[SERVO_COUNT];
    float best_cost = 1e30f;
    bool best_ok = false;

    for (int ps = 0; ps < 2; ps++) {
        const float phi = tcp_ik_pitch_rad(tool_pitch_deg, pitch_signs[ps]);
        const float r_w = r_j1 - tool_len * cosf(phi);
        const float z_w = z - tool_len * sinf(phi);
        const float z_sh = z_w - L0;

        const float d2 = r_w*r_w + z_sh*z_sh;
        const float d = sqrtf(d2);

        if (!isfinite(d)) continue;
        if (d > (L1 + L2)) continue;
        if (d < fabsf(L1 - L2)) continue;

        float cos_q2 = (d2 - L1*L1 - L2*L2) / (2.0f * L1 * L2); // saturation <-1..1> due to numerical errors
        if (cos_q2 > 1.0f) cos_q2 = 1.0f;
        if (cos_q2 < -1.0f) cos_q2 = -1.0f;

        const float q2_candidates[2] = { -acosf(cos_q2), +acosf(cos_q2) };

        for (int b = 0; b < 2; b++) {
            const float q2 = q2_candidates[b];

            const float phi2 = atan2f(z_sh, r_w); // 
            const float psi = atan2f(L2 * sinf(q2), L1 + L2 * cosf(q2));
            const float q1 = phi2 - psi;

            const float q3_raw = phi - q1 - q2; // q3 2*pi | find the best match  limits

            for (int k = -1; k <= 1; k++) {
                const float q3 = q3_raw + (float)k * 2.0f * (float)M_PI;

                const float j0 = RAD2DEG(q0);
                const float j1 = RAD2DEG(q1);
                const float j2 = RAD2DEG(q2);
                const float j3 = RAD2DEG(q3);

                float cand[SERVO_COUNT];
                for (int i = 0; i < SERVO_COUNT; i++) cand[i] = last_q[i];

                cand[SERVO_J0] = map_servo(SERVO_J0, j0);
                cand[SERVO_J1_A] = map_servo(SERVO_J1_A, j1);
                cand[SERVO_J2] = map_servo(SERVO_J2, j2);
                cand[SERVO_J3] = map_servo(SERVO_J3, j3);

                if (!robot_validate_and_prepare_q(cand, false)) continue;

                float cost = 0.0f;
                cost += fabsf(cand[SERVO_J0] - last_q[SERVO_J0]);
                cost += fabsf(cand[SERVO_J1_A] - last_q[SERVO_J1_A]);
                cost += fabsf(cand[SERVO_J2] - last_q[SERVO_J2]);
                cost += fabsf(cand[SERVO_J3] - last_q[SERVO_J3]);
                cost += (ps == 0) ? 0.0f : 0.01f;

                if (cost < best_cost) {
                    best_cost = cost;
                    for (int i = 0; i < SERVO_COUNT; i++) best_q[i] = cand[i];
                    best_ok = true;
                }
            }
        }
    }

    if (!best_ok) return false;
    for (int i = 0; i < SERVO_COUNT; i++) q_target[i] = best_q[i];
    return true;
}

bool robot_ik_tcp(float x, float y, float z, float pitch_deg, float q_target[SERVO_COUNT])
{
    if (!inverse_kinematics(x, y, z, pitch_deg, q_target)) return false;

    robot_validate_and_prepare_q(q_target, true);
    return true;
}

// ===============================
// PLANNER / EXECUTOR
// ===============================
static bool seg_full(void)  { return ((s_seg_w + 1) % SEG_BUF_LEN) == s_seg_r; }
static bool seg_empty(void) { return s_seg_w == s_seg_r; }

static void seg_flush(void) {
    s_seg_w = s_seg_r = 0;
    s_current_segment.active = false;
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

static void planner_get_tail_q(float q[SERVO_COUNT])
{
    if (!seg_empty()) {
        int idx = (s_seg_w - 1 + SEG_BUF_LEN) % SEG_BUF_LEN;
        for (int i = 0; i < SERVO_COUNT; i++) q[i] = s_seg_buf[idx].q1[i];
        return;
    }

    if (s_current_segment.active) {
        for (int i = 0; i < SERVO_COUNT; i++) q[i] = s_current_segment.q1[i];
        return;
    }

    portENTER_CRITICAL(&s_state_mux);
    for (int i = 0; i < SERVO_COUNT; i++) q[i] = s_last_q[i];
    portEXIT_CRITICAL(&s_state_mux);
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

static void robot_control_task(void *arg)
{
    (void)arg;

    for (int i = 0; i < SERVO_COUNT; i++) s_last_q[i] = 90.0f;
    
    seed_last_q_from_home();
    robot_clear_reference();
    robot_clear_error();

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

                s_operating = false;
                vTaskDelay(pdMS_TO_TICKS(EXEC_DT_MS));
                continue;
            }

        disarmed_latched = false;

        if (xQueueReceive(s_robot_queue, &cmd, pdMS_TO_TICKS(EXEC_DT_MS)) == pdTRUE) {

            if (cmd.type == ROBOT_CMD_QUEUE_FLUSH) {
                seg_flush();
                s_operating = false;
                continue;
            }

            if (seg_full()) {
                ESP_LOGW(TAG, "Planner full, dropping command");
                s_operating = s_current_segment.active || !seg_empty();
                continue;
            }

            traj_seg_t seg = {0};
            planner_get_tail_q(seg.q0);
            seg.tcp_target_valid = false;
            seg.preserve_tcp_estimate = false;
            seg.mark_referenced_on_finish = false;

            switch (cmd.type) {
            case ROBOT_CMD_MOVE_JOINTS:
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = cmd.q_target[i];
                robot_validate_and_prepare_q(seg.q1, true);
                seg.T = (cmd.duration_s > 0.0f) ? cmd.duration_s : 1.0f;
                seg.tcp_target_valid = cmd.tcp_target_valid;
                seg.mark_referenced_on_finish = cmd.mark_referenced_on_finish;
                seg.tcp_target_base.x = cmd.x;
                seg.tcp_target_base.y = cmd.y;
                seg.tcp_target_base.z = cmd.z;
                seg.tcp_target_base.pitch_deg = cmd.pitch_deg;
                break;

            case ROBOT_CMD_MOVE_XYZ: {
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = seg.q0[i];

                float pitch = clamp_pitch_deg(cmd.pitch_deg);

                ESP_LOGD(TAG, "IK TCP(base): x=%.1f y=%.1f z=%.1f pitch=%.1f",
                         cmd.x, cmd.y, cmd.z, pitch);

                bool ok = inverse_kinematics(cmd.x, cmd.y, cmd.z, pitch, seg.q1);
                if (!ok) {
                    ESP_LOGE(TAG, "IK TCP failed for BASE x=%.1f y=%.1f z=%.1f pitch=%.1f",
                             cmd.x, cmd.y, cmd.z, pitch);

                    if (robot_is_program_running()) {
                        robot_set_error_internal(ROBOT_ERROR_GCODE);
                        gcode_fail_external("Planner IK failed for queued XYZ move");

                        seg_flush();
                        while (xQueueReceive(s_robot_queue, &cmd, 0) == pdTRUE) {
                        }
                        s_operating = false;
                    } else {
                        s_operating = s_current_segment.active || !seg_empty();
                    }
                    continue;
                }

                ESP_LOGD(TAG, "IK servo: s0(J0)=%.1f s1(J1)=%.1f s3(J2)=%.1f s4(J3)=%.1f",
                         seg.q1[0], seg.q1[1], seg.q1[3], seg.q1[4]);

                robot_validate_and_prepare_q(seg.q1, true);
                seg.T = (cmd.duration_s > 0.0f) ? cmd.duration_s : 1.0f;
                seg.tcp_target_valid = true;
                seg.tcp_target_base.x = cmd.x;
                seg.tcp_target_base.y = cmd.y;
                seg.tcp_target_base.z = cmd.z;
                seg.tcp_target_base.pitch_deg = pitch;
                break;
            }

            case ROBOT_CMD_MOVE_JOINTS_T:
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = cmd.q_target[i];
                robot_validate_and_prepare_q(seg.q1, true);
                seg.T = cmd.duration_s;
                seg.tcp_target_valid = cmd.tcp_target_valid;
                seg.mark_referenced_on_finish = cmd.mark_referenced_on_finish;
                seg.tcp_target_base.x = cmd.x;
                seg.tcp_target_base.y = cmd.y;
                seg.tcp_target_base.z = cmd.z;
                seg.tcp_target_base.pitch_deg = cmd.pitch_deg;
                break;

            case ROBOT_CMD_DWELL:
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = seg.q0[i];
                seg.T = ((float)cmd.dwell_ms) / 1000.0f;
                seg.preserve_tcp_estimate = true;
                break;

            case ROBOT_CMD_GRIPPER: {
                for (int i = 0; i < SERVO_COUNT; i++) seg.q1[i] = seg.q0[i];
                const float lo = g_joint_limits[SERVO_J5].min_deg;
                const float hi = g_joint_limits[SERVO_J5].max_deg;
                seg.q1[SERVO_J5] = clampf(cmd.gripper_deg, lo, hi);
                seg.T = (cmd.duration_s > 0.0f) ? cmd.duration_s : GRIPPER_MOVE_T_S;
                seg.preserve_tcp_estimate = true;
                break;
            }

            default:
                ESP_LOGW(TAG, "Unknown robot command: %d", cmd.type);
                s_operating = s_current_segment.active || !seg_empty();
                continue;
            }

            float Tmin = robot_min_time_for_move(seg.q0, seg.q1);
            if (seg.T < Tmin) seg.T = Tmin;
            if (seg.T < MIN_SEG_T) seg.T = MIN_SEG_T;

            seg.t = 0;
            seg.active = false;

            seg_push(&seg);
        }

        if (!s_current_segment.active) {
            if (!seg_pop(&s_current_segment)) {
                s_operating = false;
                continue;
            }
            s_current_segment.t = 0;
            s_current_segment.active = true;

            if (s_current_segment.preserve_tcp_estimate) {
                robot_pose_t pose;
                if (robot_get_tcp_estimate_base(&pose)) {
                    s_current_segment.tcp_target_base = pose;
                    s_current_segment.tcp_target_valid = true;
                } else {
                    s_current_segment.tcp_target_valid = false;
                }
            }
        }

        s_current_segment.t += EXEC_DT_S;
        float s = (s_current_segment.T > 1e-6f) ? (s_current_segment.t / s_current_segment.T) : 1.0f;
        float a = smoothstep(s);

        float q[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) {
            q[i] = s_current_segment.q0[i] + (s_current_segment.q1[i] - s_current_segment.q0[i]) * a;
        }

        robot_validate_and_prepare_q(q, true);
        apply_joints(q);

        if (s >= 1.0f) {
            portENTER_CRITICAL(&s_state_mux);
            for (int i = 0; i < SERVO_COUNT; i++) s_last_q[i] = s_current_segment.q1[i];
            portEXIT_CRITICAL(&s_state_mux);

            if (s_current_segment.preserve_tcp_estimate) {
                if (s_current_segment.tcp_target_valid) {
                    robot_tcp_estimate_set_base(s_current_segment.tcp_target_base.x,
                                                s_current_segment.tcp_target_base.y,
                                                s_current_segment.tcp_target_base.z,
                                                s_current_segment.tcp_target_base.pitch_deg);
                } else {
                    robot_tcp_estimate_invalidate();
                }
            } else if (s_current_segment.tcp_target_valid) {
                robot_tcp_estimate_set_base(s_current_segment.tcp_target_base.x,
                                            s_current_segment.tcp_target_base.y,
                                            s_current_segment.tcp_target_base.z,
                                            s_current_segment.tcp_target_base.pitch_deg);
            } else {
                robot_tcp_estimate_invalidate();
            }

            if (s_current_segment.mark_referenced_on_finish) {
                portENTER_CRITICAL(&s_state_mux);
                s_referenced = true;
                portEXIT_CRITICAL(&s_state_mux);
                robot_clear_error();
                gcode_reset();
                ESP_LOGI(TAG, "Robot reference established");
            }

            s_current_segment.active = false;
        }

        s_operating = s_current_segment.active || !seg_empty();
    }
}

bool robot_is_armed(void)
{
    return s_armed;
}

bool robot_is_operating(void)
{
    return s_operating;
}

void robot_stop_all(void)
{
    gcode_stop();
}

bool robot_cmd_home_reference(void)
{
    if (!s_armed || s_robot_queue == NULL) return false;

    const bool blind_recovery = !robot_is_referenced() || !robot_has_tcp_estimate();

    robot_stop_all();

    if (blind_recovery) {
        seed_last_q_from_pose(s_home_pre_q_init);
    }

    bool ok = robot_cmd_move_joints_t(s_home_pre_q_init, HOME_PREMOVE_T_S, 0);
    if (ok) {
        ok = robot_cmd_move_joints_home(s_home_q_init,
                                        ROBOT_HOME_X_BASE_DEFAULT,
                                        ROBOT_HOME_Y_BASE_DEFAULT,
                                        ROBOT_HOME_Z_BASE_DEFAULT,
                                        ROBOT_HOME_PITCH_DEG_DEFAULT);
    }
    if (ok) {
        robot_clear_error();
    } else {
        robot_set_error_internal(ROBOT_ERROR_HOME);
    }
    return ok;
}

void robot_disarm(void)
{
    s_armed = false;
    s_operating = false;
    robot_stop_all();
    robot_clear_reference();
    ESP_LOGI(TAG, "Disarmed: reference invalidated");

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, servos[i].channel, 0);
    }
}

void robot_arm(void)
{
    robot_clear_error();
    robot_set_sync_ready(false);
    s_armed = true;
}

// ===============================
// CONTROL API
// ===============================
void robot_control_start(void)
{
    s_robot_queue = xQueueCreate(ROBOT_CMD_QUEUE_LEN, sizeof(robot_cmd_t));
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

    bool queued = (xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE);
    if (queued) robot_set_sync_ready(false);
    return queued;
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

    bool queued = (xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE);
    if (queued) robot_set_sync_ready(false);
    return queued;
}

bool robot_cmd_move_xyz(float x, float y, float z, float pitch_deg)
{
    return robot_cmd_move_xyz_t(x, y, z, pitch_deg, 0.0f, 0);
}

bool robot_cmd_move_xyz_t(float x, float y, float z, float pitch_deg, float duration_s, TickType_t timeout)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_XYZ;
    cmd.x = x; cmd.y = y; cmd.z = z;
    cmd.pitch_deg = pitch_deg;
    cmd.duration_s = duration_s;

    bool queued = (xQueueSend(s_robot_queue, &cmd, timeout) == pdTRUE);
    if (queued) robot_set_sync_ready(false);
    return queued;
}

bool robot_cmd_move_xyz_work(float x, float y, float z, float pitch_deg)
{
    return robot_cmd_move_xyz_work_t(x, y, z, pitch_deg, 0.0f, 0);
}

bool robot_cmd_move_xyz_work_t(float x, float y, float z, float pitch_deg, float duration_s, TickType_t timeout)
{
    if (!s_referenced) {
        ESP_LOGW(TAG, "Rejecting work-frame move: robot not referenced");
        return false;
    }

    float xb, yb, zb;
    work_to_base_xyz(x, y, z, &xb, &yb, &zb);
    return robot_cmd_move_xyz_t(xb, yb, zb, pitch_deg, duration_s, timeout);
}

bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_JOINTS_T;
    for (int i = 0; i < SERVO_COUNT; i++) cmd.q_target[i] = q_target[i];
    cmd.duration_s = duration_s;

    bool queued = (xQueueSend(s_robot_queue, &cmd, timeout) == pdTRUE);
    if (queued) robot_set_sync_ready(false);
    return queued;
}

bool robot_cmd_dwell_ms(uint32_t dwell_ms, TickType_t timeout)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_DWELL;
    cmd.dwell_ms = dwell_ms;

    bool queued = (xQueueSend(s_robot_queue, &cmd, timeout) == pdTRUE);
    if (queued) robot_set_sync_ready(false);
    return queued;
}

bool robot_cmd_gripper_set(float gripper_deg, TickType_t timeout)
{
    if (!s_armed) return false;
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_GRIPPER;
    cmd.gripper_deg = gripper_deg;
    cmd.duration_s = GRIPPER_MOVE_T_S;

    bool queued = (xQueueSend(s_robot_queue, &cmd, timeout) == pdTRUE);
    if (queued) robot_set_sync_ready(false);
    return queued;
}

void robot_cmd_queue_flush(void)
{
    if (s_robot_queue == NULL) return;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_QUEUE_FLUSH;
    (void)xQueueSend(s_robot_queue, &cmd, 0);
    robot_set_sync_ready(false);
}

// ===============================
// GCODE EXECUTION
// ===============================
static void gcode_executor_task(void *arg)
{
    gcode_task_params_t *params = (gcode_task_params_t *)arg;

    ESP_LOGI(TAG, "Starting G-Code task for file: %s", params->filename);
    bool res = gcode_run_file(params->filename);

    if (res) {
        while (robot_is_operating()) {
            vTaskDelay(pdMS_TO_TICKS(EXEC_DT_MS));
        }
        robot_clear_error();
        ESP_LOGI(TAG, "G-Code finished successfully");
    } else {
        robot_set_error_internal(ROBOT_ERROR_GCODE);
        ESP_LOGE(TAG, "G-Code failed");
    }

    free(params);
    portENTER_CRITICAL(&s_state_mux);
    s_gcode_running = false;
    s_program_stop_requested = false;
    portEXIT_CRITICAL(&s_state_mux);
    vTaskDelete(NULL);
}

bool robot_core_run_gcode(const char *filename)
{
    if (!s_armed) return false;
    if (!filename || filename[0] == '\0') return false;
    if (robot_is_program_running()) return false;

    gcode_task_params_t *params = malloc(sizeof(gcode_task_params_t));
    if (!params) {
        ESP_LOGE(TAG, "Failed to allocate memory for G-code task");
        return false;
    }

    strncpy(params->filename, filename, sizeof(params->filename) - 1);
    params->filename[sizeof(params->filename) - 1] = '\0';
    robot_set_sync_ready(false);
    robot_clear_error();
    portENTER_CRITICAL(&s_state_mux);
    s_gcode_running = true;
    s_program_stop_requested = false;
    portEXIT_CRITICAL(&s_state_mux);

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
        portENTER_CRITICAL(&s_state_mux);
        s_gcode_running = false;
        s_program_stop_requested = false;
        portEXIT_CRITICAL(&s_state_mux);
        free(params);
        return false;
    }

    return true;
}
