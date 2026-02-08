// ===============================
// robot_io.c
// ===============================

#include "robot_io.h"

static const char *TAG = "robot_io";
static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_oneshot_unit_handle_t s_adc2 = NULL;

servo_t servos[SERVO_COUNT] = {
    { .gpio_num = SERVO0_GPIO, .channel = SERVO0_CH },
    { .gpio_num = SERVO1_GPIO, .channel = SERVO1_CH },
    { .gpio_num = SERVO2_GPIO, .channel = SERVO2_CH },
    { .gpio_num = SERVO3_GPIO, .channel = SERVO3_CH },
    { .gpio_num = SERVO4_GPIO, .channel = SERVO4_CH },
    { .gpio_num = SERVO5_GPIO, .channel = SERVO5_CH },
    //{ .gpio_num = GPIO_NUM_42, .channel = LEDC_CHANNEL_6 }, (manipulator)
};

sensor_t sensors[SENSOR_COUNT] = {
    { .unit = S0_ADC_UNIT, .channel = S0_ADC_CH },
    { .unit = S1_ADC_UNIT, .channel = S1_ADC_CH },
    { .unit = S2_ADC_UNIT, .channel = S2_ADC_CH },
    { .unit = S3_ADC_UNIT, .channel = S3_ADC_CH },
    { .unit = S4_ADC_UNIT, .channel = S4_ADC_CH },
    { .unit = S5_ADC_UNIT, .channel = S5_ADC_CH },
    //{ .unit = ADC_UNIT_2, .channel = ADC_CHANNEL_7 },  // IO18 (reservation)
};

const joint_limits_t g_joint_limits[SERVO_COUNT] = {
    { .min_deg = J0_MIN, .max_deg = J0_MAX, .max_deg_s = J0_V },
    { .min_deg = J1_MIN, .max_deg = J1_MAX, .max_deg_s = J1_V },
    { .min_deg = J2_MIN, .max_deg = J2_MAX, .max_deg_s = J2_V },
    { .min_deg = J3_MIN, .max_deg = J3_MAX, .max_deg_s = J3_V },
    { .min_deg = J4_MIN, .max_deg = J4_MAX, .max_deg_s = J4_V },
    { .min_deg = J5_MIN, .max_deg = J5_MAX, .max_deg_s = J5_V },
};

typedef struct {
    char filename[64];
} gcode_task_params_t;

// Queue for robot commands
static QueueHandle_t s_robot_queue = NULL;

// Trajectory segment buffer
static traj_seg_t s_seg_buf[SEG_BUF_LEN];
static int s_seg_w = 0, s_seg_r = 0;
static traj_seg_t s_cur = {0};
static float s_last_q[SERVO_COUNT] = {0};

// ===============================
// INICIALIZATION OF SERVOS
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

    if (SENSOR_COUNT == SERVO_COUNT) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
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
    else
    {
        ESP_LOGE(TAG, "Servo and sensor count mismatch");
    }
}

// ===============================
// HELPER FUNCTIONS
// ===============================

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

bool robot_validate_and_prepare_q(float q[SERVO_COUNT], bool clamp)
{
    bool ok = true;
    for (int i = 0; i < SERVO_COUNT; i++) {
        float lo = g_joint_limits[i].min_deg;
        float hi = g_joint_limits[i].max_deg;
        if (q[i] < lo || q[i] > hi) ok = false;
        if (clamp) q[i] = clampf(q[i], lo, hi);
    }
    return ok;
}

bool robot_xyz_reachable(float x, float y, float z)
{
    float R = sqrtf(x*x + y*y);
    float d = sqrtf(R*R + z*z);

    if (d > (L1 + L2)) return false;
    if (d < fabsf(L1 - L2)) return false;

    return true;
}

float robot_min_time_for_move(const float q0[SERVO_COUNT], const float q1[SERVO_COUNT])
{
    float Tmin = 0.0f;
    for (int i = 0; i < SERVO_COUNT; i++) {
        float diff = fabsf(q1[i] - q0[i]);
        float v = g_joint_limits[i].max_deg_s;
        if (v > 1e-6f) {
            float t = diff / v;
            if (t > Tmin) Tmin = t;
        }
    }
    return Tmin;
}

// ===============================
// INVERSE KINEMATICS (triangulation)
// ===============================
void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT])
{
    float q0 = atan2f(y, x); // base angle

    float R = sqrtf(x*x + y*y);
    float d = sqrtf(R*R + z*z);

    float cos_q2 = (d*d - L1*L1 - L2*L2) / (2*L1*L2);
    if (cos_q2 > 1.0f) cos_q2 = 1.0f;
    if (cos_q2 < -1.0f) cos_q2 = -1.0f;
    float q2 = acosf(cos_q2); // elbow angle

    float phi = atan2f(z, R);
    float psi = atan2f(L2*sin(q2), L1 + L2*cos_q2);
    float q1 = phi - psi; // shoulder angle

    q_target[0] = RAD2DEG(q0);
    q_target[1] = RAD2DEG(q1);
    q_target[2] = RAD2DEG(q2);
    for (int i = 3; i < SERVO_COUNT; i++) q_target[i] = 90;
}

// ===============================
// MOVE TO POSITION (interpolated)
// ===============================
void move_to_position(float q_target[SERVO_COUNT]) {
    float q_current[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        q_current[i] = sensor_read_angle(i);
    }

    float max_diff = 0; // maximum angle difference
    for (int i = 0; i < SERVO_COUNT; i++) {
        float diff = fabsf(q_target[i] - q_current[i]);
        if (diff > max_diff) max_diff = diff;
    }

    int steps = INTERP_STEPS;
    if (max_diff > 0) {
        for (int s = 0; s <= steps; s++) { // interpolation steps
            for (int i = 0; i < SERVO_COUNT; i++) {
                float q = q_current[i] + (q_target[i] - q_current[i]) * ((float)s / steps); 
                servo_set_angle(i, q);
            }
            vTaskDelay(pdMS_TO_TICKS(INTERP_DELAY_MS));
        }
    }
}

// ===============================
// INICIALIZATION OF SENSORS
// ===============================
void sensors_init(void)
{
    bool need_adc1 = false;
    bool need_adc2 = false;

    for (int i = 0; i < SENSOR_COUNT; i++) { // check which ADC units are needed
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
        .atten    = ADC_ATTEN_DB_12,   // 0–3.3 V
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
// SET SERVO ANGLE (0–180°)
// ===============================
void servo_set_angle(int servo_id, float angle) {
    if (servo_id < 0 || servo_id >= SERVO_COUNT) {
        ESP_LOGW(TAG, "Invalid servo ID: %d", servo_id);
        return;
    }

    angle = clampf(angle,
                   g_joint_limits[servo_id].min_deg,
                   g_joint_limits[servo_id].max_deg);

    uint32_t duty_min = (uint32_t)(0.5f / 20.0f * 16384);
    uint32_t duty_max = (uint32_t)(2.5f / 20.0f * 16384);
    uint32_t duty = duty_min + ((duty_max - duty_min) * angle) / 180;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[servo_id].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[servo_id].channel);
}

// ===============================
// SENSOR READ RAW DATA (0–4095)
// ===============================
int sensor_read_raw(int id)
{
    if (id < 0 || id >= SENSOR_COUNT) {
        return -1;
    }

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
float sensor_read_angle(int id) {
    int raw = sensor_read_raw(id);
    if (raw < 0) return -1;
    return (raw / 4095.0f) * 180.0f;
}

// ===============================
// ROBOT SEGMENT BUFFER MANAGEMENT
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

static float smoothstep(float s) { // 0..1
    if (s < 0) s = 0;
    if (s > 1) s = 1;
    return 3.0f*s*s - 2.0f*s*s*s;
}

static void apply_joints(const float q[SERVO_COUNT]) {
    for (int i = 0; i < SERVO_COUNT; i++) servo_set_angle(i, q[i]);
}

// ===============================
// ROBOT CONTROL TASK
// ===============================
static void robot_control_task(void *arg)
{
    for (int i = 0; i < SERVO_COUNT; i++) {
        float a = sensor_read_angle(i);
        if (a < 0) a = 90.0f;
        a = clampf(a, g_joint_limits[i].min_deg, g_joint_limits[i].max_deg);
        s_last_q[i] = a;
    }

    robot_cmd_t cmd;

    for (;;) {
        if (xQueueReceive(s_robot_queue, &cmd, pdMS_TO_TICKS(EXEC_DT_MS)) == pdTRUE) {

            if (cmd.type == ROBOT_CMD_QUEUE_FLUSH) {
                seg_flush();
                continue;
            }

            if (seg_full()) { // full planner buffer
                ESP_LOGW(TAG, "Planner full, dropping command");
                continue;
            }

            traj_seg_t s = {0};

            // start = last_q
            for (int i = 0; i < SERVO_COUNT; i++) s.q0[i] = s_last_q[i];

            if (cmd.type == ROBOT_CMD_MOVE_JOINTS) {
                for (int i = 0; i < SERVO_COUNT; i++) s.q1[i] = cmd.q_target[i];
                robot_validate_and_prepare_q(s.q1, true);
                s.T = 1.0f;
            }
            else if (cmd.type == ROBOT_CMD_MOVE_XYZ) {
                if (!robot_xyz_reachable(cmd.x, cmd.y, cmd.z)) {
                    ESP_LOGW(TAG, "XYZ out of reach: x=%.1f y=%.1f z=%.1f", cmd.x, cmd.y, cmd.z);
                    continue;
                }
                inverse_kinematics(cmd.x, cmd.y, cmd.z, s.q1);
                robot_validate_and_prepare_q(s.q1, true);
                s.T = 1.0f;
            }
            else if (cmd.type == ROBOT_CMD_MOVE_JOINTS_T) {
                for (int i = 0; i < SERVO_COUNT; i++) s.q1[i] = cmd.q_target[i];
                robot_validate_and_prepare_q(s.q1, true);
                s.T = cmd.duration_s;
            }
            else {
                ESP_LOGW(TAG, "Unknown robot command: %d", cmd.type);
                continue;
            }

            float Tmin = robot_min_time_for_move(s.q0, s.q1);
            if (s.T < Tmin) s.T = Tmin;
            if (s.T < MIN_SEG_T) s.T = MIN_SEG_T;

            s.t = 0;
            s.active = false;

            seg_push(&s);
        }

        // 2) tick executor
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

        apply_joints(q);

        if (s >= 1.0f) {
            // segment ready -> update last_q
            for (int i = 0; i < SERVO_COUNT; i++) s_last_q[i] = s_cur.q1[i];
            s_cur.active = false;
        }
    }
}

// ===============================
// START ROBOT CONTROL (queue + task)
// ===============================
void robot_control_start(void)
{
    // Create command queue
    s_robot_queue = xQueueCreate(8, sizeof(robot_cmd_t));
    if (s_robot_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create robot queue");
        return;
    }

    BaseType_t res = xTaskCreatePinnedToCore(robot_control_task, "robot_ctrl", 4096, NULL, 6, NULL, CORE_ROBOT);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create robot_control_task");
    }
}

// ===============================
// Send command - MOVE_JOINTS
// ===============================
bool robot_cmd_move_joints(const float q_target[SERVO_COUNT])
{
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_JOINTS;
    for (int i = 0; i < SERVO_COUNT; i++) {
        cmd.q_target[i] = q_target[i];
    }

    return xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE;
}

// ===============================
// Send command - MOVE_XYZ
// ===============================
bool robot_cmd_move_xyz(float x, float y, float z)
{
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_XYZ;
    cmd.x = x;
    cmd.y = y;
    cmd.z = z;

    return xQueueSend(s_robot_queue, &cmd, 0) == pdTRUE;
}

// ===============================
// Send command - MOVE_JOINTS_T
// ===============================
bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout)
{
    if (s_robot_queue == NULL) return false;

    robot_cmd_t cmd = {0};
    cmd.type = ROBOT_CMD_MOVE_JOINTS_T;
    for (int i = 0; i < SERVO_COUNT; i++) {
        cmd.q_target[i] = q_target[i];
    }
    cmd.duration_s = duration_s;

    return xQueueSend(s_robot_queue, &cmd, timeout) == pdTRUE;
}

// ===============================
// Send command - QUEUE_FLUSH
// ===============================
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

    if (res) {
        ESP_LOGI(TAG, "G-Code finished successfully");
    } else {
        ESP_LOGE(TAG, "G-Code failed");
    }

    free(params);
    vTaskDelete(NULL);
}

void robot_core_run_gcode(const char *filename)
{
    gcode_task_params_t *params = malloc(sizeof(gcode_task_params_t));
    if (params) {
        strncpy(params->filename, filename, 64);

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
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for G-code task");
    }
}