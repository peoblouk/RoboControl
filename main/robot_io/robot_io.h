// ===============================
// robot_io.h
// ===============================

#ifndef ROBOT_IO
#define ROBOT_IO

#include "config.h"

#include "esp_log.h"
#include "math.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdbool.h>

// Radians / Degrees conversion
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

#ifndef ROBOT_WORK_OFFSET_X_DEFAULT
#define ROBOT_WORK_OFFSET_X_DEFAULT 220.0f
#endif
#ifndef ROBOT_WORK_OFFSET_Y_DEFAULT
#define ROBOT_WORK_OFFSET_Y_DEFAULT 0.0f
#endif
#ifndef ROBOT_WORK_OFFSET_Z_DEFAULT
#define ROBOT_WORK_OFFSET_Z_DEFAULT 25.0f
#endif
#ifndef ROBOT_HOME_X_BASE_DEFAULT
#define ROBOT_HOME_X_BASE_DEFAULT ROBOT_WORK_OFFSET_X_DEFAULT
#endif
#ifndef ROBOT_HOME_Y_BASE_DEFAULT
#define ROBOT_HOME_Y_BASE_DEFAULT ROBOT_WORK_OFFSET_Y_DEFAULT
#endif
#ifndef ROBOT_HOME_Z_BASE_DEFAULT
#define ROBOT_HOME_Z_BASE_DEFAULT 80.0f
#endif
#ifndef ROBOT_HOME_PITCH_DEG_DEFAULT
#define ROBOT_HOME_PITCH_DEG_DEFAULT 0.0f
#endif

// ===============================
// SERVO CONFIGURATION (PWM output)
// ===============================
typedef struct {
    ledc_channel_t channel;
    gpio_num_t gpio_num;
} servo_t;

typedef struct {
    uint16_t min_us;
    uint16_t max_us;
} servo_pwm_range_t;

extern servo_t servos[SERVO_COUNT];

// ===============================
// SENSOR CONFIGURATION (ADC input)
// ===============================
typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;
} sensor_t;

extern sensor_t sensors[SENSOR_COUNT];

// ===============================
// JOINT LIMITS
// ===============================
typedef struct {
    float min_deg;
    float max_deg;
    float max_deg_s;
} joint_limits_t;

extern const joint_limits_t g_joint_limits[SERVO_COUNT];

// ===============================
// TCP / POSE TYPES
// ===============================
typedef struct {
    float x;
    float y;
    float z;
    float pitch_deg;
} robot_pose_t;

// ===============================
// TRAJECTORY SEGMENT STRUCTURE
// ===============================
typedef struct {
    float q0[SERVO_COUNT];
    float q1[SERVO_COUNT];
    float T;
    float t;
    bool  active;

    bool  tcp_target_valid;
    bool  mark_referenced_on_finish;
    robot_pose_t tcp_target_base;
} traj_seg_t;

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void servos_init(void);
void sensors_init(void);

int   sensor_read_raw(int id);
float sensor_read_angle(int id);

void servo_set_angle(int servo_id, float angle);
void joint_set_angle(int joint_id, float angle);

bool  robot_validate_and_prepare_q(float q[SERVO_COUNT], bool clamp);
bool  robot_tcp_reachable(float x, float y, float z, float pitch_deg);
bool  robot_tcp_reachable_work(float x, float y, float z, float pitch_deg);
float robot_min_time_for_move(const float q0[SERVO_COUNT], const float q1[SERVO_COUNT]);

float robot_get_est_angle(int id);

void robot_set_work_offset(float x, float y, float z);
void robot_get_work_offset(float *x, float *y, float *z);

bool robot_is_referenced(void);
bool robot_has_tcp_estimate(void);
void robot_clear_reference(void);

bool robot_get_tcp_estimate_base(robot_pose_t *pose);
bool robot_get_tcp_estimate_work(robot_pose_t *pose);

// Type of robot command
typedef enum {
    ROBOT_CMD_NONE = 0,
    ROBOT_CMD_MOVE_JOINTS,
    ROBOT_CMD_MOVE_XYZ,
    ROBOT_CMD_MOVE_JOINTS_T,
    ROBOT_CMD_QUEUE_FLUSH
} robot_cmd_type_t;

// Command structure
typedef struct {
    robot_cmd_type_t type;
    float q_target[SERVO_COUNT];
    float x, y, z;
    float pitch_deg;
    float duration_s;

    bool mark_referenced_on_finish;
    bool tcp_target_valid;
} robot_cmd_t;

// ===============================
// CONTROL API
// ===============================
void robot_control_start(void);
bool robot_cmd_move_joints(const float q_target[SERVO_COUNT]);
bool robot_cmd_move_joints_home(const float q_target[SERVO_COUNT],
                                float home_x_base,
                                float home_y_base,
                                float home_z_base,
                                float home_pitch_deg);
bool robot_cmd_move_xyz(float x, float y, float z, float pitch_deg);
bool robot_cmd_move_xyz_work(float x, float y, float z, float pitch_deg);
bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout);
void robot_cmd_queue_flush(void);
void robot_core_run_gcode(const char *filename);

void robot_disarm(void);
void robot_arm(void);
bool robot_is_armed(void);

bool robot_ik_tcp(float x, float y, float z, float pitch_deg, float q_target[SERVO_COUNT]);

#endif // ROBOT_IO