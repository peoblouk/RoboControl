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
#include "gcode.h"

// Radians / Degrees conversion
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)


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
// JOINT LIMITS (per-axis safety)
// ===============================
typedef struct {
    float min_deg;
    float max_deg;
    float max_deg_s;
} joint_limits_t;

extern const joint_limits_t g_joint_limits[SERVO_COUNT];

// ===============================
// TRAJECTORY SEGMENT STRUCTURE
// ===============================
typedef struct {
    float q0[SERVO_COUNT];
    float q1[SERVO_COUNT];
    float T;
    float t;
    bool  active;
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
bool  robot_xyz_reachable(float x, float y, float z);
float robot_min_time_for_move(const float q0[SERVO_COUNT], const float q1[SERVO_COUNT]);

void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT]);
//void move_to_position(float q_target[SERVO_COUNT]);

float robot_get_est_angle(int id);


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
    float duration_s;
} robot_cmd_t;

// ===============================
// CONTROL API
// ===============================
void robot_control_start(void);
bool robot_cmd_move_joints(const float q_target[SERVO_COUNT]);
bool robot_cmd_move_xyz(float x, float y, float z);
bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout);
void robot_cmd_queue_flush(void);
void robot_core_run_gcode(const char *filename);

void robot_disarm(void);
void robot_arm(void);
bool robot_is_armed(void);

#endif // ROBOT_IO