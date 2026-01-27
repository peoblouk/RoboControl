// ===============================
// robot_io.c
// ===============================

#ifndef ROBOT_IO
#define ROBOT_IO
#include "config.h"   // Configuration

#include "esp_log.h"
#include "math.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h" // #include "driver/adc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include "gcode.h" // G-code interpreter


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

extern servo_t servos[SERVO_COUNT];

// ===============================
//  SENSOR CONFIGURATION (ADC input)
// ===============================
typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;
} sensor_t;

extern sensor_t sensors[SENSOR_COUNT];

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
int sensor_read_raw(int id);
float sensor_read_angle(int id);
void servo_set_angle(int servo_id, float angle);
void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT]);
void move_to_position(float q_target[SERVO_COUNT]);


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
    float q_target[SERVO_COUNT];  // target joint angles
    float x, y, z;                // target XYZ (MOVE_XYZ)
    float duration_s;             // duration for movement (MOVE_JOINTS_T)
} robot_cmd_t;

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void robot_control_start(void);
bool robot_cmd_move_joints(const float q_target[SERVO_COUNT]);
bool robot_cmd_move_xyz(float x, float y, float z);

bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout);
void robot_cmd_queue_flush(void);

void robot_core_run_gcode(const char *filename);

#endif // ROBOT_IO