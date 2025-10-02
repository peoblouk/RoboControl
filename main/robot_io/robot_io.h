// ===============================
// robot_io.c
// ===============================

#ifndef ROBOT_IO
#define ROBOT_IO

#include "esp_log.h"
#include "math.h"
#include "driver/ledc.h"
#include "driver/adc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_COUNT 6
#define SENSOR_COUNT 6

#define SERVO_MIN_US 500   // 0° ~ 0.5 ms
#define SERVO_MAX_US 2500  // 180° ~ 2.5 ms
#define SERVO_PWM_FREQ 50  // 50 Hz = 20 ms period

#define INTERP_STEPS 50
#define INTERP_DELAY_MS 20

#define L0  50.0f   // vertical offset from base to first joint
#define L1  30.0f   // offset v radiální rovině (např. základní "reach" před prvním kloubem)
#define L2  70.0f   // délka prvního článku (rameno)
#define L3  60.0f   // délka druhého článku (loket)
#define L4  20.0f   // délka zápěstí / posledního článku


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
// FUNCTION PROTOTYPES
// ===============================
void servos_init(void);
void sensors_init(void);
int sensor_read_raw(int id);
float sensor_read_angle(int id);
void servo_set_angle(int servo_id, float angle);
void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT]);
void move_to_position(float q_target[SERVO_COUNT]);

#endif // ROBOT_IO