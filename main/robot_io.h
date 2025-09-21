// ===============================
// robotic_arm.c
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

// ===============================
// Servo Configuration (PWM output)
// ===============================
typedef struct {
    ledc_channel_t channel;
    gpio_num_t gpio_num;
} servo_t;

extern servo_t servos[SERVO_COUNT];

// ===============================
// Sensor Configuration (ADC input)
// ===============================
typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;
} sensor_t;

extern sensor_t sensors[SENSOR_COUNT];
// ===============================
// Function Prototypes
// ===============================
void servos_init(void);
void sensors_init(void);
int sensor_read_raw(int id);
float sensor_read_angle(int id);
void servo_set_angle(int servo_id, float angle);
void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT]);

#endif // ROBOT_IO