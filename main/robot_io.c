// ===============================
// robotic_arm.c
// ===============================

#include "robot_io.h"

static const char *TAG = "robot_io";

servo_t servos[SERVO_COUNT] = {
    { .gpio_num = GPIO_NUM_35,  .channel = LEDC_CHANNEL_0 },
    { .gpio_num = GPIO_NUM_36,  .channel = LEDC_CHANNEL_1 },
    { .gpio_num = GPIO_NUM_37,  .channel = LEDC_CHANNEL_2 },
    { .gpio_num = GPIO_NUM_39,  .channel = LEDC_CHANNEL_3 },
    { .gpio_num = GPIO_NUM_40, .channel = LEDC_CHANNEL_4 },
    { .gpio_num = GPIO_NUM_41, .channel = LEDC_CHANNEL_5 },
    //{ .gpio_num = GPIO_NUM_42, .channel = LEDC_CHANNEL_6 },

};

sensor_t sensors[SENSOR_COUNT] = {
    { .unit = ADC_UNIT_1, .channel = ADC_CHANNEL_3 },  // IO4
    { .unit = ADC_UNIT_1, .channel = ADC_CHANNEL_4 },  // IO5
    { .unit = ADC_UNIT_1, .channel = ADC_CHANNEL_5 },  // IO6
    { .unit = ADC_UNIT_1, .channel = ADC_CHANNEL_6 },  // IO7
    { .unit = ADC_UNIT_1, .channel = ADC_CHANNEL_7 },  // IO12

    { .unit = ADC_UNIT_2, .channel = ADC_CHANNEL_6 },  // IO17
    //{ .unit = ADC_UNIT_2, .channel = ADC_CHANNEL_7 },  // IO18 (reservation)
};

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
// INVERSE KINEMATICS
// ===============================
void inverse_kinematics(float x, float y, float z, float q_target[SERVO_COUNT])
{
    // TODO: Implement the actual inverse kinematics calculations here
}

// ===============================
// INVERSE KINEMATICS
// ===============================
void move_to_position(float q_target[SERVO_COUNT]) {
    float q_current[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        q_current[i] = sensor_read_angle(i);
    }

    float max_diff = 0;
    for (int i = 0; i < SERVO_COUNT; i++) {
        float diff = fabsf(q_target[i] - q_current[i]);
        if (diff > max_diff) max_diff = diff;
    }

    int steps = INTERP_STEPS;
    if (max_diff > 0) {
        
        for (int s = 0; s <= steps; s++) {
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
void sensors_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);

    if (SENSOR_COUNT == SERVO_COUNT) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            if (sensors[i].unit == ADC_UNIT_1) {
                adc1_config_channel_atten(sensors[i].channel, ADC_ATTEN_DB_12);
            }
            else if (sensors[i].unit == ADC_UNIT_2) {
                adc2_config_channel_atten(sensors[i].channel, ADC_ATTEN_DB_12);
            }
        }
        ESP_LOGI(TAG, "Sensors initialized");
    }
    else
    {
        ESP_LOGE(TAG, "Servo and sensor count mismatch");
    }
}

// ===============================
// SET SERVO ANGLE (0–180°)
// ===============================
void servo_set_angle(int servo_id, float angle) {
    if (servo_id < 0 || servo_id >= SERVO_COUNT) {
        ESP_LOGW(TAG, "Invalid servo ID: %d", servo_id);
        return;
    }

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Angle to duty cycle (14bit = 0–16383)
    uint32_t duty_min = (uint32_t)(0.5f / 20.0f * 16384);  // 0.5 ms 
    uint32_t duty_max = (uint32_t)(2.5f / 20.0f * 16384);  // 2.5 ms
    uint32_t duty = duty_min + ((duty_max - duty_min) * angle) / 180;

    // Set PWM for the channel
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[servo_id].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[servo_id].channel);

    ESP_LOGI(TAG, "Servo %d set to %.1f°", servo_id, angle);
}

// ===============================
// SENSOR READ RAW DATA (0–4095)
// ===============================
int sensor_read_raw(int id) {
    if (id < 0 || id >= SENSOR_COUNT) return -1;

    if (sensors[id].unit == ADC_UNIT_1) {
        return adc1_get_raw(sensors[id].channel);
    }
    else if (sensors[id].unit == ADC_UNIT_2) {
        int raw = 0;
        if (adc2_get_raw(sensors[id].channel, ADC_WIDTH_BIT_12, &raw) == ESP_OK) {
            return raw;
        } else {
            ESP_LOGW(TAG, "ADC2 read failed for sensor %d", id);
            return -1;
        }
    }
    return -1;
}

// ===============================
// SENSOR READ ANGLE (0-180°)
// ===============================
float sensor_read_angle(int id) {
    int raw = sensor_read_raw(id);
    if (raw < 0) return -1;
    return (raw / 4095.0f) * 180.0f;  // lineární mapování 0–180°
}