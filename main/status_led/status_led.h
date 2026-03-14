#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "config.h"
#include "robot_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_strip.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdint.h>

typedef enum {
    ROBOT_LED_DISARM = 0,
    ROBOT_LED_ARMED,
    ROBOT_LED_OPERATING,
} robot_led_state_t;

void status_led_start(void);

#endif // STATUS_LED_H
