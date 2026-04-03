// ===============================
// status_led.h
// ===============================

#ifndef STATUS_LED_H
#define STATUS_LED_H

// ===============================
// Dependencies
// ===============================

#include "config.h"
#include "robot_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_strip.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdint.h>

// ===============================
// Public Types
// ===============================

/** @brief High-level status LED mode derived from robot state. */
typedef enum {
    ROBOT_LED_DISARM = 0, ///< Robot is disarmed.
    ROBOT_LED_ARMED,      ///< Robot is armed and idle/ready.
    ROBOT_LED_OPERATING,  ///< Robot is currently operating (blink mode).
} robot_led_state_t;

// ===============================
// Public API
// ===============================

/** @brief Initialize status LED driver and start status LED task. */
void status_led_start(void);

#endif // STATUS_LED_H
