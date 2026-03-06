// ===============================
// gcode.h
// ===============================

#ifndef GCODE_H
#define GCODE_H
#include "config.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "robot_io.h"
#include "esp_log.h"

typedef struct {
    bool absolute;     // G90/G91
    bool units_mm;     // G21/G20
    float feed_mm_s;   // from F (mm/min -> mm/s)
    float x, y, z;     // current position in mm
} gcode_state_t;

// Functions prototypes
bool gcode_push_line(const char *line);    // push single G-code line
bool gcode_run_file(const char *filename); // run G-code from file
void gcode_stop(void);                     // stop G-code execution
void gcode_reset(void);                    // reset G-code state

#endif // GCODE_H