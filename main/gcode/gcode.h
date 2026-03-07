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
    float x, y, z;     // current position in work frame [mm]
    float pitch_deg;   // current tool pitch estimate [deg]
} gcode_state_t;

bool gcode_push_line(const char *line);
bool gcode_run_file(const char *filename);
void gcode_stop(void);
void gcode_reset(void);
void gcode_set_current_position(float x, float y, float z, float pitch_deg);
bool gcode_sync_to_robot_pose(void);

#endif // GCODE_H