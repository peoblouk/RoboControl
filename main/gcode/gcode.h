// ===============================
// gcode.h
// ===============================

#ifndef GCODE_H
#define GCODE_H

// ===============================
// Dependencies
// ===============================

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

// ===============================
// Public Types
// ===============================

/** @brief Runtime parser/interpreter state for G-code execution. */
typedef struct {
    bool absolute;   ///< `true` = G90 (absolute), `false` = G91 (incremental).
    bool units_mm;   ///< `true` = G21 (mm), `false` = G20 (inch).
    bool pose_known; ///< Internal flag: parser has valid current TCP pose.
    float feed_mm_s; ///< Active feed speed in mm/s (converted from `F` word).
    float x, y, z;   ///< Current TCP position in work frame [mm].
    float pitch_deg; ///< Current tool pitch estimate [deg].
} gcode_state_t;

// ===============================
// Public API
// ===============================

/** @brief Parse and execute one G-code text line. */
bool gcode_push_line(const char *line);
/** @brief Execute a G-code file line-by-line until completion or stop/error. */
bool gcode_run_file(const char *filename);
/** @brief Request graceful stop of current G-code execution. */
void gcode_stop(void);
/** @brief Force external failure state and stop execution. */
void gcode_fail_external(const char *msg);
/** @brief Reset parser state and try to sync from current robot TCP pose. */
void gcode_reset(void);
/** @brief Set parser current TCP position explicitly (work frame). */
void gcode_set_current_position(float x, float y, float z, float pitch_deg);
/** @brief Sync parser TCP position from robot estimate. Returns `false` if unavailable. */
bool gcode_sync_to_robot_pose(void);

#endif // GCODE_H