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
#include <stdint.h>
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

/** @brief High-level command kind produced by the G-code parser. */
typedef enum {
    GCMD_NONE = 0,   ///< Empty line or modal update without robot action.
    GCMD_MOVE,       ///< G0/G1 motion command.
    GCMD_DWELL,      ///< G4 dwell command.
    GCMD_GRIPPER,    ///< M10/M11/M280 gripper command.
    GCMD_STOP,       ///< M2/M30 stop program request.
    GCMD_STATE,      ///< Modal state update only (G20/G21/G90/G91).
} gcode_cmd_type_t;

/** @brief Parsed command payload passed from parser to executor. */
typedef struct {
    gcode_cmd_type_t type;

    float x;
    float y;
    float z;
    float pitch_deg;
    float duration_s;
    bool is_rapid;

    uint32_t dwell_ms;

    float gripper_deg;
} gcode_cmd_t;

// ===============================
// Public API
// ===============================

/**
 * @brief Parse one G-code text line into an executable command.
 *
 * The parser updates modal state such as units/feed/absolute mode, but it does
 * not send anything to the robot and does not block. Motion pose state is
 * committed only after successful execution of the parsed command.
 */
bool gcode_parse_line(const char *line, gcode_cmd_t *cmd);
/**
 * @brief Execute one previously parsed G-code command.
 *
 * This function performs robot I/O and may block until the command completes.
 */
bool gcode_execute_cmd(const gcode_cmd_t *cmd);
/** @brief Parse and execute one G-code text line. */
bool gcode_push_line(const char *line);
/** @brief Execute a G-code file line-by-line until completion or stop/error. */
bool gcode_run_file(const char *filename);
/** @brief Enable queue-ahead execution for synchronized PREPARE. */
void gcode_set_prefill_mode(bool enabled);
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
