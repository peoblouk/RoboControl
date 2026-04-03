// ===============================
// robot_io.h
// ===============================

#ifndef ROBOT_IO
#define ROBOT_IO

// ===============================
// Dependencies
// ===============================

#include "config.h"

#include "esp_log.h"
#include "math.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <stdint.h>

// ===============================
// Math Helpers
// ===============================

/** @brief Convert radians to degrees. */
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
/** @brief Convert degrees to radians. */
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

// ===============================
// Default Pose / Offset Values
// ===============================

#ifndef ROBOT_WORK_OFFSET_X_DEFAULT
#define ROBOT_WORK_OFFSET_X_DEFAULT 220.0f
#endif
#ifndef ROBOT_WORK_OFFSET_Y_DEFAULT
#define ROBOT_WORK_OFFSET_Y_DEFAULT 0.0f
#endif
#ifndef ROBOT_WORK_OFFSET_Z_DEFAULT
#define ROBOT_WORK_OFFSET_Z_DEFAULT 25.0f
#endif
#ifndef ROBOT_HOME_X_BASE_DEFAULT
#define ROBOT_HOME_X_BASE_DEFAULT ROBOT_WORK_OFFSET_X_DEFAULT
#endif
#ifndef ROBOT_HOME_Y_BASE_DEFAULT
#define ROBOT_HOME_Y_BASE_DEFAULT ROBOT_WORK_OFFSET_Y_DEFAULT
#endif
#ifndef ROBOT_HOME_Z_BASE_DEFAULT
#define ROBOT_HOME_Z_BASE_DEFAULT 80.0f
#endif
#ifndef ROBOT_HOME_PITCH_DEG_DEFAULT
#define ROBOT_HOME_PITCH_DEG_DEFAULT 0.0f
#endif

// ===============================
// Public Types
// ===============================

/** @brief Servo output mapping (LEDC channel + GPIO pin). */
typedef struct {
    ledc_channel_t channel;
    gpio_num_t gpio_num;
} servo_t;

/** @brief Servo PWM pulse range [us] for 0..180 deg mapping. */
typedef struct {
    uint16_t min_us;
    uint16_t max_us;
} servo_pwm_range_t;

/** @brief Global servo mapping table (`SERVO_COUNT` entries). */
extern servo_t servos[SERVO_COUNT];

/** @brief Sensor ADC mapping (unit + channel). */
typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;
} sensor_t;

#if SENSOR_COUNT > 0
/** @brief Global sensor mapping table (`SENSOR_COUNT` entries). */
extern sensor_t sensors[SENSOR_COUNT];
#endif

/** @brief Motion and angle limits for one servo/joint axis. */
typedef struct {
    float min_deg;
    float max_deg;
    float max_deg_s;
} joint_limits_t;

/** @brief Joint limits table indexed by servo ID. */
extern const joint_limits_t g_joint_limits[SERVO_COUNT];

/** @brief TCP pose in XYZ + pitch (degrees). */
typedef struct {
    float x;
    float y;
    float z;
    float pitch_deg;
} robot_pose_t;

/** @brief One interpolated joint-space trajectory segment. */
typedef struct {
    float q0[SERVO_COUNT];          ///< Segment start joint angles [deg].
    float q1[SERVO_COUNT];          ///< Segment end joint angles [deg].
    float T;                        ///< Segment total duration [s].
    float t;                        ///< Current elapsed time [s].
    bool active;                    ///< Segment is currently active.

    bool tcp_target_valid;          ///< `tcp_target_base` is valid.
    bool preserve_tcp_estimate;     ///< Keep current TCP estimate unchanged.
    bool mark_referenced_on_finish; ///< Mark robot referenced when finished.
    robot_pose_t tcp_target_base;   ///< Expected TCP pose in base frame.
} traj_seg_t;

/** @brief High-level robot state summary. */
typedef enum {
    ROBOT_STATE_DISARMED = 0,      ///< Robot is disarmed.
    ROBOT_STATE_UNREFERENCED = 1,  ///< Robot not referenced/home'd yet.
    ROBOT_STATE_READY = 2,         ///< Ready for command execution.
    ROBOT_STATE_RUNNING = 3,       ///< Executing motion or program.
    ROBOT_STATE_READY_FOR_SYNC = 4, ///< Prepared for synchronized start.
    ROBOT_STATE_ERROR = 5,         ///< Error latched.
} robot_system_state_t;

/** @brief Last latched high-level error source. */
typedef enum {
    ROBOT_ERROR_NONE = 0,    ///< No active error.
    ROBOT_ERROR_HOME = 1,    ///< Homing/reference error.
    ROBOT_ERROR_GCODE = 2,   ///< G-code execution error.
    ROBOT_ERROR_SYNC = 3,    ///< Synchronization/control error.
    ROBOT_ERROR_STORAGE = 4, ///< File/storage error.
} robot_error_t;

// ===============================
// Command Queue Types
// ===============================

/** @brief Type of queued control command. */
typedef enum {
    ROBOT_CMD_NONE = 0,      ///< Empty/no-op command.
    ROBOT_CMD_MOVE_JOINTS,   ///< Joint-space move with default timing.
    ROBOT_CMD_MOVE_XYZ,      ///< Cartesian move with default timing.
    ROBOT_CMD_MOVE_JOINTS_T, ///< Joint-space move with explicit duration.
    ROBOT_CMD_DWELL,         ///< Timed pause.
    ROBOT_CMD_GRIPPER,       ///< Gripper setpoint command.
    ROBOT_CMD_QUEUE_FLUSH    ///< Flush command queue.
} robot_cmd_type_t;

/** @brief One robot command queue item. */
typedef struct {
    robot_cmd_type_t type;       ///< Command discriminator.
    float q_target[SERVO_COUNT]; ///< Joint-space target [deg].
    float x, y, z;               ///< Cartesian target XYZ.
    float pitch_deg;             ///< Cartesian target pitch [deg].
    float duration_s;            ///< Requested segment duration [s].
    uint32_t dwell_ms;           ///< Dwell duration [ms].
    float gripper_deg;           ///< Gripper target angle [deg].

    bool mark_referenced_on_finish; ///< Set referenced flag after completion.
    bool tcp_target_valid;          ///< Cartesian target fields are valid.
} robot_cmd_t;

// ===============================
// Public API
// ===============================

// --- Servo/Sensor I/O ---
/** @brief Initialize servo PWM outputs and mapping. */
void servos_init(void);
/** @brief Initialize ADC channels for configured sensors. */
void sensors_init(void);

/** @brief Read raw ADC value of sensor `id`. */
int   sensor_read_raw(int id);
/** @brief Read mapped sensor angle in degrees. */
float sensor_read_angle(int id);
/** @brief Return `true` when sensor subsystem is initialized. */
bool  robot_sensors_initialized(void);

/** @brief Set physical servo angle by servo index. */
void servo_set_angle(int servo_id, float angle);
/** @brief Set logical joint angle (handles master/follower mapping). */
void joint_set_angle(int joint_id, float angle);

// --- Kinematics / Validation ---
/** @brief Validate joint targets and optionally clamp to limits. */
bool  robot_validate_and_prepare_q(float q[SERVO_COUNT], bool clamp);
/** @brief Check reachability of base-frame TCP target. */
bool  robot_tcp_reachable(float x, float y, float z, float pitch_deg);
/** @brief Check reachability of work-frame TCP target. */
bool  robot_tcp_reachable_work(float x, float y, float z, float pitch_deg);
/** @brief Estimate minimum duration to move between two joint vectors. */
float robot_min_time_for_move(const float q0[SERVO_COUNT], const float q1[SERVO_COUNT]);

/** @brief Get last estimated joint angle for servo `id`. */
float robot_get_est_angle(int id);

// --- Frame Offsets / Pose Estimate ---
/** @brief Set work-frame XYZ offset relative to base frame. */
void robot_set_work_offset(float x, float y, float z);
/** @brief Read work-frame XYZ offset relative to base frame. */
void robot_get_work_offset(float *x, float *y, float *z);

/** @brief Return `true` if robot has valid reference/home state. */
bool robot_is_referenced(void);
/** @brief Return `true` if TCP estimate is currently valid. */
bool robot_has_tcp_estimate(void);
/** @brief Clear reference and invalidate estimated TCP pose. */
void robot_clear_reference(void);

/** @brief Get current estimated TCP pose in base frame. */
bool robot_get_tcp_estimate_base(robot_pose_t *pose);
/** @brief Get current estimated TCP pose in work frame. */
bool robot_get_tcp_estimate_work(robot_pose_t *pose);

// --- Control / Command API ---
/** @brief Start robot control tasks and command queue processing. */
void robot_control_start(void);
/** @brief Queue joint-space move with automatic timing. */
bool robot_cmd_move_joints(const float q_target[SERVO_COUNT]);
/** @brief Queue homing joint move and set home TCP reference on finish. */
bool robot_cmd_move_joints_home(const float q_target[SERVO_COUNT],
                                float home_x_base,
                                float home_y_base,
                                float home_z_base,
                                float home_pitch_deg);
/** @brief Queue base-frame Cartesian move with automatic timing. */
bool robot_cmd_move_xyz(float x, float y, float z, float pitch_deg);
/** @brief Queue base-frame Cartesian move with explicit duration. */
bool robot_cmd_move_xyz_t(float x, float y, float z, float pitch_deg, float duration_s, TickType_t timeout);
/** @brief Queue work-frame Cartesian move with automatic timing. */
bool robot_cmd_move_xyz_work(float x, float y, float z, float pitch_deg);
/** @brief Queue work-frame Cartesian move with explicit duration. */
bool robot_cmd_move_xyz_work_t(float x, float y, float z, float pitch_deg, float duration_s, TickType_t timeout);
/** @brief Queue joint-space move with explicit duration. */
bool robot_cmd_move_joints_t(const float q_target[SERVO_COUNT], float duration_s, TickType_t timeout);
/** @brief Queue dwell/pause command in milliseconds. */
bool robot_cmd_dwell_ms(uint32_t dwell_ms, TickType_t timeout);
/** @brief Queue gripper angle command. */
bool robot_cmd_gripper_set(float gripper_deg, TickType_t timeout);
/** @brief Execute homing/reference routine. */
bool robot_cmd_home_reference(void);
/** @brief Flush pending robot command queue. */
void robot_cmd_queue_flush(void);
/** @brief Start asynchronous G-code run from file. */
bool robot_core_run_gcode(const char *filename);
/** @brief Stop current execution and clear pending motion. */
void robot_stop_all(void);

// --- State / Safety ---
/** @brief Disarm robot outputs. */
void robot_disarm(void);
/** @brief Arm robot outputs. */
void robot_arm(void);
/** @brief Return `true` if robot outputs are armed. */
bool robot_is_armed(void);
/** @brief Return `true` while robot is actively executing. */
bool robot_is_operating(void);
/** @brief Return `true` while a G-code program is running. */
bool robot_is_program_running(void);
/** @brief Set synchronized-start readiness flag. */
void robot_set_sync_ready(bool ready);
/** @brief Return synchronized-start readiness flag. */
bool robot_is_sync_ready(void);
/** @brief Compute and return current high-level system state. */
robot_system_state_t robot_get_system_state(void);
/** @brief Return textual name of current system state. */
const char *robot_get_system_state_name(void);
/** @brief Return last latched high-level error code. */
robot_error_t robot_get_last_error(void);
/** @brief Clear latched high-level error code. */
void robot_clear_error(void);

/** @brief Solve IK for base-frame TCP target into `q_target`. */
bool robot_ik_tcp(float x, float y, float z, float pitch_deg, float q_target[SERVO_COUNT]);

#endif // ROBOT_IO
