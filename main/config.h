// ===============================
// config.h
// ===============================

#ifndef CONFIG_H
#define CONFIG_H

// ===============================
// CORE CONFIGURATION
// ===============================
#define CORE_COMM   0   // WiFi, CAN, webserver
#define CORE_ROBOT  1   // servos, sensors, control

// Filepaths
#define FS_DATA_BASE        "/spiffs/data"
#define FS_WEB_BASE         "/spiffs/web"
#define FILE_STORAGE_PATH   FS_DATA_BASE

// ===============================
// ROBOT: COUNTS
// ===============================
#define SERVO_COUNT   7
#define SENSOR_COUNT  6
#define JOINT_COUNT   6

// ===============================
// ROBOT: SERVO / JOINT IDs
// ===============================
// Servo IDs (indexy do servos[] / s_servo_pwm[] / OFF[] / DIR[])
#define SERVO_J0      0
#define SERVO_J1_A    1
#define SERVO_J1_B    2   // follower
#define SERVO_J2      3
#define SERVO_J3      4
#define SERVO_J4      5
#define SERVO_J5      6   // gripper

// Kompatibilita se starými názvy
#define J1_A_SERVO    SERVO_J1_A
#define J1_B_SERVO    SERVO_J1_B

// Joint -> master servo mapping (indexy jointů jsou 0..JOINT_COUNT-1)
#define JOINT0_SERVO  SERVO_J0
#define JOINT1_SERVO  SERVO_J1_A
#define JOINT2_SERVO  SERVO_J2
#define JOINT3_SERVO  SERVO_J3
#define JOINT4_SERVO  SERVO_J4
#define JOINT5_SERVO  SERVO_J5

// ===============================
// ROBOT: SERVO PWM
// ===============================
#define SERVO_PWM_FREQ 50     // 50 Hz = 20 ms period

// PWM range (µs) — 0° => MIN, 180° => MAX
#define SERVO0_MIN_US  700
#define SERVO0_MAX_US  2300

#define SERVO1_MIN_US  850
#define SERVO1_MAX_US  2300

#define SERVO2_MIN_US  850
#define SERVO2_MAX_US  2300

#define SERVO3_MIN_US  750
#define SERVO3_MAX_US  2350

#define SERVO4_MIN_US  650
#define SERVO4_MAX_US  2400

#define SERVO5_MIN_US  1000
#define SERVO5_MAX_US  2300

#define SERVO6_MIN_US  720
#define SERVO6_MAX_US  1700

#define SERVO_PWM_RANGES_INIT { \
    { SERVO0_MIN_US, SERVO0_MAX_US }, \
    { SERVO1_MIN_US, SERVO1_MAX_US }, \
    { SERVO2_MIN_US, SERVO2_MAX_US }, \
    { SERVO3_MIN_US, SERVO3_MAX_US }, \
    { SERVO4_MIN_US, SERVO4_MAX_US }, \
    { SERVO5_MIN_US, SERVO5_MAX_US }, \
    { SERVO6_MIN_US, SERVO6_MAX_US }, \
}

// ===============================
// ROBOT: SERVO MAPPING (OFF / DIR)
// ===============================
// joint_deg_math -> servo_deg = OFF + DIR * joint_deg_math
#define SERVO0_OFF_DEG  70.0f
#define SERVO1_OFF_DEG  180.0f
#define SERVO2_OFF_DEG  180.0f
#define SERVO3_OFF_DEG  150.0f
#define SERVO4_OFF_DEG  70.0f
#define SERVO5_OFF_DEG  60.0f
#define SERVO6_OFF_DEG  90.0f

#define SERVO0_DIR      +1.0f
#define SERVO1_DIR      +1.0f
#define SERVO2_DIR      -1.0f
#define SERVO3_DIR      +1.0f
#define SERVO4_DIR      -1.0f
#define SERVO5_DIR      +1.0f
#define SERVO6_DIR      +1.0f

#define SERVO_OFF_INIT { \
    SERVO0_OFF_DEG, SERVO1_OFF_DEG, SERVO2_OFF_DEG, \
    SERVO3_OFF_DEG, SERVO4_OFF_DEG, SERVO5_OFF_DEG, SERVO6_OFF_DEG \
}

#define SERVO_DIR_INIT { \
    SERVO0_DIR, SERVO1_DIR, SERVO2_DIR, \
    SERVO3_DIR, SERVO4_DIR, SERVO5_DIR, SERVO6_DIR \
}

// ===============================
// ROBOT: J1 FOLLOWER
// ===============================
#define J1_B_TRIM_DEG        -0.5f

// ===============================
// ROBOT: EXECUTOR / TIMING
// ===============================
#define EXEC_DT_MS    20
#define EXEC_DT_S     (EXEC_DT_MS / 1000.0f)
#define MIN_SEG_T     0.05f

// ===============================
// ROBOT: PLANNER / BUFFERS
// ===============================
#define SEG_BUF_LEN          32
#define ROBOT_CMD_QUEUE_LEN  32

// ===============================
// ROBOT: LEGACY INTERPOLATION
// ===============================
#define INTERP_STEPS     50
#define INTERP_DELAY_MS  20

// ===============================
// ROBOT: ARM GEOMETRY (mm)
// ===============================
#define L0  83.0f
#define L1  112.5f
#define L2  73.5f
#define L_TOOL  115.0f

// ===============================
// ROBOT: JOINT LIMITS (deg / deg/s)
// ===============================
#define J0_MIN 0
#define J0_MAX 180
#define J0_V   60

#define J1_MIN 0
#define J1_MAX 180
#define J1_V   60

#define J2_MIN 0
#define J2_MAX 180
#define J2_V   60

#define J3_MIN 0
#define J3_MAX 180
#define J3_V   60

#define J4_MIN 0
#define J4_MAX 180
#define J4_V   90

#define J5_MIN 0
#define J5_MAX 90
#define J5_V   90

// ===============================
// ROBOT: SERVO MAPPING (GPIO + LEDC)
// ===============================
#define SERVO0_GPIO GPIO_NUM_35   // J0
#define SERVO1_GPIO GPIO_NUM_36   // J1_A
#define SERVO2_GPIO GPIO_NUM_37   // J1_B (follower)
#define SERVO3_GPIO GPIO_NUM_39   // J2
#define SERVO4_GPIO GPIO_NUM_40   // J3
#define SERVO5_GPIO GPIO_NUM_41   // J4
#define SERVO6_GPIO GPIO_NUM_42   // J5 (gripper)

#define SERVO0_CH LEDC_CHANNEL_0
#define SERVO1_CH LEDC_CHANNEL_1
#define SERVO2_CH LEDC_CHANNEL_2
#define SERVO3_CH LEDC_CHANNEL_3
#define SERVO4_CH LEDC_CHANNEL_4
#define SERVO5_CH LEDC_CHANNEL_5
#define SERVO6_CH LEDC_CHANNEL_6

// ===============================
// ROBOT: SENSOR MAPPING (ADC UNIT + CHANNEL)
// ===============================
#define S0_ADC_UNIT ADC_UNIT_1
#define S0_ADC_CH   ADC_CHANNEL_3

#define S1_ADC_UNIT ADC_UNIT_1
#define S1_ADC_CH   ADC_CHANNEL_4

#define S2_ADC_UNIT ADC_UNIT_1
#define S2_ADC_CH   ADC_CHANNEL_5

#define S3_ADC_UNIT ADC_UNIT_1
#define S3_ADC_CH   ADC_CHANNEL_6

#define S4_ADC_UNIT ADC_UNIT_1
#define S4_ADC_CH   ADC_CHANNEL_7

#define S5_ADC_UNIT ADC_UNIT_2
#define S5_ADC_CH   ADC_CHANNEL_6

// ===============================
// CMD CONTROL CONFIGURATION
// ===============================
#define CMD_BUF_SIZE 128
#define STATS_PRINT

// ===============================
// GCODE CONFIGURATION
// ===============================
#define RAPID_MM_S  200.0f
#define MIN_V_MM_S  1.0f

// ===============================
// SERVER CONFIGURATION
// ===============================
#define WIFI_SSID      "RoboControl"
#define WIFI_PASS      "Robo-Control123"
#define MAX_STA_CONN   4
#define WS_MAX_CLIENTS 4
#define HTTP_FILE_PREFIX   "/file/"

// ===============================
// Robot Positions
// ===============================
#define HOME_J0 75
#define HOME_J1 75
#define HOME_J2 30
#define HOME_J3 120
#define HOME_J4 90
#define HOME_J5 90
#define HOME_J6 90

#endif // CONFIG_H