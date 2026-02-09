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
#define SERVO_COUNT          7
#define SENSOR_COUNT         6

// ===============================
// ROBOT: SERVO PWM
// ===============================
#define SERVO_MIN_US   500    // 0°  ~ 0.5 ms
#define SERVO_MAX_US   2500   // 180° ~ 2.5 ms
#define SERVO_PWM_FREQ 50     // 50 Hz = 20 ms period

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
#define L0  82.0f
#define L1  112.5f
#define L2  73.0f
#define L3  0.0f
#define L4  0.0f
#define L5  0.0f

// ===============================
// ROBOT: JOINT LIMITS (deg / deg/s)
// ===============================
#define J0_MIN 23
#define J0_MAX 180
#define J0_V   60

#define J1_MIN 27
#define J1_MAX 173
#define J1_V   60

#define J2_MIN 15
#define J2_MAX 175
#define J2_V   60

#define J3_MIN 5
#define J3_MAX 170 
#define J3_V   60

#define J4_MIN 8
#define J4_MAX 175
#define J4_V   90

#define J5_MIN 43
#define J5_MAX 170
#define J5_V   90

#define J6_MIN 20
#define J6_MAX 80
#define J6_V   90

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
// HOME position
#define HOME_J0 40
#define HOME_J1 40   // J1 master+follower
#define HOME_J2 30
#define HOME_J3 30
#define HOME_J4 30
#define HOME_J5 20


#endif // CONFIG_H