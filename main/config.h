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
#define FS_DATA_BASE        "/spiffs/data" // Path for data files
#define FS_WEB_BASE         "/spiffs/web" // Path for web files
#define FILE_STORAGE_PATH   FS_DATA_BASE  // Path for file storage

// ===============================
// ROBOT IO CONFIGURATION  
// ===============================
#define SERVO_COUNT          6
#define SENSOR_COUNT         6

#define SERVO_MIN_US 500   // 0° ~ 0.5 ms
#define SERVO_MAX_US 2500  // 180° ~ 2.5 ms
#define SERVO_PWM_FREQ 50  // 50 Hz = 20 ms period

#define EXEC_DT_MS 20                 // 50 Hz update
#define EXEC_DT_S  (EXEC_DT_MS / 1000.0f)
#define SEG_BUF_LEN 32
#define MIN_SEG_T   0.05f             // minimum segment time in seconds

#define INTERP_STEPS 50
#define INTERP_DELAY_MS 20

#define L0  50.0f   // vertical offset from base to first joint
#define L1  30.0f   // offset in z-axis from base to first joint
#define L2  70.0f   // length of first link (arm)
#define L3  60.0f   // length of second link (elbow)
#define L4  20.0f   // length of wrist / last link
#define L5  10.0f   // length of end-effector

// Executor (real-time)
#define EXEC_DT_MS           20        // 50 Hz (serva většinou v pohodě)
#define MIN_SEG_T_S          0.05f     // minimální čas segmentu, aby to necukalo

// Planner / queue
#define SEG_BUF_LEN          32        // kolik segmentů dopředu (ring buffer)
#define ROBOT_CMD_QUEUE_LEN  32        // FreeRTOS queue length pro robot_cmd_t


// ===============================
// CMD CONTROL CONFIGURATION
// ===============================
#define CMD_BUF_SIZE 128   // max command line length
#define STATS_PRINT // Enable printing of stats

// ===============================
// GCODE CONFIGURATION
// ===============================
#define RAPID_MM_S 200.0f
#define MIN_V_MM_S 1.0f

// ===============================
// SERVER CONFIGURATION
// ===============================
#define WIFI_SSID      "RoboControl" // WiFi SSID
#define WIFI_PASS      "Robo-Control123" // WiFi Password
#define MAX_STA_CONN   4 // max stable connections
#define WS_MAX_CLIENTS 4 // max websocket clients

#define HTTP_FILE_PREFIX   "/file/"
#endif // CONFIG_H