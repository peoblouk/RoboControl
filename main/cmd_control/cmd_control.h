// ===============================
// cmd_control.h
// ===============================

#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H
#include "config.h"   // Configuration

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_console.h"
#include <dirent.h>
#include <sys/stat.h>
#include "freertos/queue.h"
#include "linenoise/linenoise.h"

#include "robot_io.h"
#include "rt_stats.h"
#include "gcode.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void cmd_control_start(void);

#endif // CMD_CONTROL_H
