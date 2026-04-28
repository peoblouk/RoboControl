// ===============================
// cmd_control.h
// ===============================

#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H

// ===============================
// Dependencies
// ===============================

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
#include "gcode.h"
#include "can_communication.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===============================
// Public API
// ===============================

/** @brief Start CLI/console command control task (REPL command registration). */
void cmd_control_start(void);

#endif // CMD_CONTROL_H
