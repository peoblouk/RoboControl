// ===============================
// cmd_control.h
// ===============================

#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_console.h"
#include <dirent.h>
#include <sys/stat.h>

#include "robot_io.h"
#include "core_config.h"
#include "rt_stats.h"

#define CMD_BUF_SIZE 128   // max command line length
#define FILE_STORAGE_PATH "/spiffs/data" // File storage path

#define STATS_PRINT // Enable printing of stats

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void cmd_control_start(void);

#endif // CMD_CONTROL_H
