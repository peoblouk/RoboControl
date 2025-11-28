#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_console.h"

#include "robot_io.h"
#include "core_config.h"

#define CMD_BUF_SIZE 128   // max command line length

// ===============================
// FUNCTION PROTOTYPES
// ===============================
void cmd_control_start(void);

#endif // CMD_CONTROL_H
