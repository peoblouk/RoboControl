// ===============================
// file_manager.h
// ===============================

#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

// ===============================
// Dependencies
// ===============================

#include "esp_http_server.h"

#include "config.h"
#include "esp_log.h"
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

// ===============================
// Public API
// ===============================

/** @brief Register file-management HTTP routes (`/files`, `/file/<name>`). */
esp_err_t file_manager_register(httpd_handle_t server);

#endif // FILE_MANAGER_H
