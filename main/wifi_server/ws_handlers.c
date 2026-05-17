// ===============================
// ws_handlers.c
// ===============================

#include "ws_handlers.h"

static httpd_handle_t g_httpd = NULL;
static const char *TAG = "wifi_server";

static int g_ws_clients[WS_MAX_CLIENTS];
static SemaphoreHandle_t g_ws_lock = NULL;
static TaskHandle_t g_ws_task_handle = NULL;

static void ws_clients_reset(void) {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        g_ws_clients[i] = -1;
    }
}

static void ws_state_init(void) {
    if (!g_ws_lock) {
        g_ws_lock = xSemaphoreCreateMutex();
    }
    ws_clients_reset();
}

static void ws_clients_add(int fd) {
    if (!g_ws_lock) {
        return;
    }
    xSemaphoreTake(g_ws_lock, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_clients[i] == fd) {
            xSemaphoreGive(g_ws_lock);
            return;
        }
    }
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_clients[i] < 0) {
            g_ws_clients[i] = fd;
            xSemaphoreGive(g_ws_lock);
            return;
        }
    }
    xSemaphoreGive(g_ws_lock);
    ESP_LOGW(TAG, "WS client list full, dropping fd=%d", fd);
}

static void ws_clients_remove(int fd) {
    if (!g_ws_lock) {
        return;
    }
    xSemaphoreTake(g_ws_lock, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_clients[i] == fd) {
            g_ws_clients[i] = -1;
            break;
        }
    }
    xSemaphoreGive(g_ws_lock);
}

static esp_err_t ws_send_to(int fd, const char *msg) {
    if (!g_httpd || fd < 0 || !msg) {
        return ESP_ERR_INVALID_ARG;
    }

    if (httpd_ws_get_fd_info(g_httpd, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
        ws_clients_remove(fd);
        return ESP_ERR_INVALID_STATE;
    }

    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)msg,
        .len = strlen(msg),
    };
    esp_err_t err = httpd_ws_send_frame_async(g_httpd, fd, &frame);
    if (err != ESP_OK) {
        ws_clients_remove(fd);
    }
    return err;
}

static void ws_broadcast(const char *msg) {
    int clients[WS_MAX_CLIENTS];
    int n = 0;

    if (!g_ws_lock) {
        return;
    }

    xSemaphoreTake(g_ws_lock, portMAX_DELAY);
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_clients[i] >= 0) {
            clients[n++] = g_ws_clients[i];
        }
    }
    xSemaphoreGive(g_ws_lock);

    for (int i = 0; i < n; i++) {
        ws_send_to(clients[i], msg);
    }
}

static cJSON *build_telemetry_json(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", robot_get_system_state_name());
    cJSON_AddNumberToObject(root, "can_node_id", (double)CAN_NODE_ID);
    cJSON_AddBoolToObject(root, "armed", robot_is_armed());
    cJSON_AddBoolToObject(root, "referenced", robot_is_referenced());
    cJSON_AddBoolToObject(root, "tcp_est_valid", robot_has_tcp_estimate());

    float wx, wy, wz;
    robot_get_work_offset(&wx, &wy, &wz);
    cJSON *wo = cJSON_AddObjectToObject(root, "work_offset");
    cJSON_AddNumberToObject(wo, "x", wx);
    cJSON_AddNumberToObject(wo, "y", wy);
    cJSON_AddNumberToObject(wo, "z", wz);

    robot_pose_t pose;
    if (robot_get_tcp_estimate_work(&pose)) {
        cJSON *tcp = cJSON_AddObjectToObject(root, "tcp_work");
        cJSON_AddNumberToObject(tcp, "x", pose.x);
        cJSON_AddNumberToObject(tcp, "y", pose.y);
        cJSON_AddNumberToObject(tcp, "z", pose.z);
        cJSON_AddNumberToObject(tcp, "pitch", pose.pitch_deg);
    }

    static const int joint_to_servo[JOINT_COUNT] = {
        JOINT0_SERVO, JOINT1_SERVO, JOINT2_SERVO,
        JOINT3_SERVO, JOINT4_SERVO, JOINT5_SERVO
    };
    cJSON *arr = cJSON_AddArrayToObject(root, "sensors");
    for (int i = 0; i < JOINT_COUNT; i++) {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "id", i);
        cJSON_AddNumberToObject(o, "angle", robot_get_est_angle(joint_to_servo[i]));
        cJSON_AddItemToArray(arr, o);
    }
    return root;
}

static void ws_send_sensors_to_fd(int fd) {
    cJSON *root = build_telemetry_json();
    char *out = cJSON_PrintUnformatted(root);
    ws_send_to(fd, out);
    free(out);
    cJSON_Delete(root);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        ws_clients_add(fd);
        ws_send_to(fd, "{\"status\":\"connected\"}");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        ws_clients_remove(fd);
        return ret;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ws_clients_remove(fd);
        return ESP_OK;
    }

    if (frame.type != HTTPD_WS_TYPE_TEXT || frame.len == 0) {
        return ESP_OK;
    }

    frame.payload = malloc(frame.len + 1);
    if (!frame.payload) {
        return ESP_ERR_NO_MEM;
    }

    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        free(frame.payload);
        ws_clients_remove(fd);
        return ret;
    }
    frame.payload[frame.len] = '\0';

    cJSON *json = cJSON_Parse((char *)frame.payload);
    if (json) {
        cJSON *joint = cJSON_GetObjectItem(json, "joint");
        cJSON *angle = cJSON_GetObjectItem(json, "angle");

        if (cJSON_IsNumber(joint) && cJSON_IsNumber(angle)) {
            int jid = joint->valueint;
            float a = (float)angle->valuedouble;

            if (!robot_is_armed()) {
                ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"joint\",\"reason\":\"disarmed\"}");
            } else if (jid >= 0 && jid < 6) {
                joint_set_angle(jid, a);
                ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"joint\"}");
            } else {
                ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"joint\",\"reason\":\"bad_id\"}");
            }
        }

        cJSON *cmd = cJSON_GetObjectItem(json, "cmd");

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "sensors") == 0) {
            ws_send_sensors_to_fd(fd);
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "move_xyz") == 0) {
            cJSON *jx = cJSON_GetObjectItem(json, "x");
            cJSON *jy = cJSON_GetObjectItem(json, "y");
            cJSON *jz = cJSON_GetObjectItem(json, "z");
            cJSON *jp = cJSON_GetObjectItem(json, "pitch");

            float pitch = ROBOT_DEFAULT_PITCH_DEG;

            if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy) && cJSON_IsNumber(jz)) {
                float x = (float)jx->valuedouble;
                float y = (float)jy->valuedouble;
                float z = (float)jz->valuedouble;
                if (cJSON_IsNumber(jp)) {
                    pitch = (float)jp->valuedouble;
                }

                if (!robot_is_armed()) {
                    ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"move_xyz\",\"reason\":\"disarmed\"}");
                } else {
                    bool ok = robot_cmd_move_xyz_work(x, y, z, pitch);
                    if (ok) {
                        ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"move_xyz\",\"queued\":true}");
                    } else {
                        ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"move_xyz\",\"reason\":\"queue_full_or_not_started\"}");
                    }
                }
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "gcode_line") == 0) {
            cJSON *jl = cJSON_GetObjectItem(json, "line");
            if (cJSON_IsString(jl)) {
                bool ok = gcode_push_line(jl->valuestring);
                ws_send_to(fd, ok ? "{\"status\":\"ok\",\"cmd\":\"gcode_line\"}"
                                  : "{\"status\":\"err\",\"cmd\":\"gcode_line\"}");
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "run_gcode") == 0) {
            cJSON *jf = cJSON_GetObjectItem(json, "filename");
            if (cJSON_IsString(jf)) {
                bool ok = robot_core_run_gcode(jf->valuestring);
                ws_send_to(fd, ok ? "{\"status\":\"ok\",\"cmd\":\"run_gcode\",\"state\":\"started\"}"
                                  : "{\"status\":\"error\",\"cmd\":\"run_gcode\",\"reason\":\"start_rejected\"}");
            } else {
                ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"run_gcode\",\"msg\":\"no_filename\"}");
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "disarm") == 0) {
            gcode_stop();
            robot_disarm();
            ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"disarm\",\"state\":\"DISARMED\"}");
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "arm") == 0) {
            robot_arm();
            ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"arm\",\"state\":\"READY\"}");
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "set_work_offset") == 0) {
            cJSON *jx = cJSON_GetObjectItem(json, "x");
            cJSON *jy = cJSON_GetObjectItem(json, "y");
            cJSON *jz = cJSON_GetObjectItem(json, "z");

            if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy) && cJSON_IsNumber(jz)) {
                robot_set_work_offset((float)jx->valuedouble, (float)jy->valuedouble, (float)jz->valuedouble);
                ws_send_to(fd, "{\"status\":\"ok\",\"cmd\":\"set_work_offset\"}");
            } else {
                ws_send_to(fd, "{\"status\":\"error\",\"cmd\":\"set_work_offset\",\"reason\":\"bad_args\"}");
            }
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "home") == 0) {
            bool ok = robot_cmd_home_reference();
            ws_send_to(fd, ok ? "{\"status\":\"ok\",\"cmd\":\"home\",\"state\":\"QUEUED\"}"
                              : "{\"status\":\"error\",\"cmd\":\"home\",\"reason\":\"queue_failed\"}");
        }

        if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "gcode_stop") == 0) {
            gcode_stop();
            char resp[128];
            snprintf(resp, sizeof(resp),
                     "{\"status\":\"ok\",\"cmd\":\"gcode_stop\",\"state\":\"%s\"}",
                     robot_get_system_state_name());
            ws_send_to(fd, resp);
        }

        cJSON_Delete(json);
    }

    free(frame.payload);
    return ESP_OK;
}

static void ws_task_sensors(void *arg) {
    (void)arg;
    for (;;) {
        cJSON *root = build_telemetry_json();
        char *out = cJSON_PrintUnformatted(root);
        ws_broadcast(out);
        free(out);
        cJSON_Delete(root);
        vTaskDelay(pdMS_TO_TICKS(WS_SENSORS_PERIOD_MS));
    }
}

esp_err_t ws_handlers_register(httpd_handle_t server) {
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }

    g_httpd = server;
    ws_state_init();

    httpd_uri_t ws_uri = {
        "/ws",
        HTTP_GET,
        ws_handler,
        NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true,
    };

    return httpd_register_uri_handler(server, &ws_uri);
}

void ws_handlers_start_task(void) {
    ws_state_init();
    if (g_ws_task_handle) {
        return;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        ws_task_sensors,
        "ws_sensors",
        4096,
        NULL,
        5,
        &g_ws_task_handle,
        CORE_COMM);
    if (ok != pdPASS) {
        g_ws_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to start ws_sensors task");
    }
}
