/**
 * @file tcp_server.c
 * @brief TCP Command Server implementation
 *
 * Single-client TCP server that receives JSON motor commands from
 * the ROS2 waveshare_driver node on the Raspberry Pi.
 *
 * Supported commands:
 *   {"cmd":"motor","linear":<float>,"angular":<float>}
 *   {"cmd":"stop"}
 *   {"cmd":"estop"}
 *   {"cmd":"mode","value":<int>}   (0=two-wheel, 1=four-wheel, 2=tracked)
 *   {"cmd":"pid","kp":<f>,"ki":<f>,"kd":<f>}
 *   {"cmd":"set_motor","id":<int>,"speed":<int>}
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "cJSON.h"

#include "tcp_server.h"
#include "motor_driver.h"

static const char *TAG = "TCP_SVR";

/* =========================================================================
 * Configuration
 * ========================================================================= */

#define CMD_PORT            CONFIG_TCP_CMD_PORT
#define RX_BUFFER_SIZE      512
#define KEEPALIVE_IDLE      5       /* seconds before first keepalive probe */
#define KEEPALIVE_INTERVAL  5       /* seconds between probes */
#define KEEPALIVE_COUNT     3       /* failed probes before disconnect */

/* =========================================================================
 * State
 * ========================================================================= */

static TaskHandle_t server_task_handle = NULL;
static int listen_sock = -1;
static int client_sock = -1;
static volatile bool client_connected = false;

/* =========================================================================
 * Command Dispatch
 * ========================================================================= */

/**
 * @brief Process a parsed JSON command
 *
 * @param json Parsed cJSON object
 * @return Response string (caller must free via cJSON_Delete on parent)
 */
static void process_command(cJSON *json, int sock)
{
    cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
    if (!cJSON_IsString(cmd)) {
        const char *err = "{\"status\":\"error\",\"msg\":\"missing cmd\"}\n";
        send(sock, err, strlen(err), 0);
        return;
    }

    const char *cmd_str = cmd->valuestring;

    if (strcmp(cmd_str, "motor") == 0) {
        /* Velocity command */
        cJSON *lin = cJSON_GetObjectItem(json, "linear");
        cJSON *ang = cJSON_GetObjectItem(json, "angular");
        if (cJSON_IsNumber(lin) && cJSON_IsNumber(ang)) {
            motor_driver_set_velocity((float)lin->valuedouble,
                                     (float)ang->valuedouble);
            const char *ok = "{\"status\":\"ok\"}\n";
            send(sock, ok, strlen(ok), 0);
        } else {
            const char *err = "{\"status\":\"error\",\"msg\":\"missing linear/angular\"}\n";
            send(sock, err, strlen(err), 0);
        }

    } else if (strcmp(cmd_str, "stop") == 0) {
        motor_driver_stop();
        const char *ok = "{\"status\":\"ok\"}\n";
        send(sock, ok, strlen(ok), 0);

    } else if (strcmp(cmd_str, "estop") == 0) {
        motor_driver_emergency_stop();
        const char *ok = "{\"status\":\"ok\"}\n";
        send(sock, ok, strlen(ok), 0);

    } else if (strcmp(cmd_str, "mode") == 0) {
        cJSON *val = cJSON_GetObjectItem(json, "value");
        if (cJSON_IsNumber(val)) {
            motor_driver_set_drive_mode((drive_mode_t)val->valueint);
            const char *ok = "{\"status\":\"ok\"}\n";
            send(sock, ok, strlen(ok), 0);
        } else {
            const char *err = "{\"status\":\"error\",\"msg\":\"missing value\"}\n";
            send(sock, err, strlen(err), 0);
        }

    } else if (strcmp(cmd_str, "pid") == 0) {
        cJSON *kp = cJSON_GetObjectItem(json, "kp");
        cJSON *ki = cJSON_GetObjectItem(json, "ki");
        cJSON *kd = cJSON_GetObjectItem(json, "kd");
        if (cJSON_IsNumber(kp) && cJSON_IsNumber(ki) && cJSON_IsNumber(kd)) {
            pid_params_t params = {
                .kp = (float)kp->valuedouble,
                .ki = (float)ki->valuedouble,
                .kd = (float)kd->valuedouble,
                .integral_limit = 50.0f,    /* Sensible defaults */
                .output_limit = 30.0f
            };
            /* Check for optional limits */
            cJSON *ilim = cJSON_GetObjectItem(json, "integral_limit");
            cJSON *olim = cJSON_GetObjectItem(json, "output_limit");
            if (cJSON_IsNumber(ilim)) params.integral_limit = (float)ilim->valuedouble;
            if (cJSON_IsNumber(olim)) params.output_limit = (float)olim->valuedouble;

            motor_driver_set_pid_params(&params);
            const char *ok = "{\"status\":\"ok\"}\n";
            send(sock, ok, strlen(ok), 0);
        } else {
            const char *err = "{\"status\":\"error\",\"msg\":\"missing kp/ki/kd\"}\n";
            send(sock, err, strlen(err), 0);
        }

    } else if (strcmp(cmd_str, "set_motor") == 0) {
        cJSON *id = cJSON_GetObjectItem(json, "id");
        cJSON *spd = cJSON_GetObjectItem(json, "speed");
        if (cJSON_IsNumber(id) && cJSON_IsNumber(spd)) {
            motor_driver_set_motor((motor_id_t)id->valueint,
                                  (int8_t)spd->valueint);
            const char *ok = "{\"status\":\"ok\"}\n";
            send(sock, ok, strlen(ok), 0);
        } else {
            const char *err = "{\"status\":\"error\",\"msg\":\"missing id/speed\"}\n";
            send(sock, err, strlen(err), 0);
        }

    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_str);
        const char *err = "{\"status\":\"error\",\"msg\":\"unknown command\"}\n";
        send(sock, err, strlen(err), 0);
    }
}

/**
 * @brief Handle a connected client — read and process commands
 */
static void handle_client(int sock)
{
    char rx_buf[RX_BUFFER_SIZE];
    char line_buf[RX_BUFFER_SIZE];
    int line_pos = 0;

    ESP_LOGI(TAG, "Client connected on port %d", CMD_PORT);
    client_connected = true;

    while (1) {
        int len = recv(sock, rx_buf, sizeof(rx_buf) - 1, 0);
        if (len <= 0) {
            if (len == 0) {
                ESP_LOGI(TAG, "Client disconnected");
            } else {
                ESP_LOGE(TAG, "recv error: errno %d", errno);
            }
            break;
        }

        /* Process received bytes, extracting newline-delimited JSON */
        for (int i = 0; i < len; i++) {
            if (rx_buf[i] == '\n') {
                /* End of line — parse JSON */
                line_buf[line_pos] = '\0';

                if (line_pos > 0) {
                    cJSON *json = cJSON_Parse(line_buf);
                    if (json != NULL) {
                        process_command(json, sock);
                        cJSON_Delete(json);
                    } else {
                        ESP_LOGW(TAG, "JSON parse error: %s", line_buf);
                        const char *err = "{\"status\":\"error\",\"msg\":\"invalid JSON\"}\n";
                        send(sock, err, strlen(err), 0);
                    }
                }

                line_pos = 0;
            } else {
                /* Accumulate character */
                if (line_pos < (int)(sizeof(line_buf) - 1)) {
                    line_buf[line_pos++] = rx_buf[i];
                } else {
                    /* Line too long — discard */
                    ESP_LOGW(TAG, "Line buffer overflow, discarding");
                    line_pos = 0;
                }
            }
        }
    }

    /* Client gone — stop motors for safety */
    motor_driver_stop();
    client_connected = false;
}

/* =========================================================================
 * Server Task
 * ========================================================================= */

static void tcp_server_task(void *arg)
{
    (void)arg;

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    while (1) {
        /* Create socket */
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "socket() failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Allow address reuse */
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        /* Bind */
        if (bind(listen_sock, (struct sockaddr *)&server_addr,
                 sizeof(server_addr)) != 0) {
            ESP_LOGE(TAG, "bind() failed: errno %d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Listen */
        if (listen(listen_sock, 1) != 0) {
            ESP_LOGE(TAG, "listen() failed: errno %d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "TCP command server listening on port %d", CMD_PORT);

        while (1) {
            /* Accept client */
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            client_sock = accept(listen_sock,
                                 (struct sockaddr *)&client_addr, &addr_len);
            if (client_sock < 0) {
                ESP_LOGE(TAG, "accept() failed: errno %d", errno);
                break;
            }

            /* Enable TCP keepalive */
            int keepalive = 1;
            setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE,
                       &keepalive, sizeof(keepalive));
            int idle = KEEPALIVE_IDLE;
            setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPIDLE,
                       &idle, sizeof(idle));
            int interval = KEEPALIVE_INTERVAL;
            setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPINTVL,
                       &interval, sizeof(interval));
            int count = KEEPALIVE_COUNT;
            setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPCNT,
                       &count, sizeof(count));

            /* Handle this client (blocks until disconnect) */
            handle_client(client_sock);

            close(client_sock);
            client_sock = -1;
        }

        close(listen_sock);
        listen_sock = -1;
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t tcp_server_start(void)
{
    if (server_task_handle != NULL) {
        ESP_LOGW(TAG, "TCP server already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreate(
        tcp_server_task,
        "tcp_cmd_srv",
        4096,
        NULL,
        5,
        &server_task_handle
    );

    return (ret == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void tcp_server_stop(void)
{
    if (client_sock >= 0) {
        close(client_sock);
        client_sock = -1;
    }
    if (listen_sock >= 0) {
        close(listen_sock);
        listen_sock = -1;
    }
    if (server_task_handle != NULL) {
        vTaskDelete(server_task_handle);
        server_task_handle = NULL;
    }
    client_connected = false;
    motor_driver_stop();
}

bool tcp_server_client_connected(void)
{
    return client_connected;
}
