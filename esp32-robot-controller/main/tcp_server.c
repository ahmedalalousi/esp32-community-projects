/**
 * @file tcp_server.c
 * @brief TCP Command Server implementation
 *
 * Listens for motor commands from the ROS2 waveshare_driver node.
 * Protocol: JSON lines (newline-delimited JSON)
 *
 * Commands:
 *   {"cmd":"motor","linear":0.5,"angular":0.1}
 *   {"cmd":"stop"}
 *   {"cmd":"ping"}
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "cJSON.h"

#include "tcp_server.h"
#include "motor_driver.h"

static const char *TAG = "TCP_SVR";

#define CMD_PORT            CONFIG_TCP_CMD_PORT
#define RX_BUFFER_SIZE      256
#define MAX_CLIENTS         1       /* Only one client at a time */

static TaskHandle_t server_task_handle = NULL;
static int listen_sock = -1;

/**
 * @brief Parse and execute a motor command
 */
static void handle_command(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        ESP_LOGW(TAG, "Invalid JSON: %s", json_str);
        return;
    }

    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cmd == NULL || !cJSON_IsString(cmd)) {
        ESP_LOGW(TAG, "Missing 'cmd' field");
        cJSON_Delete(root);
        return;
    }

    const char *cmd_str = cmd->valuestring;

    if (strcmp(cmd_str, "motor") == 0) {
        /* Motor velocity command */
        cJSON *linear = cJSON_GetObjectItem(root, "linear");
        cJSON *angular = cJSON_GetObjectItem(root, "angular");

        float lin = 0.0f, ang = 0.0f;
        if (linear && cJSON_IsNumber(linear)) {
            lin = (float)linear->valuedouble;
        }
        if (angular && cJSON_IsNumber(angular)) {
            ang = (float)angular->valuedouble;
        }

        motor_driver_set_velocity(lin, ang);
        ESP_LOGD(TAG, "Motor cmd: linear=%.2f angular=%.2f", lin, ang);

    } else if (strcmp(cmd_str, "stop") == 0) {
        /* Emergency stop */
        motor_driver_stop();
        ESP_LOGI(TAG, "Stop command received");

    } else if (strcmp(cmd_str, "ping") == 0) {
        /* Keepalive ping — just acknowledge */
        ESP_LOGD(TAG, "Ping received");

    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_str);
    }

    cJSON_Delete(root);
}

/**
 * @brief Handle a connected client
 */
static void handle_client(int client_sock)
{
    char rx_buf[RX_BUFFER_SIZE];
    char line_buf[RX_BUFFER_SIZE];
    int line_pos = 0;

    ESP_LOGI(TAG, "Client connected");

    while (1) {
        int len = recv(client_sock, rx_buf, sizeof(rx_buf) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "recv() failed: errno %d", errno);
            break;
        }
        if (len == 0) {
            ESP_LOGI(TAG, "Client disconnected");
            break;
        }

        /* Process received bytes, looking for newlines */
        for (int i = 0; i < len; i++) {
            char c = rx_buf[i];
            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    handle_command(line_buf);
                    line_pos = 0;
                }
            } else {
                if (line_pos < (int)sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = c;
                }
            }
        }
    }

    /* Safety: stop motors when client disconnects */
    motor_driver_stop();
}

/**
 * @brief TCP server task
 */
static void tcp_server_task(void *arg)
{
    (void)arg;

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(CMD_PORT)
    };

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "listen() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP command server listening on port %d", CMD_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "accept() failed: errno %d", errno);
            continue;
        }

        handle_client(client_sock);
        close(client_sock);
    }
}

esp_err_t tcp_server_start(void)
{
    if (server_task_handle != NULL) {
        ESP_LOGW(TAG, "TCP server already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreate(
        tcp_server_task,
        "tcp_cmd_svr",
        4096,
        NULL,
        5,
        &server_task_handle
    );

    return (ret == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void tcp_server_stop(void)
{
    if (listen_sock >= 0) {
        close(listen_sock);
        listen_sock = -1;
    }
    if (server_task_handle != NULL) {
        vTaskDelete(server_task_handle);
        server_task_handle = NULL;
    }
}
