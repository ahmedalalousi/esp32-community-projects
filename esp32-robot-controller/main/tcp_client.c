/**
 * @file tcp_client.c
 * @brief TCP Sensor Client implementation
 *
 * Connects to the Pi and streams sensor data as newline-delimited JSON.
 * Reconnects automatically if the connection drops.
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"

#include "tcp_client.h"
#include "sensors.h"
#include "motor_driver.h"

static const char *TAG = "TCP_CLT";

/* =========================================================================
 * Configuration
 * ========================================================================= */

#define SENSOR_PORT         CONFIG_TCP_SENSOR_PORT
#define SENSOR_HOST         CONFIG_TCP_SENSOR_HOST
#define SEND_INTERVAL_MS    100     /* 10 Hz — matches sensor poll rate */
#define RECONNECT_DELAY_MS  2000
#define TX_BUFFER_SIZE      512     /* Increased for magnetometer data */

/* =========================================================================
 * State
 * ========================================================================= */

static TaskHandle_t client_task_handle = NULL;
static int sock = -1;
static volatile bool connected = false;

/* =========================================================================
 * Connection Management
 * ========================================================================= */

/**
 * @brief Attempt to connect to the Pi's sensor server
 *
 * @return socket fd on success, -1 on failure
 */
static int connect_to_pi(void)
{
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SENSOR_PORT)
    };

    if (inet_pton(AF_INET, SENSOR_HOST, &dest_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid host address: %s", SENSOR_HOST);
        return -1;
    }

    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return -1;
    }

    /* Set send timeout to avoid blocking indefinitely */
    struct timeval timeout = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (connect(s, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGD(TAG, "connect() to %s:%d failed: errno %d",
                 SENSOR_HOST, SENSOR_PORT, errno);
        close(s);
        return -1;
    }

    ESP_LOGI(TAG, "Connected to Pi at %s:%d", SENSOR_HOST, SENSOR_PORT);
    return s;
}

/* =========================================================================
 * Sensor Client Task
 * ========================================================================= */

static void tcp_client_task(void *arg)
{
    (void)arg;

    char tx_buf[TX_BUFFER_SIZE];
    imu_data_t imu;
    range_data_t range;

    ESP_LOGI(TAG, "Sensor client task started (target: %s:%d)",
             SENSOR_HOST, SENSOR_PORT);

    while (1) {
        /* Connect (or reconnect) */
        if (sock < 0) {
            sock = connect_to_pi();
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
                continue;
            }
            connected = true;
        }

        /* Read and send IMU data */
        if (sensor_get_imu(&imu) == ESP_OK) {
            int len;

            /* Include magnetometer data if available */
            if (imu.mag_valid) {
                len = snprintf(tx_buf, sizeof(tx_buf),
                    "{\"type\":\"imu\","
                    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                    "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
                    "\"mx\":%.2f,\"my\":%.2f,\"mz\":%.2f,"
                    "\"temp\":%.1f}\n",
                    imu.accel_x, imu.accel_y, imu.accel_z,
                    imu.gyro_x, imu.gyro_y, imu.gyro_z,
                    imu.mag_x, imu.mag_y, imu.mag_z,
                    imu.temperature);
            } else {
                /* 6-axis only (MPU-6500/6050) */
                len = snprintf(tx_buf, sizeof(tx_buf),
                    "{\"type\":\"imu\","
                    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                    "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
                    "\"temp\":%.1f}\n",
                    imu.accel_x, imu.accel_y, imu.accel_z,
                    imu.gyro_x, imu.gyro_y, imu.gyro_z,
                    imu.temperature);
            }

            if (send(sock, tx_buf, len, 0) < 0) {
                ESP_LOGE(TAG, "IMU send failed: errno %d", errno);
                close(sock);
                sock = -1;
                connected = false;
                motor_driver_stop();  /* Safety: stop on disconnect */
                continue;
            }
        }

        /* Read and send range data */
        if (sensor_get_range(&range) == ESP_OK) {
            int len = snprintf(tx_buf, sizeof(tx_buf),
                "{\"type\":\"range\",\"distance\":%.3f}\n",
                range.distance_m);

            if (send(sock, tx_buf, len, 0) < 0) {
                ESP_LOGE(TAG, "Range send failed: errno %d", errno);
                close(sock);
                sock = -1;
                connected = false;
                motor_driver_stop();  /* Safety: stop on disconnect */
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t tcp_client_start(void)
{
    if (client_task_handle != NULL) {
        ESP_LOGW(TAG, "Sensor client already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreate(
        tcp_client_task,
        "tcp_sensor_clt",
        4096,
        NULL,
        5,
        &client_task_handle
    );

    return (ret == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void tcp_client_stop(void)
{
    if (sock >= 0) {
        close(sock);
        sock = -1;
    }
    if (client_task_handle != NULL) {
        vTaskDelete(client_task_handle);
        client_task_handle = NULL;
    }
    connected = false;
}

bool tcp_client_connected(void)
{
    return connected;
}
