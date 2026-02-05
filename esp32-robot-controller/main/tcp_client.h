/**
 * @file tcp_client.h
 * @brief TCP Sensor Client — sends sensor data to Pi
 *
 * Connects to the Pi's TCP sensor server on CONFIG_TCP_SENSOR_PORT
 * (default 8081) and streams JSON sensor readings.
 *
 * Protocol:
 *   - Newline-delimited JSON over TCP
 *   - IMU:   {"type":"imu","ax":<f>,"ay":<f>,"az":<f>,"gx":<f>,"gy":<f>,"gz":<f>,"temp":<f>}
 *   - Range: {"type":"range","distance":<f>}
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the TCP sensor client task
 *
 * Creates a FreeRTOS task that:
 *   1. Connects to Pi at CONFIG_TCP_SENSOR_HOST:CONFIG_TCP_SENSOR_PORT
 *   2. Polls sensor data at regular intervals
 *   3. Sends JSON-encoded readings
 *   4. Reconnects if connection is lost
 *
 * @return ESP_OK on success
 */
esp_err_t tcp_client_start(void);

/**
 * @brief Stop the TCP sensor client
 */
void tcp_client_stop(void);

/**
 * @brief Check if connected to the Pi's sensor server
 *
 * @return true if connected
 */
bool tcp_client_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* TCP_CLIENT_H */
