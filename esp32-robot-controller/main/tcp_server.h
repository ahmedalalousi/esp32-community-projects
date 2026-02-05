/**
 * @file tcp_server.h
 * @brief TCP Command Server — receives motor commands from Pi
 *
 * Listens on CONFIG_TCP_CMD_PORT (default 8080) for JSON commands
 * from the ROS2 waveshare_driver node running on the Raspberry Pi.
 *
 * Protocol:
 *   - Newline-delimited JSON over TCP
 *   - Each message is a single JSON object terminated by '\n'
 *   - Command format: {"cmd":"motor","linear":0.5,"angular":0.1}
 *   - Response format: {"status":"ok"} or {"status":"error","msg":"..."}
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the TCP command server task
 *
 * Creates a FreeRTOS task that:
 *   1. Binds to CONFIG_TCP_CMD_PORT
 *   2. Accepts one client at a time (Pi)
 *   3. Reads newline-delimited JSON commands
 *   4. Dispatches motor commands via motor_driver API
 *   5. Reconnects if client disconnects
 *
 * @return ESP_OK on success
 */
esp_err_t tcp_server_start(void);

/**
 * @brief Stop the TCP command server
 *
 * Closes socket and deletes the task.
 */
void tcp_server_stop(void);

/**
 * @brief Check if a client (Pi) is currently connected
 *
 * @return true if a client is connected
 */
bool tcp_server_client_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* TCP_SERVER_H */
