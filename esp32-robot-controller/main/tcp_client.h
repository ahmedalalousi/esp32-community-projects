/**
 * @file tcp_client.h
 * @brief TCP Sensor Client interface
 */

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the TCP sensor client
 */
esp_err_t tcp_client_start(void);

/**
 * @brief Stop the TCP sensor client
 */
void tcp_client_stop(void);

/**
 * @brief Check if client is connected
 */
bool tcp_client_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* TCP_CLIENT_H */
