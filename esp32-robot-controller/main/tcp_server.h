/**
 * @file tcp_server.h
 * @brief TCP Command Server interface
 */

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the TCP command server
 */
esp_err_t tcp_server_start(void);

/**
 * @brief Stop the TCP command server
 */
void tcp_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TCP_SERVER_H */
