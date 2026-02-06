/**
 * @file network.h
 * @brief Dual-interface network management (WiFi + Ethernet)
 * 
 * Provides seamless WiFi and Ethernet connectivity with automatic failover.
 * Hardware: ESP32-S3 + W5500 Ethernet module
 * 
 * @author Ahmed Al-Alousi
 * @date December 2025
 */

#ifndef NETWORK_H
#define NETWORK_H

#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Network interface types
 */
typedef enum {
    NETWORK_IF_WIFI,
    NETWORK_IF_ETHERNET
} network_interface_t;

/**
 * @brief Initialise dual-interface network system
 * 
 * Initialises both WiFi and Ethernet interfaces.
 * WiFi credentials must be set via menuconfig.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t network_init(void);

/**
 * @brief Start network connections
 * 
 * Attempts to connect both WiFi and Ethernet (non-blocking).
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t network_start(void);

/**
 * @brief Check if specific interface is connected
 * 
 * @param interface Interface to check
 * @return true if connected, false otherwise
 */
bool network_is_connected(network_interface_t interface);

/**
 * @brief Get IP address of specific interface
 * 
 * @param interface Interface to query
 * @return IP address string, or NULL if not connected
 */
const char* network_get_ip(network_interface_t interface);

/**
 * @brief Get currently active interface
 * 
 * @return Active interface (prefers Ethernet over WiFi)
 */
network_interface_t network_get_active_interface(void);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_H
