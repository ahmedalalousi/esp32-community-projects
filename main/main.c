/**
 * @file main.c
 * @brief ESP32-S3 Dual Interface Network Example
 * 
 * Demonstrates WiFi + W5500 Ethernet working together with automatic failover.
 * 
 * Hardware: ESP32-S3 + W5500 Ethernet Module
 * Author: Ahmed Al-Alousi
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "network.h"

static const char *TAG = "MAIN";

/**
 * @brief Status monitoring task
 * 
 * Periodically displays network connection status
 */
static void status_task(void *pvParameters)
{
    while (1) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Network Status:");
        
        // Check WiFi
        if (network_is_connected(NETWORK_IF_WIFI)) {
            ESP_LOGI(TAG, "  WiFi:     Connected (%s)", 
                     network_get_ip(NETWORK_IF_WIFI));
        } else {
            ESP_LOGI(TAG, "  WiFi:     Disconnected");
        }
        
        // Check Ethernet
        if (network_is_connected(NETWORK_IF_ETHERNET)) {
            ESP_LOGI(TAG, "  Ethernet: Connected (%s)", 
                     network_get_ip(NETWORK_IF_ETHERNET));
        } else {
            ESP_LOGI(TAG, "  Ethernet: Disconnected");
        }
        
        // Show active interface
        network_interface_t active = network_get_active_interface();
        ESP_LOGI(TAG, "  Active:   %s", 
                 active == NETWORK_IF_ETHERNET ? "Ethernet" : "WiFi");
        
        ESP_LOGI(TAG, "========================================");
        
        // Check every 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-S3 Dual Interface Network Example");
    ESP_LOGI(TAG, " WiFi + W5500 Ethernet");
    ESP_LOGI(TAG, " Author: Ahmed Al-Alousi December 2025");
    ESP_LOGI(TAG, " Please accredit authorship");
    ESP_LOGI(TAG, "========================================");
    
    // Initialise NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || 
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialise network subsystem
    ESP_ERROR_CHECK(network_init());
    
    // Start network connections
    ESP_ERROR_CHECK(network_start());
    
    ESP_LOGI(TAG, "✓ System initialised");
    ESP_LOGI(TAG, "Connecting to networks (this may take a few seconds)...");
    
    // Create status monitoring task
    xTaskCreate(status_task, "status", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✓ Status task started");
    ESP_LOGI(TAG, "Watch for connection events and status updates...");
}
