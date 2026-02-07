/**
 * @file main.c
 * @brief Robot Controller — application entry point
 *
 * Initialisation sequence:
 *   1. Network (WiFi + W5500 Ethernet, from existing dual-interface code)
 *   2. Motor driver (PWM + direction GPIOs)
 *   3. Sensors (I2C bus, IMU, ultrasonic)
 *   4. TCP command server (receives motor commands from Pi)
 *   5. TCP sensor client (sends sensor data to Pi)
 *   6. Camera + MJPEG server (optional, if enabled in Kconfig)
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "network.h"
#include "motor_driver.h"
#include "sensors.h"
#include "tcp_server.h"
#include "tcp_client.h"
#include "camera_server.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Robot Controller Starting ===");

    /* ---------------------------------------------------------------
     * Step 0: System prerequisites
     * --------------------------------------------------------------- */

    /* Initialise NVS — required for WiFi credential storage */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return;
    }

    /* ---------------------------------------------------------------
     * Step 1: Network
     *
     * Uses the existing dual-interface code from
     * esp32-community-projects. Initialises WiFi + W5500 Ethernet
     * with automatic failover.
     * --------------------------------------------------------------- */

    ESP_LOGI(TAG, "Initialising network...");
    err = network_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Network init failed: %s", esp_err_to_name(err));
        return;
    }

    err = network_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Network start failed: %s", esp_err_to_name(err));
        return;
    }

    /* Wait for network connection before starting TCP services */
    ESP_LOGI(TAG, "Waiting for network connection...");
    int retries = 0;
    while (!network_is_connected(NETWORK_IF_ETHERNET) &&
           !network_is_connected(NETWORK_IF_WIFI) &&
           retries < 30) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        retries++;
    }

    if (!network_is_connected(NETWORK_IF_ETHERNET) &&
        !network_is_connected(NETWORK_IF_WIFI)) {
        ESP_LOGE(TAG, "Network connection timeout — continuing anyway");
    } else {
        const char *ip = network_get_ip(
            network_is_connected(NETWORK_IF_ETHERNET)
            ? NETWORK_IF_ETHERNET
            : NETWORK_IF_WIFI);
        ESP_LOGI(TAG, "Connected, IP: %s", ip ? ip : "unknown");
    }

    /* ---------------------------------------------------------------
     * Step 2: Motor driver
     * --------------------------------------------------------------- */

    ESP_LOGI(TAG, "Initialising motor driver...");
    motor_driver_config_t motor_cfg = motor_driver_default_config();
    err = motor_driver_init(&motor_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor driver init failed: %s", esp_err_to_name(err));
        /* Non-fatal — continue to allow sensor-only operation */
    }

    /* ---------------------------------------------------------------
     * Step 3: Camera (MUST be before sensors to create I2C bus)
     *
     * The camera's SCCB interface creates an I2C bus on GPIO 47/48.
     * External sensors (IMU, URM09) will share this bus.
     * --------------------------------------------------------------- */

#ifdef CONFIG_CAMERA_ENABLED
    ESP_LOGI(TAG, "Initialising camera...");
    err = camera_init();
    if (err == ESP_OK) {
        err = camera_server_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera server start failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
    }
#else
    ESP_LOGI(TAG, "Camera disabled in configuration");
#endif

    /* ---------------------------------------------------------------
     * Step 4: Sensors (I2C bus shared with camera SCCB)
     *
     * When camera is enabled, sensors reuse the I2C bus created by
     * the camera's SCCB driver. When camera is disabled, sensors
     * create their own I2C bus.
     * --------------------------------------------------------------- */

    ESP_LOGI(TAG, "Initialising sensors...");
    sensor_config_t sensor_cfg = sensor_default_config();
    err = sensor_init(&sensor_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(err));
        /* Non-fatal */
    } else {
        /* Start sensor polling task */
        err = sensor_task_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Sensor task start failed: %s", esp_err_to_name(err));
        }
    }

    /* ---------------------------------------------------------------
     * Step 5: TCP command server (Pi → ESP32 motor commands)
     * --------------------------------------------------------------- */

    ESP_LOGI(TAG, "Starting TCP command server...");
    err = tcp_server_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCP server start failed: %s", esp_err_to_name(err));
    }

    /* ---------------------------------------------------------------
     * Step 6: TCP sensor client (ESP32 → Pi sensor data)
     * --------------------------------------------------------------- */

    ESP_LOGI(TAG, "Starting TCP sensor client...");
    err = tcp_client_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCP client start failed: %s", esp_err_to_name(err));
    }

    /* ---------------------------------------------------------------
     * Done — all tasks running
     * --------------------------------------------------------------- */

    ESP_LOGI(TAG, "=== Robot Controller Ready ===");
    ESP_LOGI(TAG, "  Motor driver:   %s", motor_cfg.pid_enabled ? "PID enabled" : "open-loop");
    ESP_LOGI(TAG, "  Drive mode:     %s",
             motor_cfg.drive_mode == DRIVE_MODE_TWO_WHEEL ? "two-wheel" :
             motor_cfg.drive_mode == DRIVE_MODE_FOUR_WHEEL ? "four-wheel" : "tracked");
    ESP_LOGI(TAG, "  IMU:            %s (%s)",
             sensor_imu_present() ? "detected" : "not found",
             sensor_imu_type_str());
    ESP_LOGI(TAG, "  Magnetometer:   %s", sensor_mag_present() ? "detected" : "not available");
    ESP_LOGI(TAG, "  Ultrasonic:     %s", sensor_urm09_present() ? "detected" : "not found");
    ESP_LOGI(TAG, "  TCP cmd server: port %d", CONFIG_TCP_CMD_PORT);
    ESP_LOGI(TAG, "  TCP sensor:     -> %s:%d", CONFIG_TCP_SENSOR_HOST, CONFIG_TCP_SENSOR_PORT);
#ifdef CONFIG_CAMERA_ENABLED
    ESP_LOGI(TAG, "  Camera stream:  port %d/stream", CONFIG_TCP_CAMERA_PORT);
#endif

    /* app_main returns — FreeRTOS scheduler continues running tasks */
}
