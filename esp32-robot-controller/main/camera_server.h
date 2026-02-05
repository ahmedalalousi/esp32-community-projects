/**
 * @file camera_server.h
 * @brief MJPEG Camera Stream Server
 *
 * Serves the ESP32-S3 camera as an MJPEG stream over HTTP
 * on CONFIG_TCP_CAMERA_PORT (default 8082).
 *
 * Endpoint: GET /stream
 *   Returns: multipart/x-mixed-replace MJPEG stream
 *
 * The ROS2 waveshare_driver node on the Pi fetches this stream
 * and publishes frames to /camera/image_raw.
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#ifndef CAMERA_SERVER_H
#define CAMERA_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the camera hardware
 *
 * Configures the ESP32-S3 camera peripheral with appropriate
 * resolution and JPEG quality settings.
 *
 * @return ESP_OK on success
 */
esp_err_t camera_init(void);

/**
 * @brief Start the MJPEG HTTP server task
 *
 * Creates an HTTP server on CONFIG_TCP_CAMERA_PORT serving
 * MJPEG frames at /stream.
 *
 * @return ESP_OK on success
 */
esp_err_t camera_server_start(void);

/**
 * @brief Stop the camera server
 */
void camera_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SERVER_H */
