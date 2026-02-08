/**
 * @file camera_server.c
 * @brief MJPEG Camera Stream Server implementation
 *
 * Uses the ESP-IDF camera driver and HTTP server to stream JPEG
 * frames as multipart/x-mixed-replace (MJPEG).
 *
 * Camera settings (frame size, JPEG quality, frame rate) are
 * configured via Kconfig under "Camera Configuration > Camera Image Settings".
 *
 * Camera pin mapping for Waveshare ESP32-S3-ETH:
 *   Configured via Kconfig under "Camera Configuration > Camera GPIO Pins".
 *   Default pins match the Waveshare board's camera connector.
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "driver/gpio.h"

#include "camera_server.h"

static const char *TAG = "CAM_SVR";

/* =========================================================================
 * Camera Configuration from Kconfig
 * ========================================================================= */

#ifdef CONFIG_CAMERA_ENABLED

/* GPIO Pin Configuration */
#define CAM_PIN_PWDN    CONFIG_CAM_PIN_PWDN
#define CAM_PIN_RESET   CONFIG_CAM_PIN_RESET
#define CAM_PIN_XCLK    CONFIG_CAM_PIN_XCLK
#define CAM_PIN_SIOD    CONFIG_CAM_PIN_SIOD
#define CAM_PIN_SIOC    CONFIG_CAM_PIN_SIOC
#define CAM_PIN_D7      CONFIG_CAM_PIN_D7
#define CAM_PIN_D6      CONFIG_CAM_PIN_D6
#define CAM_PIN_D5      CONFIG_CAM_PIN_D5
#define CAM_PIN_D4      CONFIG_CAM_PIN_D4
#define CAM_PIN_D3      CONFIG_CAM_PIN_D3
#define CAM_PIN_D2      CONFIG_CAM_PIN_D2
#define CAM_PIN_D1      CONFIG_CAM_PIN_D1
#define CAM_PIN_D0      CONFIG_CAM_PIN_D0
#define CAM_PIN_VSYNC   CONFIG_CAM_PIN_VSYNC
#define CAM_PIN_HREF    CONFIG_CAM_PIN_HREF
#define CAM_PIN_PCLK    CONFIG_CAM_PIN_PCLK

/* Image Settings from Kconfig */
#define CAM_JPEG_QUALITY    CONFIG_CAM_JPEG_QUALITY
#define CAM_FRAME_RATE      CONFIG_CAM_FRAME_RATE
#define CAM_FB_COUNT        CONFIG_CAM_FB_COUNT

/* Frame size selection from Kconfig choice */
#if defined(CONFIG_CAM_FRAME_SIZE_QQVGA)
    #define CAM_FRAME_SIZE      FRAMESIZE_QQVGA
    #define CAM_FRAME_SIZE_STR  "QQVGA (160x120)"
#elif defined(CONFIG_CAM_FRAME_SIZE_QVGA)
    #define CAM_FRAME_SIZE      FRAMESIZE_QVGA
    #define CAM_FRAME_SIZE_STR  "QVGA (320x240)"
#elif defined(CONFIG_CAM_FRAME_SIZE_CIF)
    #define CAM_FRAME_SIZE      FRAMESIZE_CIF
    #define CAM_FRAME_SIZE_STR  "CIF (400x296)"
#elif defined(CONFIG_CAM_FRAME_SIZE_HVGA)
    #define CAM_FRAME_SIZE      FRAMESIZE_HVGA
    #define CAM_FRAME_SIZE_STR  "HVGA (480x320)"
#elif defined(CONFIG_CAM_FRAME_SIZE_VGA)
    #define CAM_FRAME_SIZE      FRAMESIZE_VGA
    #define CAM_FRAME_SIZE_STR  "VGA (640x480)"
#elif defined(CONFIG_CAM_FRAME_SIZE_SVGA)
    #define CAM_FRAME_SIZE      FRAMESIZE_SVGA
    #define CAM_FRAME_SIZE_STR  "SVGA (800x600)"
#elif defined(CONFIG_CAM_FRAME_SIZE_XGA)
    #define CAM_FRAME_SIZE      FRAMESIZE_XGA
    #define CAM_FRAME_SIZE_STR  "XGA (1024x768)"
#else
    /* Default fallback */
    #define CAM_FRAME_SIZE      FRAMESIZE_QVGA
    #define CAM_FRAME_SIZE_STR  "QVGA (320x240) [default]"
#endif

#endif /* CONFIG_CAMERA_ENABLED */

#define CAMERA_PORT     CONFIG_TCP_CAMERA_PORT

/* MJPEG boundary string */
#define MJPEG_BOUNDARY  "frame"
#define MJPEG_CONTENT_TYPE  "multipart/x-mixed-replace;boundary=" MJPEG_BOUNDARY

/* =========================================================================
 * State
 * ========================================================================= */

static httpd_handle_t http_server = NULL;
#ifdef CONFIG_CAMERA_ENABLED
static bool camera_initialised = false;
#endif

/* =========================================================================
 * MJPEG Stream Handler
 * ========================================================================= */

#ifdef CONFIG_CAMERA_ENABLED

/**
 * @brief HTTP handler for /stream endpoint
 *
 * Captures frames from the camera and sends them as a continuous
 * MJPEG stream using multipart/x-mixed-replace.
 *
 * Frame rate is controlled by CAM_FRAME_RATE from Kconfig.
 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t err;
    char part_header[64];

    /* Calculate frame delay from configured frame rate */
    const TickType_t frame_delay_ms = 1000 / CAM_FRAME_RATE;

    /* Set response content type */
    err = httpd_resp_set_type(req, MJPEG_CONTENT_TYPE);
    if (err != ESP_OK) {
        return err;
    }

    /* Disable caching */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    ESP_LOGI(TAG, "MJPEG stream started (target %d fps, %lu ms/frame)",
             CAM_FRAME_RATE, (unsigned long)frame_delay_ms);

    while (1) {
        /* Capture a frame */
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGE(TAG, "Camera capture failed");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        /* Build multipart header */
        int hdr_len = snprintf(part_header, sizeof(part_header),
            "\r\n--" MJPEG_BOUNDARY "\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %zu\r\n\r\n",
            fb->len);

        /* Send header */
        err = httpd_resp_send_chunk(req, part_header, hdr_len);
        if (err != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        /* Send JPEG data */
        err = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);

        if (err != ESP_OK) {
            break;
        }

        /* Delay to control frame rate */
        vTaskDelay(pdMS_TO_TICKS(frame_delay_ms));
    }

    ESP_LOGI(TAG, "MJPEG stream ended");
    return ESP_OK;
}

#endif /* CONFIG_CAMERA_ENABLED */

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t camera_init(void)
{
#ifndef CONFIG_CAMERA_ENABLED
    ESP_LOGW(TAG, "Camera disabled in Kconfig");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (camera_initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Camera configuration for Waveshare ESP32-S3-ETH.
     *
     * IMPORTANT: The camera SCCB (I2C) interface uses GPIO 47/48.
     * We let the camera driver own these pins completely.
     *
     * GPIO 8 (PWDN) controls a MOSFET that powers the camera.
     * Testing both polarities to determine correct one.
     *
     * Power-on sequence:
     *   1. Set PWDN to enable power
     *   2. Wait for power to stabilise
     *   3. Let esp_camera_init() handle XCLK and probe
     */
    
    /* 
     * Try GPIO 8 LOW = power ON (typical PWDN is active-high, so LOW = not powered down)
     * If this doesn't work, we'll try HIGH.
     */
    gpio_config_t pwdn_conf = {
        .pin_bit_mask = (1ULL << 8),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwdn_conf);
    
    /* Power cycle sequence */
    gpio_set_level(8, 1);  /* Power OFF first (PWDN high = powered down) */
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(8, 0);  /* Power ON (PWDN low = active) */
    vTaskDelay(pdMS_TO_TICKS(100));  /* Let power stabilise */
    
    ESP_LOGI(TAG, "Camera power enabled (GPIO 8 = LOW, PWDN inactive)");
    
    camera_config_t cam_cfg = {
        .pin_pwdn   = -1,               /* We handle power manually above */
        .pin_reset  = CAM_PIN_RESET,
        .pin_xclk   = CAM_PIN_XCLK,
        /* Let camera driver own SCCB pins */
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7     = CAM_PIN_D7,
        .pin_d6     = CAM_PIN_D6,
        .pin_d5     = CAM_PIN_D5,
        .pin_d4     = CAM_PIN_D4,
        .pin_d3     = CAM_PIN_D3,
        .pin_d2     = CAM_PIN_D2,
        .pin_d1     = CAM_PIN_D1,
        .pin_d0     = CAM_PIN_D0,
        .pin_vsync  = CAM_PIN_VSYNC,
        .pin_href   = CAM_PIN_HREF,
        .pin_pclk   = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,        /* 20 MHz XCLK */
        .ledc_timer   = LEDC_TIMER_1,    /* Timer 1 (Timer 0 used by motors) */
        .ledc_channel = LEDC_CHANNEL_4,  /* Channel 4+ (0-3 used by motors) */
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = CAM_FRAME_SIZE,  /* From Kconfig */
        .jpeg_quality = CAM_JPEG_QUALITY,/* From Kconfig (0-63, lower = better) */
        .fb_count     = CAM_FB_COUNT,    /* From Kconfig */
        .grab_mode    = CAMERA_GRAB_LATEST
    };

    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    camera_initialised = true;
    ESP_LOGI(TAG, "Camera initialised:");
    ESP_LOGI(TAG, "  Resolution:   %s", CAM_FRAME_SIZE_STR);
    ESP_LOGI(TAG, "  JPEG quality: %d (0=best, 63=worst)", CAM_JPEG_QUALITY);
    ESP_LOGI(TAG, "  Frame rate:   %d fps", CAM_FRAME_RATE);
    ESP_LOGI(TAG, "  Buffers:      %d", CAM_FB_COUNT);

    /* Warn if settings are likely to cause overheating */
    if (CAM_FRAME_SIZE >= FRAMESIZE_VGA) {
        ESP_LOGW(TAG, "WARNING: High resolution may cause camera overheating!");
    }
    if (CAM_JPEG_QUALITY < 15) {
        ESP_LOGW(TAG, "WARNING: Low JPEG quality value increases encoding heat!");
    }
    if (CAM_FRAME_RATE > 15) {
        ESP_LOGW(TAG, "WARNING: High frame rate may cause camera overheating!");
    }

    return ESP_OK;
#endif
}

esp_err_t camera_server_start(void)
{
#ifndef CONFIG_CAMERA_ENABLED
    ESP_LOGW(TAG, "Camera disabled — HTTP server not started");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!camera_initialised) {
        ESP_LOGE(TAG, "Camera not initialised — call camera_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (http_server != NULL) {
        ESP_LOGW(TAG, "Camera server already running");
        return ESP_ERR_INVALID_STATE;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CAMERA_PORT;
    config.stack_size = 8192;   /* Larger stack for JPEG handling */

    esp_err_t err = httpd_start(&http_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Register /stream endpoint */
    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(http_server, &stream_uri);

    ESP_LOGI(TAG, "Camera MJPEG server started on port %d (/stream)", CAMERA_PORT);
    return ESP_OK;
#endif
}

void camera_server_stop(void)
{
    if (http_server != NULL) {
        httpd_stop(http_server);
        http_server = NULL;
    }
}
