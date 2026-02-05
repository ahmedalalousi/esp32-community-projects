/**
 * @file sensors.h
 * @brief Sensor interface for URM09 ultrasonic and MPU-6050 IMU
 *
 * Both sensors share an I2C bus configured via Kconfig:
 *   - CONFIG_SENSOR_I2C_SDA / CONFIG_SENSOR_I2C_SCL (GPIO pins)
 *   - CONFIG_IMU_I2C_ADDR / CONFIG_URM09_I2C_ADDR (device addresses)
 *
 * The sensor task runs in a FreeRTOS task, polling both devices
 * and making data available via thread-safe accessors.
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Data Structures
 * ========================================================================= */

/**
 * @brief IMU reading (accelerometer + gyroscope)
 *
 * Raw values converted to physical units:
 *   - Accelerometer: m/s² (±2g default range)
 *   - Gyroscope: rad/s (±250°/s default range)
 */
typedef struct {
    float accel_x;      /**< Acceleration X (m/s²) */
    float accel_y;      /**< Acceleration Y (m/s²) */
    float accel_z;      /**< Acceleration Z (m/s²) */
    float gyro_x;       /**< Angular velocity X (rad/s) */
    float gyro_y;       /**< Angular velocity Y (rad/s) */
    float gyro_z;       /**< Angular velocity Z — yaw rate (rad/s) */
    float temperature;  /**< Die temperature (°C) */
    bool valid;         /**< True if reading is fresh and valid */
} imu_data_t;

/**
 * @brief Ultrasonic range reading
 */
typedef struct {
    float distance_m;   /**< Distance in metres */
    bool valid;         /**< True if reading is within sensor range */
} range_data_t;

/**
 * @brief Sensor configuration (from Kconfig)
 */
typedef struct {
    int i2c_sda_gpio;       /**< I2C SDA pin */
    int i2c_scl_gpio;       /**< I2C SCL pin */
    uint8_t imu_addr;       /**< IMU I2C address (7-bit) */
    uint8_t urm09_addr;     /**< URM09 I2C address (7-bit) */
    uint32_t poll_interval_ms;  /**< Sensor polling interval */
} sensor_config_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Get default sensor configuration from Kconfig
 *
 * @return Configuration struct populated from menuconfig settings
 */
sensor_config_t sensor_default_config(void);

/**
 * @brief Initialise I2C bus and sensor devices
 *
 * Configures I2C master, probes for IMU and URM09, and initialises
 * device registers. Does NOT start the polling task.
 *
 * @param cfg Pointer to sensor configuration
 * @return ESP_OK on success
 */
esp_err_t sensor_init(const sensor_config_t *cfg);

/**
 * @brief Start the sensor polling task
 *
 * Creates a FreeRTOS task that periodically reads both sensors
 * and updates the internal data structures.
 *
 * @return ESP_OK on success
 */
esp_err_t sensor_task_start(void);

/**
 * @brief Get latest IMU reading
 *
 * Thread-safe. Copies the most recent IMU data into the provided struct.
 *
 * @param data Pointer to destination struct
 * @return ESP_OK if valid data available, ESP_ERR_INVALID_STATE if no reading yet
 */
esp_err_t sensor_get_imu(imu_data_t *data);

/**
 * @brief Get latest ultrasonic range reading
 *
 * Thread-safe. Copies the most recent range data into the provided struct.
 *
 * @param data Pointer to destination struct
 * @return ESP_OK if valid data available, ESP_ERR_INVALID_STATE if no reading yet
 */
esp_err_t sensor_get_range(range_data_t *data);

/**
 * @brief Check if IMU is detected and responding
 *
 * @return true if IMU is present on the I2C bus
 */
bool sensor_imu_present(void);

/**
 * @brief Check if URM09 is detected and responding
 *
 * @return true if URM09 is present on the I2C bus
 */
bool sensor_urm09_present(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
