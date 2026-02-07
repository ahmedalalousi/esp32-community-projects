/**
 * @file sensors.h
 * @brief Sensor interface for URM09 ultrasonic and MPU-9250 IMU
 *
 * Supports:
 *   - MPU-9250: 9-axis IMU (accel + gyro + magnetometer)
 *   - MPU-6500: 6-axis IMU (accel + gyro, no magnetometer)
 *   - MPU-6050: 6-axis IMU (legacy fallback)
 *   - URM09: I2C ultrasonic range sensor
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
 * @brief IMU reading (accelerometer + gyroscope + magnetometer)
 *
 * Raw values converted to physical units:
 *   - Accelerometer: m/s² (configurable range: ±2g, ±4g, ±8g, ±16g)
 *   - Gyroscope: rad/s (configurable range: ±250, ±500, ±1000, ±2000 °/s)
 *   - Magnetometer: µT (micro-Tesla) — MPU-9250 only
 */
typedef struct {
    /* Accelerometer (all IMU types) */
    float accel_x;      /**< Acceleration X (m/s²) */
    float accel_y;      /**< Acceleration Y (m/s²) */
    float accel_z;      /**< Acceleration Z (m/s²) */

    /* Gyroscope (all IMU types) */
    float gyro_x;       /**< Angular velocity X (rad/s) */
    float gyro_y;       /**< Angular velocity Y (rad/s) */
    float gyro_z;       /**< Angular velocity Z — yaw rate (rad/s) */

    /* Magnetometer (MPU-9250 only) */
    float mag_x;        /**< Magnetic field X (µT) */
    float mag_y;        /**< Magnetic field Y (µT) */
    float mag_z;        /**< Magnetic field Z (µT) */
    bool mag_valid;     /**< True if magnetometer reading is valid */

    /* Temperature and validity */
    float temperature;  /**< Die temperature (°C) */
    bool valid;         /**< True if accel/gyro reading is valid */
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
    int i2c_sda_gpio;           /**< I2C SDA pin */
    int i2c_scl_gpio;           /**< I2C SCL pin */
    uint8_t imu_addr;           /**< IMU I2C address (7-bit) */
    uint8_t urm09_addr;         /**< URM09 I2C address (7-bit) */
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
 * device registers. Automatically detects IMU type (MPU-9250/6500/6050).
 * Does NOT start the polling task.
 *
 * @param cfg Pointer to sensor configuration
 * @return ESP_OK on success
 */
esp_err_t sensor_init(const sensor_config_t *cfg);

/**
 * @brief Start the sensor polling task
 *
 * Creates a FreeRTOS task that periodically reads all sensors
 * and updates the internal data structures.
 *
 * @return ESP_OK on success
 */
esp_err_t sensor_task_start(void);

/**
 * @brief Get latest IMU reading
 *
 * Thread-safe. Copies the most recent IMU data into the provided struct.
 * Includes magnetometer data if MPU-9250 is detected.
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
 * @return true if IMU (any type) is present on the I2C bus
 */
bool sensor_imu_present(void);

/**
 * @brief Check if magnetometer is detected and responding
 *
 * Only returns true for MPU-9250 with functioning AK8963.
 *
 * @return true if magnetometer is available
 */
bool sensor_mag_present(void);

/**
 * @brief Check if URM09 is detected and responding
 *
 * @return true if URM09 is present on the I2C bus
 */
bool sensor_urm09_present(void);

/**
 * @brief Get detected IMU type as string
 *
 * @return "MPU-9250", "MPU-6500", "MPU-6050", or "None"
 */
const char *sensor_imu_type_str(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
