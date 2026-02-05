/**
 * @file sensors.c
 * @brief Sensor implementation for URM09 ultrasonic and MPU-6050 IMU
 *
 * I2C bus shared between both devices. The sensor task polls at a
 * configurable interval and feeds IMU yaw rate into the motor driver
 * PID controller via motor_driver_update_imu().
 *
 * URM09 (DFRobot I2C Ultrasonic):
 *   - Default address: 0x11
 *   - Register 0x01: trigger measurement (write 0x01)
 *   - Register 0x03-0x04: distance in mm (big-endian uint16)
 *   - Measurement time: ~100ms
 *
 * MPU-6050:
 *   - Default address: 0x68
 *   - Register 0x6B: power management (write 0x00 to wake)
 *   - Registers 0x3B-0x48: accel/temp/gyro raw data (14 bytes)
 *   - Accel scale: ±2g  → 16384 LSB/g
 *   - Gyro scale:  ±250°/s → 131 LSB/(°/s)
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "sensors.h"
#include "motor_driver.h"

static const char *TAG = "SENSORS";

/* =========================================================================
 * I2C Configuration
 * ========================================================================= */

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000      /* 100 kHz — safe for both devices */
#define I2C_TIMEOUT_MS      100

/* =========================================================================
 * URM09 Register Map
 * ========================================================================= */

#define URM09_REG_CMD           0x01    /* Command register */
#define URM09_REG_DIST_H        0x03    /* Distance high byte */
#define URM09_REG_DIST_L        0x04    /* Distance low byte */
#define URM09_CMD_MEASURE       0x01    /* Trigger one measurement */

#define URM09_MIN_DISTANCE_MM   20      /* Minimum measurable distance */
#define URM09_MAX_DISTANCE_MM   7500    /* Maximum measurable distance */

/* =========================================================================
 * MPU-6050 Register Map
 * ========================================================================= */

#define MPU6050_REG_PWR_MGMT_1  0x6B    /* Power management 1 */
#define MPU6050_REG_WHO_AM_I    0x75    /* Device identity (should read 0x68) */
#define MPU6050_REG_ACCEL_XOUT  0x3B    /* Start of 14-byte burst read */

#define MPU6050_WHO_AM_I_VAL    0x68    /* Expected WHO_AM_I response */

/* Conversion factors */
#define ACCEL_SCALE_2G          16384.0f    /* LSB/g at ±2g */
#define GYRO_SCALE_250DPS       131.0f      /* LSB/(°/s) at ±250°/s */
#define DEG_TO_RAD              0.017453292519943f
#define GRAVITY_MS2             9.80665f

/* =========================================================================
 * Driver State
 * ========================================================================= */

static sensor_config_t cfg;
static bool initialised = false;
static bool imu_detected = false;
static bool urm09_detected = false;

/* Latest readings — protected by mutex */
static SemaphoreHandle_t data_mutex = NULL;
static imu_data_t latest_imu = {0};
static range_data_t latest_range = {0};

/* Task handle */
static TaskHandle_t sensor_task_handle = NULL;

/* =========================================================================
 * I2C Helpers
 * ========================================================================= */

/**
 * @brief Write a single byte to an I2C device register
 */
static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, buf, 2,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * @brief Read bytes from an I2C device starting at a register
 */
static esp_err_t i2c_read_regs(uint8_t addr, uint8_t start_reg,
                                uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr,
                                         &start_reg, 1,
                                         data, len,
                                         pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * @brief Probe for a device on the I2C bus
 */
static bool i2c_probe(uint8_t addr)
{
    uint8_t dummy;
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, addr,
                                                &dummy, 0,
                                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    return (err == ESP_OK);
}

/* =========================================================================
 * MPU-6050 Functions
 * ========================================================================= */

/**
 * @brief Initialise MPU-6050
 *
 * Wakes the device and verifies WHO_AM_I register.
 */
static esp_err_t mpu6050_init(void)
{
    /* Check WHO_AM_I */
    uint8_t who;
    esp_err_t err = i2c_read_regs(cfg.imu_addr, MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6050 WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }

    if (who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "MPU-6050 WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, MPU6050_WHO_AM_I_VAL);
        /* Continue anyway — some clones report different values */
    }

    /* Wake up (clear sleep bit) */
    err = i2c_write_reg(cfg.imu_addr, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6050 wake failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU-6050 initialised at 0x%02X (WHO_AM_I=0x%02X)",
             cfg.imu_addr, who);
    return ESP_OK;
}

/**
 * @brief Read MPU-6050 accelerometer, temperature, and gyroscope
 *
 * Burst-reads 14 bytes starting from ACCEL_XOUT_H:
 *   [0-1]  Accel X   (big-endian int16)
 *   [2-3]  Accel Y
 *   [4-5]  Accel Z
 *   [6-7]  Temperature
 *   [8-9]  Gyro X
 *   [10-11] Gyro Y
 *   [12-13] Gyro Z
 */
static esp_err_t mpu6050_read(imu_data_t *out)
{
    uint8_t raw[14];
    esp_err_t err = i2c_read_regs(cfg.imu_addr, MPU6050_REG_ACCEL_XOUT, raw, 14);
    if (err != ESP_OK) {
        out->valid = false;
        return err;
    }

    /* Combine bytes into signed 16-bit values (big-endian) */
    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t temp_raw = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    /* Convert accelerometer to m/s² */
    out->accel_x = ((float)ax / ACCEL_SCALE_2G) * GRAVITY_MS2;
    out->accel_y = ((float)ay / ACCEL_SCALE_2G) * GRAVITY_MS2;
    out->accel_z = ((float)az / ACCEL_SCALE_2G) * GRAVITY_MS2;

    /* Convert gyroscope to rad/s */
    out->gyro_x = ((float)gx / GYRO_SCALE_250DPS) * DEG_TO_RAD;
    out->gyro_y = ((float)gy / GYRO_SCALE_250DPS) * DEG_TO_RAD;
    out->gyro_z = ((float)gz / GYRO_SCALE_250DPS) * DEG_TO_RAD;

    /* Convert temperature: T(°C) = raw/340 + 36.53 */
    out->temperature = ((float)temp_raw / 340.0f) + 36.53f;

    out->valid = true;
    return ESP_OK;
}

/* =========================================================================
 * URM09 Functions
 * ========================================================================= */

/**
 * @brief Trigger a URM09 measurement
 */
static esp_err_t urm09_trigger(void)
{
    return i2c_write_reg(cfg.urm09_addr, URM09_REG_CMD, URM09_CMD_MEASURE);
}

/**
 * @brief Read URM09 distance result
 *
 * Must be called ≥100ms after urm09_trigger().
 */
static esp_err_t urm09_read(range_data_t *out)
{
    uint8_t raw[2];
    esp_err_t err = i2c_read_regs(cfg.urm09_addr, URM09_REG_DIST_H, raw, 2);
    if (err != ESP_OK) {
        out->valid = false;
        return err;
    }

    uint16_t dist_mm = ((uint16_t)raw[0] << 8) | raw[1];

    /* Validate range */
    if (dist_mm < URM09_MIN_DISTANCE_MM || dist_mm > URM09_MAX_DISTANCE_MM) {
        out->distance_m = 0.0f;
        out->valid = false;
    } else {
        out->distance_m = (float)dist_mm / 1000.0f;
        out->valid = true;
    }

    return ESP_OK;
}

/* =========================================================================
 * Sensor Task
 * ========================================================================= */

/**
 * @brief FreeRTOS task that polls sensors at regular intervals
 *
 * Flow per iteration:
 *   1. Trigger URM09 measurement
 *   2. Read IMU (fast — no trigger delay)
 *   3. Feed yaw rate to motor driver PID
 *   4. Wait ~100ms for URM09
 *   5. Read URM09 distance
 *   6. Update shared data under mutex
 */
static void sensor_poll_task(void *arg)
{
    (void)arg;

    imu_data_t imu_reading;
    range_data_t range_reading;

    ESP_LOGI(TAG, "Sensor task started (poll interval: %lu ms)",
             (unsigned long)cfg.poll_interval_ms);

    while (1) {
        /* Step 1: Trigger ultrasonic measurement (non-blocking) */
        if (urm09_detected) {
            urm09_trigger();
        }

        /* Step 2: Read IMU immediately (no trigger needed) */
        if (imu_detected) {
            if (mpu6050_read(&imu_reading) == ESP_OK && imu_reading.valid) {
                /* Step 3: Feed yaw rate to motor driver PID */
                motor_driver_update_imu(imu_reading.gyro_z);
            }
        }

        /* Step 4: Wait for URM09 measurement to complete */
        vTaskDelay(pdMS_TO_TICKS(cfg.poll_interval_ms));

        /* Step 5: Read ultrasonic distance */
        if (urm09_detected) {
            urm09_read(&range_reading);
        }

        /* Step 6: Update shared data under mutex */
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (imu_detected) {
                memcpy(&latest_imu, &imu_reading, sizeof(imu_data_t));
            }
            if (urm09_detected) {
                memcpy(&latest_range, &range_reading, sizeof(range_data_t));
            }
            xSemaphoreGive(data_mutex);
        }
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

sensor_config_t sensor_default_config(void)
{
    sensor_config_t c = {
        .i2c_sda_gpio = CONFIG_SENSOR_I2C_SDA,
        .i2c_scl_gpio = CONFIG_SENSOR_I2C_SCL,
        .imu_addr = CONFIG_IMU_I2C_ADDR,
        .urm09_addr = CONFIG_URM09_I2C_ADDR,
        .poll_interval_ms = 100     /* 10 Hz default */
    };
    return c;
}

esp_err_t sensor_init(const sensor_config_t *config)
{
    if (initialised) {
        ESP_LOGW(TAG, "Sensors already initialised");
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(&cfg, config, sizeof(sensor_config_t));

    /* Create data mutex */
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Configure I2C master */
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg.i2c_sda_gpio,
        .scl_io_num = cfg.i2c_scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &i2c_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialised (SDA=%d, SCL=%d)",
             cfg.i2c_sda_gpio, cfg.i2c_scl_gpio);

    /* Probe and initialise IMU */
    imu_detected = i2c_probe(cfg.imu_addr);
    if (imu_detected) {
        err = mpu6050_init();
        if (err != ESP_OK) {
            imu_detected = false;
        }
    } else {
        ESP_LOGW(TAG, "MPU-6050 not found at 0x%02X", cfg.imu_addr);
    }

    /* Probe URM09 */
    urm09_detected = i2c_probe(cfg.urm09_addr);
    if (urm09_detected) {
        ESP_LOGI(TAG, "URM09 detected at 0x%02X", cfg.urm09_addr);
    } else {
        ESP_LOGW(TAG, "URM09 not found at 0x%02X", cfg.urm09_addr);
    }

    if (!imu_detected && !urm09_detected) {
        ESP_LOGW(TAG, "No sensors detected — task will be idle");
    }

    initialised = true;
    return ESP_OK;
}

esp_err_t sensor_task_start(void)
{
    if (!initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor_task_handle != NULL) {
        ESP_LOGW(TAG, "Sensor task already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreate(
        sensor_poll_task,
        "sensor_poll",
        CONFIG_SENSOR_TASK_STACK_SIZE,
        NULL,
        CONFIG_SENSOR_TASK_PRIORITY,
        &sensor_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t sensor_get_imu(imu_data_t *data)
{
    if (!initialised || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(data, &latest_imu, sizeof(imu_data_t));
        xSemaphoreGive(data_mutex);
        return data->valid ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t sensor_get_range(range_data_t *data)
{
    if (!initialised || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(data, &latest_range, sizeof(range_data_t));
        xSemaphoreGive(data_mutex);
        return data->valid ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    return ESP_ERR_TIMEOUT;
}

bool sensor_imu_present(void)
{
    return imu_detected;
}

bool sensor_urm09_present(void)
{
    return urm09_detected;
}
