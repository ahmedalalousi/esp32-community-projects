/**
 * @file sensors.c
 * @brief Sensor implementation for URM09 ultrasonic and MPU-9250 IMU
 *
 * Uses the NEW ESP-IDF I2C master driver API (driver/i2c_master.h).
 * This driver cannot coexist with the legacy driver/i2c.h API.
 *
 * I2C bus shared between devices. The sensor task polls at a
 * configurable interval and feeds IMU yaw rate into the motor driver
 * PID controller via motor_driver_update_imu().
 *
 * MPU-9250 (InvenSense 9-axis IMU):
 *   The MPU-9250 contains two dies:
 *     - MPU-6500: 3-axis accelerometer + 3-axis gyroscope (I2C addr 0x68/0x69)
 *     - AK8963:   3-axis magnetometer (I2C addr 0x0C, accessed via bypass)
 *
 *   Register map (MPU-6500 portion):
 *     0x19 = Sample rate divider
 *     0x1A = Config (DLPF)
 *     0x1B = Gyro config (full-scale range)
 *     0x1C = Accel config (full-scale range)
 *     0x37 = INT_PIN_CFG (enable I2C bypass for AK8963 access)
 *     0x3B-0x48 = Accel/Temp/Gyro data (14 bytes)
 *     0x6B = Power management 1
 *     0x75 = WHO_AM_I (should read 0x71 for MPU-9250, 0x70 for MPU-6500)
 *
 *   AK8963 magnetometer (via bypass mode):
 *     0x00 = WIA (device ID, should read 0x48)
 *     0x02 = ST1 (status 1, bit 0 = data ready)
 *     0x03-0x08 = Mag X/Y/Z data (6 bytes, little-endian)
 *     0x09 = ST2 (status 2, must read to complete measurement)
 *     0x0A = CNTL1 (control, set mode)
 *     0x0C = CNTL2 (soft reset)
 *     0x10-0x12 = Sensitivity adjustment values (ASAX, ASAY, ASAZ)
 *
 * URM09 (DFRobot Gravity I2C Ultrasonic, SKU:SEN0304):
 *   - Default address: 0x11 (configurable via CONFIG_URM09_I2C_ADDR)
 *   - Register 0x07: configuration (mode + range)
 *   - Register 0x08: command (write 0x01 to trigger)
 *   - Register 0x03-0x04: distance in cm (big-endian uint16)
 *   - Measurement cycle: ~40ms for 500cm range
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "sensors.h"
#include "motor_driver.h"

static const char *TAG = "SENSORS";

/* =========================================================================
 * I2C Configuration
 * ========================================================================= */

#define I2C_MASTER_FREQ_HZ  400000      /* 400 kHz — MPU-9250 supports up to 400kHz */
#define I2C_TIMEOUT_MS      100

/* =========================================================================
 * MPU-9250 Register Map
 * ========================================================================= */

/* MPU-6500 (accel/gyro) registers */
#define MPU9250_REG_SMPLRT_DIV      0x19    /* Sample rate divider */
#define MPU9250_REG_CONFIG          0x1A    /* Configuration (DLPF) */
#define MPU9250_REG_GYRO_CONFIG     0x1B    /* Gyroscope configuration */
#define MPU9250_REG_ACCEL_CONFIG    0x1C    /* Accelerometer configuration */
#define MPU9250_REG_ACCEL_CONFIG2   0x1D    /* Accelerometer configuration 2 */
#define MPU9250_REG_INT_PIN_CFG     0x37    /* INT pin / bypass config */
#define MPU9250_REG_ACCEL_XOUT_H    0x3B    /* Accel X high byte */
#define MPU9250_REG_PWR_MGMT_1      0x6B    /* Power management 1 */
#define MPU9250_REG_PWR_MGMT_2      0x6C    /* Power management 2 */
#define MPU9250_REG_WHO_AM_I        0x75    /* Device ID */

/* WHO_AM_I expected values */
#define MPU9250_WHO_AM_I_VAL        0x71    /* MPU-9250 */
#define MPU6500_WHO_AM_I_VAL        0x70    /* MPU-6500 (no magnetometer) */
#define MPU6050_WHO_AM_I_VAL        0x68    /* MPU-6050 (fallback) */

/* INT_PIN_CFG bits */
#define MPU9250_BYPASS_EN           0x02    /* Enable I2C bypass for AK8963 */

/* AK8963 magnetometer registers (accessed via bypass mode) */
#define AK8963_I2C_ADDR             0x0C    /* Fixed I2C address */
#define AK8963_REG_WIA              0x00    /* Device ID (should read 0x48) */
#define AK8963_REG_ST1              0x02    /* Status 1 */
#define AK8963_REG_HXL              0x03    /* Mag X low byte */
#define AK8963_REG_ST2              0x09    /* Status 2 (must read to complete) */
#define AK8963_REG_CNTL1            0x0A    /* Control 1 */
#define AK8963_REG_CNTL2            0x0B    /* Control 2 (reset) */
#define AK8963_REG_ASAX             0x10    /* Sensitivity adjustment X */

#define AK8963_WHO_AM_I_VAL         0x48    /* Expected device ID */

/* AK8963 operating modes */
#define AK8963_MODE_POWER_DOWN      0x00
#define AK8963_MODE_SINGLE          0x01
#define AK8963_MODE_CONT_8HZ        0x02
#define AK8963_MODE_CONT_100HZ      0x06
#define AK8963_MODE_FUSE_ROM        0x0F
#define AK8963_BIT_16BIT            0x10    /* 16-bit output */

/* Conversion factors */
#define ACCEL_SCALE_2G              16384.0f    /* LSB/g at ±2g */
#define ACCEL_SCALE_4G              8192.0f     /* LSB/g at ±4g */
#define ACCEL_SCALE_8G              4096.0f     /* LSB/g at ±8g */
#define ACCEL_SCALE_16G             2048.0f     /* LSB/g at ±16g */

#define GYRO_SCALE_250DPS           131.0f      /* LSB/(°/s) at ±250°/s */
#define GYRO_SCALE_500DPS           65.5f       /* LSB/(°/s) at ±500°/s */
#define GYRO_SCALE_1000DPS          32.8f       /* LSB/(°/s) at ±1000°/s */
#define GYRO_SCALE_2000DPS          16.4f       /* LSB/(°/s) at ±2000°/s */

#define MAG_SCALE_16BIT             0.15f       /* µT/LSB in 16-bit mode */

#define DEG_TO_RAD                  0.017453292519943f
#define GRAVITY_MS2                 9.80665f

/* =========================================================================
 * URM09 Register Map (DFRobot SEN0304 datasheet)
 * ========================================================================= */

#define URM09_REG_DIST_H            0x03    /* Distance high byte */
#define URM09_REG_DIST_L            0x04    /* Distance low byte */
#define URM09_REG_CFG               0x07    /* Configuration register */
#define URM09_REG_CMD               0x08    /* Command register */

#define URM09_CMD_MEASURE           0x01    /* Trigger one measurement */

/* Config register bits */
#define URM09_CFG_PASSIVE           0x00    /* Passive mode (manual trigger) */
#define URM09_CFG_RANGE_500CM       0x20    /* 500cm range (~40ms cycle) */

/* Distance limits (datasheet: 2cm to 500cm, 1 LSB = 1 cm) */
#define URM09_MIN_DISTANCE_CM       2
#define URM09_MAX_DISTANCE_CM       500

/* =========================================================================
 * Driver State
 * ========================================================================= */

static sensor_config_t cfg;
static bool initialised = false;
static bool imu_detected = false;
static bool mag_detected = false;      /* AK8963 magnetometer */
static bool urm09_detected = false;

/* NEW I2C master driver handles */
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t imu_dev_handle = NULL;
static i2c_master_dev_handle_t mag_dev_handle = NULL;   /* AK8963 */
static i2c_master_dev_handle_t urm09_dev_handle = NULL;

/* IMU type detected */
typedef enum {
    IMU_TYPE_NONE = 0,
    IMU_TYPE_MPU6050,
    IMU_TYPE_MPU6500,
    IMU_TYPE_MPU9250
} imu_type_t;

static imu_type_t imu_type = IMU_TYPE_NONE;

/* AK8963 sensitivity adjustment values */
static float mag_adj_x = 1.0f;
static float mag_adj_y = 1.0f;
static float mag_adj_z = 1.0f;

/* Current scale factors (from Kconfig) */
static float accel_scale = ACCEL_SCALE_2G;
static float gyro_scale = GYRO_SCALE_250DPS;

/* Latest readings — protected by mutex */
static SemaphoreHandle_t data_mutex = NULL;
static imu_data_t latest_imu = {0};
static range_data_t latest_range = {0};

/* Task handle */
static TaskHandle_t sensor_task_handle = NULL;

/* =========================================================================
 * I2C Helpers (NEW API)
 * ========================================================================= */

/**
 * @brief Write a single byte to an I2C device register
 */
static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dev, buf, 2, I2C_TIMEOUT_MS);
}

/**
 * @brief Read bytes from an I2C device starting at a register
 */
static esp_err_t i2c_read_regs(i2c_master_dev_handle_t dev, uint8_t start_reg,
                                uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &start_reg, 1, data, len, I2C_TIMEOUT_MS);
}

/**
 * @brief Probe for a device on the I2C bus using the new API
 */
static bool i2c_probe_device(uint8_t addr)
{
    esp_err_t err = i2c_master_probe(i2c_bus_handle, addr, I2C_TIMEOUT_MS);
    return (err == ESP_OK);
}

/**
 * @brief Add a device to the I2C bus
 */
static esp_err_t i2c_add_device(uint8_t addr, i2c_master_dev_handle_t *dev_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, dev_handle);
}

/* =========================================================================
 * MPU-9250 Functions
 * ========================================================================= */

/**
 * @brief Initialise MPU-9250 (or MPU-6500/MPU-6050 fallback)
 *
 * Detection sequence:
 *   1. Read WHO_AM_I to identify device
 *   2. Wake device (clear sleep bit)
 *   3. Configure gyro and accel ranges
 *   4. Enable I2C bypass for magnetometer access
 *   5. Initialise AK8963 magnetometer (if MPU-9250)
 */
static esp_err_t mpu9250_init(void)
{
    esp_err_t err;
    uint8_t who;

    /* Read WHO_AM_I */
    err = i2c_read_regs(imu_dev_handle, MPU9250_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Identify IMU type */
    switch (who) {
        case MPU9250_WHO_AM_I_VAL:
            imu_type = IMU_TYPE_MPU9250;
            ESP_LOGI(TAG, "MPU-9250 detected (WHO_AM_I=0x%02X)", who);
            break;
        case MPU6500_WHO_AM_I_VAL:
            imu_type = IMU_TYPE_MPU6500;
            ESP_LOGI(TAG, "MPU-6500 detected (WHO_AM_I=0x%02X) — no magnetometer", who);
            break;
        case MPU6050_WHO_AM_I_VAL:
            imu_type = IMU_TYPE_MPU6050;
            ESP_LOGI(TAG, "MPU-6050 detected (WHO_AM_I=0x%02X) — legacy device", who);
            break;
        default:
            imu_type = IMU_TYPE_MPU9250;  /* Assume MPU-9250 compatible */
            ESP_LOGW(TAG, "Unknown IMU WHO_AM_I=0x%02X, assuming MPU-9250 compatible", who);
            break;
    }

    /* Reset device */
    err = i2c_write_reg(imu_dev_handle, MPU9250_REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU reset failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  /* Wait for reset */

    /* Wake up (auto-select best clock source) */
    err = i2c_write_reg(imu_dev_handle, MPU9250_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU wake failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Configure gyroscope range */
    uint8_t gyro_cfg = 0;
#if defined(CONFIG_IMU_GYRO_RANGE_250)
    gyro_cfg = 0x00;
    gyro_scale = GYRO_SCALE_250DPS;
#elif defined(CONFIG_IMU_GYRO_RANGE_500)
    gyro_cfg = 0x08;
    gyro_scale = GYRO_SCALE_500DPS;
#elif defined(CONFIG_IMU_GYRO_RANGE_1000)
    gyro_cfg = 0x10;
    gyro_scale = GYRO_SCALE_1000DPS;
#elif defined(CONFIG_IMU_GYRO_RANGE_2000)
    gyro_cfg = 0x18;
    gyro_scale = GYRO_SCALE_2000DPS;
#else
    gyro_cfg = 0x00;
    gyro_scale = GYRO_SCALE_250DPS;
#endif
    err = i2c_write_reg(imu_dev_handle, MPU9250_REG_GYRO_CONFIG, gyro_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Gyro config failed: %s", esp_err_to_name(err));
    }

    /* Configure accelerometer range */
    uint8_t accel_cfg = 0;
#if defined(CONFIG_IMU_ACCEL_RANGE_2G)
    accel_cfg = 0x00;
    accel_scale = ACCEL_SCALE_2G;
#elif defined(CONFIG_IMU_ACCEL_RANGE_4G)
    accel_cfg = 0x08;
    accel_scale = ACCEL_SCALE_4G;
#elif defined(CONFIG_IMU_ACCEL_RANGE_8G)
    accel_cfg = 0x10;
    accel_scale = ACCEL_SCALE_8G;
#elif defined(CONFIG_IMU_ACCEL_RANGE_16G)
    accel_cfg = 0x18;
    accel_scale = ACCEL_SCALE_16G;
#else
    accel_cfg = 0x00;
    accel_scale = ACCEL_SCALE_2G;
#endif
    err = i2c_write_reg(imu_dev_handle, MPU9250_REG_ACCEL_CONFIG, accel_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Accel config failed: %s", esp_err_to_name(err));
    }

    /* Enable I2C bypass to access AK8963 directly */
    err = i2c_write_reg(imu_dev_handle, MPU9250_REG_INT_PIN_CFG, MPU9250_BYPASS_EN);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bypass enable failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "IMU initialised at 0x%02X (accel=±%dg, gyro=±%d°/s)",
             cfg.imu_addr,
             (accel_scale == ACCEL_SCALE_2G) ? 2 :
             (accel_scale == ACCEL_SCALE_4G) ? 4 :
             (accel_scale == ACCEL_SCALE_8G) ? 8 : 16,
             (gyro_scale == GYRO_SCALE_250DPS) ? 250 :
             (gyro_scale == GYRO_SCALE_500DPS) ? 500 :
             (gyro_scale == GYRO_SCALE_1000DPS) ? 1000 : 2000);

    return ESP_OK;
}

/**
 * @brief Initialise AK8963 magnetometer (inside MPU-9250)
 *
 * Must be called after mpu9250_init() enables bypass mode.
 */
static esp_err_t ak8963_init(void)
{
    esp_err_t err;
    uint8_t who;

    /* Check if AK8963 is accessible */
    if (!i2c_probe_device(AK8963_I2C_ADDR)) {
        ESP_LOGW(TAG, "AK8963 not found at 0x%02X (bypass may not be enabled)",
                 AK8963_I2C_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    /* Add AK8963 as a device on the bus */
    err = i2c_add_device(AK8963_I2C_ADDR, &mag_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AK8963 device: %s", esp_err_to_name(err));
        return err;
    }

    /* Read WHO_AM_I */
    err = i2c_read_regs(mag_dev_handle, AK8963_REG_WIA, &who, 1);
    if (err != ESP_OK || who != AK8963_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "AK8963 WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, AK8963_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    /* Power down before changing mode */
    err = i2c_write_reg(mag_dev_handle, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Enter Fuse ROM access mode to read sensitivity adjustment values */
    err = i2c_write_reg(mag_dev_handle, AK8963_REG_CNTL1, AK8963_MODE_FUSE_ROM);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Read sensitivity adjustment values */
    uint8_t asa[3];
    err = i2c_read_regs(mag_dev_handle, AK8963_REG_ASAX, asa, 3);
    if (err == ESP_OK) {
        /* Hadj = H × ((ASA - 128) × 0.5 / 128 + 1) */
        mag_adj_x = ((float)(asa[0] - 128) * 0.5f / 128.0f) + 1.0f;
        mag_adj_y = ((float)(asa[1] - 128) * 0.5f / 128.0f) + 1.0f;
        mag_adj_z = ((float)(asa[2] - 128) * 0.5f / 128.0f) + 1.0f;
        ESP_LOGD(TAG, "AK8963 sensitivity: X=%.3f Y=%.3f Z=%.3f",
                 mag_adj_x, mag_adj_y, mag_adj_z);
    }

    /* Power down again */
    err = i2c_write_reg(mag_dev_handle, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set continuous measurement mode (100Hz) with 16-bit output */
    err = i2c_write_reg(mag_dev_handle, AK8963_REG_CNTL1,
                        AK8963_MODE_CONT_100HZ | AK8963_BIT_16BIT);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "AK8963 magnetometer initialised (100Hz, 16-bit)");
    return ESP_OK;
}

/**
 * @brief Read MPU-9250 accelerometer, temperature, and gyroscope
 *
 * Burst-reads 14 bytes starting from ACCEL_XOUT_H:
 *   [0-1]   Accel X   (big-endian int16)
 *   [2-3]   Accel Y
 *   [4-5]   Accel Z
 *   [6-7]   Temperature
 *   [8-9]   Gyro X
 *   [10-11] Gyro Y
 *   [12-13] Gyro Z
 */
static esp_err_t mpu9250_read(imu_data_t *out)
{
    uint8_t raw[14];
    esp_err_t err = i2c_read_regs(imu_dev_handle, MPU9250_REG_ACCEL_XOUT_H, raw, 14);
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
    out->accel_x = ((float)ax / accel_scale) * GRAVITY_MS2;
    out->accel_y = ((float)ay / accel_scale) * GRAVITY_MS2;
    out->accel_z = ((float)az / accel_scale) * GRAVITY_MS2;

    /* Convert gyroscope to rad/s */
    out->gyro_x = ((float)gx / gyro_scale) * DEG_TO_RAD;
    out->gyro_y = ((float)gy / gyro_scale) * DEG_TO_RAD;
    out->gyro_z = ((float)gz / gyro_scale) * DEG_TO_RAD;

    /* Convert temperature: T(°C) = (raw - RoomTemp_Offset) / Sensitivity + 21
     * For MPU-9250: Sensitivity = 333.87, RoomTemp_Offset = 0 */
    out->temperature = ((float)temp_raw / 333.87f) + 21.0f;

    out->valid = true;
    return ESP_OK;
}

/**
 * @brief Read AK8963 magnetometer
 *
 * Reads 7 bytes: HXL, HXH, HYL, HYH, HZL, HZH, ST2
 * ST2 must be read to signal measurement complete.
 * Data is little-endian (unlike MPU-9250 accel/gyro).
 */
static esp_err_t ak8963_read(imu_data_t *out)
{
    uint8_t st1;
    esp_err_t err;

    if (mag_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if data is ready */
    err = i2c_read_regs(mag_dev_handle, AK8963_REG_ST1, &st1, 1);
    if (err != ESP_OK) {
        return err;
    }

    if (!(st1 & 0x01)) {
        /* Data not ready — use previous values */
        return ESP_OK;
    }

    /* Read magnetometer data + ST2 (7 bytes) */
    uint8_t raw[7];
    err = i2c_read_regs(mag_dev_handle, AK8963_REG_HXL, raw, 7);
    if (err != ESP_OK) {
        return err;
    }

    /* Check for magnetic sensor overflow (ST2 bit 3) */
    if (raw[6] & 0x08) {
        ESP_LOGD(TAG, "AK8963 overflow detected");
        return ESP_OK;  /* Keep previous values */
    }

    /* Combine bytes (little-endian) */
    int16_t mx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t my = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t mz = (int16_t)((raw[5] << 8) | raw[4]);

    /* Apply sensitivity adjustment and convert to µT */
    out->mag_x = (float)mx * mag_adj_x * MAG_SCALE_16BIT;
    out->mag_y = (float)my * mag_adj_y * MAG_SCALE_16BIT;
    out->mag_z = (float)mz * mag_adj_z * MAG_SCALE_16BIT;
    out->mag_valid = true;

    return ESP_OK;
}

/* =========================================================================
 * URM09 Functions
 * ========================================================================= */

/**
 * @brief Initialise URM09 sensor
 *
 * Configures passive mode with 500cm range.
 */
static esp_err_t urm09_init(void)
{
    /* Configure: passive mode, 500cm range */
    uint8_t cfg_val = URM09_CFG_PASSIVE | URM09_CFG_RANGE_500CM;
    esp_err_t err = i2c_write_reg(urm09_dev_handle, URM09_REG_CFG, cfg_val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "URM09 config write failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "URM09 initialised at 0x%02X (range=500cm, passive mode)",
             cfg.urm09_addr);
    return ESP_OK;
}

/**
 * @brief Trigger a URM09 measurement
 */
static esp_err_t urm09_trigger(void)
{
    return i2c_write_reg(urm09_dev_handle, URM09_REG_CMD, URM09_CMD_MEASURE);
}

/**
 * @brief Read URM09 distance result
 */
static esp_err_t urm09_read(range_data_t *out)
{
    uint8_t raw[2];
    esp_err_t err = i2c_read_regs(urm09_dev_handle, URM09_REG_DIST_H, raw, 2);
    if (err != ESP_OK) {
        out->valid = false;
        return err;
    }

    /* Distance is big-endian, units are centimetres */
    uint16_t dist_cm = ((uint16_t)raw[0] << 8) | raw[1];

    /* Validate range */
    if (dist_cm < URM09_MIN_DISTANCE_CM || dist_cm > URM09_MAX_DISTANCE_CM) {
        out->distance_m = 0.0f;
        out->valid = false;
    } else {
        out->distance_m = (float)dist_cm / 100.0f;
        out->valid = true;
    }

    return ESP_OK;
}

/* =========================================================================
 * Sensor Task
 * ========================================================================= */

/**
 * @brief FreeRTOS task that polls sensors at regular intervals
 */
static void sensor_poll_task(void *arg)
{
    (void)arg;

    imu_data_t imu_reading = {0};
    range_data_t range_reading = {0};

    ESP_LOGI(TAG, "Sensor task started (poll interval: %lu ms)",
             (unsigned long)cfg.poll_interval_ms);

    while (1) {
        /* Step 1: Trigger ultrasonic measurement (non-blocking) */
        if (urm09_detected) {
            urm09_trigger();
        }

        /* Step 2: Read IMU accel/gyro */
        if (imu_detected) {
            if (mpu9250_read(&imu_reading) == ESP_OK && imu_reading.valid) {
                /* Feed yaw rate to motor driver PID */
                motor_driver_update_imu(imu_reading.gyro_z);
            }
        }

        /* Step 3: Read magnetometer (if available) */
        if (mag_detected) {
            ak8963_read(&imu_reading);
        }

        /* Step 4: Wait for URM09 measurement */
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
        .poll_interval_ms = CONFIG_SENSOR_POLL_INTERVAL_MS
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

    /*
     * Configure I2C master bus.
     *
     * On Waveshare ESP32-S3-ETH, GPIO 47/48 are the I2C bus pins shared
     * between the camera SCCB interface and external sensors (IMU, URM09).
     *
     * Strategy:
     *   - If camera is enabled: Camera init runs FIRST and creates the
     *     SCCB I2C bus on I2C_NUM_1. We get that existing bus handle.
     *   - If camera is disabled: We create our own I2C bus.
     *
     * The new I2C driver API (i2c_master.h) allows us to get an existing
     * bus handle via i2c_master_get_bus_handle() and add our devices to it.
     */
    esp_err_t err;

#ifdef CONFIG_CAMERA_ENABLED
    /*
     * Camera is enabled — it has already created the SCCB I2C bus.
     * Get the existing bus handle on I2C_NUM_1.
     */
    err = i2c_master_get_bus_handle(I2C_NUM_1, &i2c_bus_handle);
    if (err != ESP_OK || i2c_bus_handle == NULL) {
        ESP_LOGW(TAG, "Could not get camera's I2C bus handle: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "External sensors will not be available");
        i2c_bus_handle = NULL;
        imu_detected = false;
        urm09_detected = false;
        initialised = true;
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Sharing I2C bus with camera SCCB (port 1, SDA=%d, SCL=%d)",
             cfg.i2c_sda_gpio, cfg.i2c_scl_gpio);
#else
    /* Camera disabled — create our own I2C bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = cfg.i2c_sda_gpio,
        .scl_io_num = cfg.i2c_scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    err = i2c_new_master_bus(&bus_cfg, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master bus initialised on port 1 (SDA=%d, SCL=%d, %dkHz)",
             cfg.i2c_sda_gpio, cfg.i2c_scl_gpio, I2C_MASTER_FREQ_HZ / 1000);
#endif

    /* Probe and initialise IMU */
    imu_detected = i2c_probe_device(cfg.imu_addr);
    if (imu_detected) {
        /* Add IMU as a device on the bus */
        err = i2c_add_device(cfg.imu_addr, &imu_dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add IMU device: %s", esp_err_to_name(err));
            imu_detected = false;
        } else {
            err = mpu9250_init();
            if (err != ESP_OK) {
                imu_detected = false;
            } else {
                /* Try to initialise magnetometer (MPU-9250 only) */
                if (imu_type == IMU_TYPE_MPU9250) {
                    if (ak8963_init() == ESP_OK) {
                        mag_detected = true;
                    }
                }
            }
        }
    } else {
        ESP_LOGW(TAG, "IMU not found at 0x%02X", cfg.imu_addr);
    }

    /* Probe and initialise URM09 */
    urm09_detected = i2c_probe_device(cfg.urm09_addr);
    if (urm09_detected) {
        /* Add URM09 as a device on the bus */
        err = i2c_add_device(cfg.urm09_addr, &urm09_dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add URM09 device: %s", esp_err_to_name(err));
            urm09_detected = false;
        } else {
            err = urm09_init();
            if (err != ESP_OK) {
                urm09_detected = false;
            }
        }
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

bool sensor_mag_present(void)
{
    return mag_detected;
}

bool sensor_urm09_present(void)
{
    return urm09_detected;
}

const char *sensor_imu_type_str(void)
{
    switch (imu_type) {
        case IMU_TYPE_MPU9250: return "MPU-9250";
        case IMU_TYPE_MPU6500: return "MPU-6500";
        case IMU_TYPE_MPU6050: return "MPU-6050";
        default: return "None";
    }
}
