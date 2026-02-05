/**
 * @file motor_driver.h
 * @brief Motor Driver interface for skid-steer robot
 *
 * Provides PWM motor control with skid-steer kinematics and optional
 * PID heading correction using IMU feedback.
 *
 * Hardware assumptions:
 *   - Each motor has a PWM pin (speed) and DIR pin (direction)
 *   - PWM via ESP32 LEDC peripheral (10-bit, 0-1023)
 *   - Direction: HIGH = forward, LOW = reverse
 *   - All GPIO assignments configured via Kconfig menuconfig
 *
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Motor Identification
 * ========================================================================= */

/**
 * @brief Motor identifiers
 *
 * Corresponds to physical motor positions on the chassis.
 * Also used as LEDC channel index (0-3).
 */
typedef enum {
    MOTOR_FRONT_LEFT  = 0,
    MOTOR_FRONT_RIGHT = 1,
    MOTOR_REAR_LEFT   = 2,
    MOTOR_REAR_RIGHT  = 3,
    MOTOR_COUNT       = 4
} motor_id_t;

/* =========================================================================
 * Drive Modes
 * ========================================================================= */

/**
 * @brief Drive mode selection
 *
 * Determines which motors are driven and how kinematics are applied.
 */
typedef enum {
    DRIVE_MODE_TWO_WHEEL  = 0,  /**< Rear motors only, front free-rolling */
    DRIVE_MODE_FOUR_WHEEL = 1,  /**< All four motors driven */
    DRIVE_MODE_TRACKED    = 2   /**< Left/right pairs locked (tank drive) */
} drive_mode_t;

/* =========================================================================
 * Feedback Sources
 * ========================================================================= */

/**
 * @brief Feedback source for PID heading correction
 */
typedef enum {
    FEEDBACK_NONE     = 0,  /**< No feedback — open-loop control */
    FEEDBACK_IMU      = 1,  /**< IMU yaw rate (gyroscope Z-axis) */
    FEEDBACK_ENCODERS = 2   /**< Wheel encoders (future) */
} feedback_source_t;

/* =========================================================================
 * PID Parameters
 * ========================================================================= */

/**
 * @brief PID controller parameters for heading correction
 *
 * The PID operates on yaw rate error:
 *   error = desired_yaw_rate - actual_yaw_rate (from IMU)
 *
 * Output is a correction factor applied differentially to left/right motors.
 */
typedef struct {
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float kd;               /**< Derivative gain */
    float integral_limit;   /**< Anti-windup: max |integral accumulator| */
    float output_limit;     /**< Max |correction output| */
} pid_params_t;

/* =========================================================================
 * Driver Configuration
 * ========================================================================= */

/**
 * @brief Motor driver configuration
 *
 * Populated from Kconfig defaults via motor_driver_default_config(),
 * or set manually before calling motor_driver_init().
 */
typedef struct {
    drive_mode_t drive_mode;    /**< Which motors are driven */
    float wheel_base_m;         /**< Distance between left and right wheels (metres) */
    float max_speed_ms;         /**< Maximum linear speed (m/s) at 100% duty */
    float max_pwm_duty;         /**< Maximum PWM duty percentage (0-100) */
    pid_params_t heading_pid;   /**< PID parameters for heading correction */
    bool pid_enabled;           /**< Whether PID correction is active */
} motor_driver_config_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Get default configuration from Kconfig values
 *
 * @return Configuration struct populated from menuconfig settings
 */
motor_driver_config_t motor_driver_default_config(void);

/**
 * @brief Initialise the motor driver
 *
 * Configures LEDC timer, PWM channels, and direction GPIOs for all motors.
 * Must be called once before any other motor_driver_* functions.
 *
 * @param cfg Pointer to configuration (use motor_driver_default_config())
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already initialised
 */
esp_err_t motor_driver_init(const motor_driver_config_t *cfg);

/**
 * @brief Set robot velocity using skid-steer kinematics
 *
 * Converts linear/angular velocity to left/right motor speeds,
 * applies PID heading correction if enabled.
 *
 * @param linear  Linear velocity in m/s (positive = forward)
 * @param angular Angular velocity in rad/s (positive = counter-clockwise)
 */
void motor_driver_set_velocity(float linear, float angular);

/**
 * @brief Update IMU yaw rate for PID feedback
 *
 * Called by the sensor task whenever a new IMU reading is available.
 * Thread-safe (single float write is atomic on ESP32).
 *
 * @param yaw_rate Yaw rate in rad/s from gyroscope Z-axis
 */
void motor_driver_update_imu(float yaw_rate);

/**
 * @brief Set individual motor speed (bypasses kinematics)
 *
 * @param motor Motor identifier
 * @param speed Speed percentage (-100 to +100)
 */
void motor_driver_set_motor(motor_id_t motor, int8_t speed);

/**
 * @brief Change drive mode at runtime
 *
 * Stops all motors during mode transition.
 *
 * @param mode New drive mode
 */
void motor_driver_set_drive_mode(drive_mode_t mode);

/**
 * @brief Get current drive mode
 *
 * @return Current drive mode
 */
drive_mode_t motor_driver_get_drive_mode(void);

/**
 * @brief Change feedback source for PID
 *
 * Resets PID state when source changes.
 *
 * @param source New feedback source
 */
void motor_driver_set_feedback(feedback_source_t source);

/**
 * @brief Update PID parameters at runtime
 *
 * Resets PID state (integral, previous error) after update.
 *
 * @param params Pointer to new PID parameters
 */
void motor_driver_set_pid_params(const pid_params_t *params);

/**
 * @brief Stop all motors gracefully
 *
 * Sets all motor speeds to zero and resets PID state.
 */
void motor_driver_stop(void);

/**
 * @brief Emergency stop — immediate PWM kill
 *
 * Bypasses initialisation check. Use in fault conditions.
 */
void motor_driver_emergency_stop(void);

/**
 * @brief Get current speed of a motor
 *
 * @param motor Motor identifier
 * @return Current speed percentage (-100 to +100), 0 if invalid motor
 */
int8_t motor_driver_get_speed(motor_id_t motor);

/**
 * @brief Process encoder tick update (future use)
 *
 * Placeholder for closed-loop speed control via wheel encoders.
 *
 * @param motor Motor identifier
 * @param ticks Encoder tick count
 */
void motor_driver_encoder_update(motor_id_t motor, int32_t ticks);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
