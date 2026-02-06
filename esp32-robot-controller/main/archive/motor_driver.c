/**
 * @file motor_driver.c
 * @brief Motor Driver implementation for skid-steer robot
 * 
 * @author Ahmed Al-Alousi
 * @date February 2026
 */

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "motor_driver.h"

static const char *TAG = "MOTOR_DRV";

/* =========================================================================
 * GPIO Configuration
 *
 * Each motor requires two GPIOs:
 *   - PWM pin: Controls speed via duty cycle (LEDC)
 *   - DIR pin: Controls direction (HIGH = forward, LOW = reverse)
 *
 * All pin assignments are configured via Kconfig (idf.py menuconfig)
 * under "Motor Driver Configuration". The defaults avoid conflict with:
 *   - W5500 SPI (GPIO 9-14)
 *   - USB (GPIO 19-20)
 *   - UART (GPIO 43-44)
 *   - Flash/PSRAM (GPIO 26-37)
 *   - I2C sensors (per I2C Bus Configuration)
 *   - Camera interface (if enabled)
 * ========================================================================= */

/* Motor GPIO assignments (PWM, Direction) */
typedef struct {
    gpio_num_t pwm_gpio;
    gpio_num_t dir_gpio;
} motor_pins_t;

static const motor_pins_t motor_pins[MOTOR_COUNT] = {
    [MOTOR_FRONT_LEFT]  = { .pwm_gpio = CONFIG_MOTOR_FL_PWM_GPIO,
                            .dir_gpio = CONFIG_MOTOR_FL_DIR_GPIO },
    [MOTOR_FRONT_RIGHT] = { .pwm_gpio = CONFIG_MOTOR_FR_PWM_GPIO,
                            .dir_gpio = CONFIG_MOTOR_FR_DIR_GPIO },
    [MOTOR_REAR_LEFT]   = { .pwm_gpio = CONFIG_MOTOR_RL_PWM_GPIO,
                            .dir_gpio = CONFIG_MOTOR_RL_DIR_GPIO },
    [MOTOR_REAR_RIGHT]  = { .pwm_gpio = CONFIG_MOTOR_RR_PWM_GPIO,
                            .dir_gpio = CONFIG_MOTOR_RR_DIR_GPIO },
};

/* PWM configuration — from Kconfig */
#define PWM_FREQUENCY   CONFIG_MOTOR_PWM_FREQ_HZ
#define PWM_RESOLUTION  LEDC_TIMER_10_BIT  /* 0-1023 duty range */
#define PWM_MAX_DUTY    1023

/* =========================================================================
 * Driver State
 * ========================================================================= */

static motor_driver_config_t config;
static int8_t motor_speeds[MOTOR_COUNT] = {0};
static bool initialised = false;

/* PID state */
typedef struct {
    float integral;
    float previous_error;
    float desired_yaw_rate;
    float actual_yaw_rate;
    TickType_t last_update;
} pid_state_t;

static pid_state_t pid_state = {0};
static feedback_source_t feedback_source = FEEDBACK_IMU;

/* =========================================================================
 * Default Configuration
 * ========================================================================= */

motor_driver_config_t motor_driver_default_config(void)
{
    /* Determine drive mode from Kconfig choice */
    drive_mode_t mode = DRIVE_MODE_FOUR_WHEEL;
#if defined(CONFIG_MOTOR_DRIVE_MODE_TWO_WHEEL)
    mode = DRIVE_MODE_TWO_WHEEL;
#elif defined(CONFIG_MOTOR_DRIVE_MODE_TRACKED)
    mode = DRIVE_MODE_TRACKED;
#endif

    motor_driver_config_t cfg = {
        .drive_mode = mode,
        .wheel_base_m = (float)CONFIG_MOTOR_WHEEL_BASE_MM / 1000.0f,
        .max_speed_ms = (float)CONFIG_MOTOR_MAX_SPEED_MMS / 1000.0f,
        .max_pwm_duty = (float)CONFIG_MOTOR_MAX_DUTY_PCT,
        .heading_pid = {
#ifdef CONFIG_PID_ENABLED
            .kp = (float)CONFIG_PID_KP / 100.0f,
            .ki = (float)CONFIG_PID_KI / 100.0f,
            .kd = (float)CONFIG_PID_KD / 100.0f,
            .integral_limit = (float)CONFIG_PID_INTEGRAL_LIMIT,
            .output_limit = (float)CONFIG_PID_OUTPUT_LIMIT
#else
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .output_limit = 0.0f
#endif
        },
#ifdef CONFIG_PID_ENABLED
        .pid_enabled = true
#else
        .pid_enabled = false
#endif
    };
    return cfg;
}

/* =========================================================================
 * PWM Configuration
 * ========================================================================= */

/**
 * @brief Configure PWM for a single motor
 * 
 * @param motor Motor ID (used to select LEDC channel)
 * @return ESP_OK on success
 */
static esp_err_t configure_motor_pwm(motor_id_t motor)
{
    /* Each motor gets its own LEDC channel */
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = motor_pins[motor].pwm_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)motor,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };

    esp_err_t err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel for motor %d: %s",
                 motor, esp_err_to_name(err));
        return err;
    }

    /* Configure direction pin as output */
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << motor_pins[motor].dir_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    err = gpio_config(&dir_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DIR pin for motor %d: %s",
                 motor, esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Set PWM duty and direction for a motor
 * 
 * @param motor Motor ID
 * @param speed Speed percentage (-100 to +100)
 */
static void set_motor_pwm(motor_id_t motor, int8_t speed)
{
    /* Clamp speed to valid range */
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    /* Set direction */
    bool forward = (speed >= 0);
    gpio_set_level(motor_pins[motor].dir_gpio, forward ? 1 : 0);

    /* Calculate duty cycle */
    int abs_speed = abs(speed);
    
    /* Apply maximum duty limit */
    float duty_pct = (float)abs_speed * (config.max_pwm_duty / 100.0f);
    uint32_t duty = (uint32_t)((duty_pct / 100.0f) * PWM_MAX_DUTY);

    if (duty > PWM_MAX_DUTY) {
        duty = PWM_MAX_DUTY;
    }

    /* Set PWM duty */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor);

    /* Store current speed */
    motor_speeds[motor] = speed;
}

/* =========================================================================
 * PID Controller
 * ========================================================================= */

/**
 * @brief Compute PID correction
 * 
 * Calculates heading correction based on difference between
 * desired and actual yaw rate from IMU.
 * 
 * @return Correction value to apply to left/right speed difference
 */
static float pid_compute(void)
{
    if (!config.pid_enabled || feedback_source == FEEDBACK_NONE) {
        return 0.0f;
    }

    TickType_t now = xTaskGetTickCount();
    float dt = (float)(now - pid_state.last_update) / configTICK_RATE_HZ;
    pid_state.last_update = now;

    /* Avoid division by zero or huge dt on first call */
    if (dt <= 0.0f || dt > 1.0f) {
        return 0.0f;
    }

    /* Error = desired yaw rate - actual yaw rate */
    float error = pid_state.desired_yaw_rate - pid_state.actual_yaw_rate;

    /* Proportional term */
    float p_term = config.heading_pid.kp * error;

    /* Integral term with anti-windup */
    pid_state.integral += error * dt;
    if (pid_state.integral > config.heading_pid.integral_limit) {
        pid_state.integral = config.heading_pid.integral_limit;
    } else if (pid_state.integral < -config.heading_pid.integral_limit) {
        pid_state.integral = -config.heading_pid.integral_limit;
    }
    float i_term = config.heading_pid.ki * pid_state.integral;

    /* Derivative term */
    float derivative = (error - pid_state.previous_error) / dt;
    float d_term = config.heading_pid.kd * derivative;
    pid_state.previous_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    if (output > config.heading_pid.output_limit) {
        output = config.heading_pid.output_limit;
    } else if (output < -config.heading_pid.output_limit) {
        output = -config.heading_pid.output_limit;
    }

    return output;
}

/**
 * @brief Reset PID state
 * 
 * Called when motors are stopped or command changes significantly.
 */
static void pid_reset(void)
{
    pid_state.integral = 0.0f;
    pid_state.previous_error = 0.0f;
    pid_state.last_update = xTaskGetTickCount();
}

/* =========================================================================
 * Skid-Steer Kinematics
 * ========================================================================= */

/**
 * @brief Apply velocity to motors using skid-steer kinematics
 * 
 * Converts linear/angular velocity to left/right side speeds,
 * then applies PID heading correction.
 * 
 * @param linear Linear velocity in m/s
 * @param angular Angular velocity in rad/s
 */
static void apply_skid_steer(float linear, float angular)
{
    /* Skid-steer kinematics:
     *   v_left  = linear - (angular * wheel_base / 2)
     *   v_right = linear + (angular * wheel_base / 2)
     */
    float half_base = config.wheel_base_m / 2.0f;
    float v_left  = linear - (angular * half_base);
    float v_right = linear + (angular * half_base);

    /* Store desired yaw rate for PID */
    pid_state.desired_yaw_rate = angular;

    /* Apply PID heading correction */
    float correction = pid_compute();
    v_left  -= correction;
    v_right += correction;

    /* Convert velocity (m/s) to speed percentage (-100 to +100) */
    float speed_left  = (v_left  / config.max_speed_ms) * 100.0f;
    float speed_right = (v_right / config.max_speed_ms) * 100.0f;

    /* Clamp to valid range */
    if (speed_left > 100.0f)  speed_left = 100.0f;
    if (speed_left < -100.0f) speed_left = -100.0f;
    if (speed_right > 100.0f)  speed_right = 100.0f;
    if (speed_right < -100.0f) speed_right = -100.0f;

    int8_t left  = (int8_t)speed_left;
    int8_t right = (int8_t)speed_right;

    /* Apply to motors based on drive mode */
    switch (config.drive_mode) {
        case DRIVE_MODE_TWO_WHEEL:
            /* Only rear motors driven, front free-rolling */
            set_motor_pwm(MOTOR_REAR_LEFT, left);
            set_motor_pwm(MOTOR_REAR_RIGHT, right);
            set_motor_pwm(MOTOR_FRONT_LEFT, 0);
            set_motor_pwm(MOTOR_FRONT_RIGHT, 0);
            break;

        case DRIVE_MODE_FOUR_WHEEL:
            /* All four motors driven independently */
            set_motor_pwm(MOTOR_FRONT_LEFT, left);
            set_motor_pwm(MOTOR_FRONT_RIGHT, right);
            set_motor_pwm(MOTOR_REAR_LEFT, left);
            set_motor_pwm(MOTOR_REAR_RIGHT, right);
            break;

        case DRIVE_MODE_TRACKED:
            /* Left and right pairs locked together */
            set_motor_pwm(MOTOR_FRONT_LEFT, left);
            set_motor_pwm(MOTOR_FRONT_RIGHT, right);
            set_motor_pwm(MOTOR_REAR_LEFT, left);
            set_motor_pwm(MOTOR_REAR_RIGHT, right);
            break;
    }

    ESP_LOGD(TAG, "Skid-steer: L=%d%% R=%d%% (PID correction=%.1f)",
             left, right, correction);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t motor_driver_init(const motor_driver_config_t *cfg)
{
    if (initialised) {
        ESP_LOGW(TAG, "Motor driver already initialised");
        return ESP_ERR_INVALID_STATE;
    }

    /* Store configuration */
    memcpy(&config, cfg, sizeof(motor_driver_config_t));

    /* Configure LEDC timer (shared by all channels) */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* Configure each motor */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        err = configure_motor_pwm((motor_id_t)i);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* Initialise PID state */
    pid_reset();
    feedback_source = config.pid_enabled ? FEEDBACK_IMU : FEEDBACK_NONE;

    initialised = true;

    ESP_LOGI(TAG, "Motor driver initialised (mode=%d, wheel_base=%.2fm, max=%.1fm/s)",
             config.drive_mode, config.wheel_base_m, config.max_speed_ms);

    return ESP_OK;
}

void motor_driver_set_velocity(float linear, float angular)
{
    if (!initialised) {
        ESP_LOGW(TAG, "Motor driver not initialised");
        return;
    }

    /* If both zero, stop cleanly */
    if (fabsf(linear) < 0.001f && fabsf(angular) < 0.001f) {
        motor_driver_stop();
        return;
    }

    apply_skid_steer(linear, angular);
}

void motor_driver_update_imu(float yaw_rate)
{
    pid_state.actual_yaw_rate = yaw_rate;
}

void motor_driver_set_motor(motor_id_t motor, int8_t speed)
{
    if (!initialised || motor >= MOTOR_COUNT) {
        return;
    }

    set_motor_pwm(motor, speed);
}

void motor_driver_set_drive_mode(drive_mode_t mode)
{
    config.drive_mode = mode;
    ESP_LOGI(TAG, "Drive mode changed to %d", mode);

    /* Stop motors during mode change */
    motor_driver_stop();
}

drive_mode_t motor_driver_get_drive_mode(void)
{
    return config.drive_mode;
}

void motor_driver_set_feedback(feedback_source_t source)
{
    feedback_source = source;
    pid_reset();
    ESP_LOGI(TAG, "Feedback source changed to %d", source);
}

void motor_driver_set_pid_params(const pid_params_t *params)
{
    memcpy(&config.heading_pid, params, sizeof(pid_params_t));
    pid_reset();
    ESP_LOGI(TAG, "PID params updated: Kp=%.2f Ki=%.2f Kd=%.2f",
             params->kp, params->ki, params->kd);
}

void motor_driver_stop(void)
{
    if (!initialised) {
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        set_motor_pwm((motor_id_t)i, 0);
    }

    pid_reset();
    ESP_LOGD(TAG, "All motors stopped");
}

void motor_driver_emergency_stop(void)
{
    /* Immediately kill all PWM regardless of init state */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
        motor_speeds[i] = 0;
    }

    pid_reset();
    ESP_LOGW(TAG, "EMERGENCY STOP");
}

int8_t motor_driver_get_speed(motor_id_t motor)
{
    if (motor >= MOTOR_COUNT) {
        return 0;
    }
    return motor_speeds[motor];
}

void motor_driver_encoder_update(motor_id_t motor, int32_t ticks)
{
    /* Future: process encoder feedback for closed-loop control */
    ESP_LOGD(TAG, "Encoder update motor %d: %ld ticks (not implemented)",
             motor, (long)ticks);
}
