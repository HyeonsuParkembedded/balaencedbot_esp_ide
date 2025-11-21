/**
 * @file motor_control.c
 * @brief 모터 제어 구현
 * 
 * H-브리지와 PWM을 사용한 DC 모터 제어 기능을 구현합니다.
 * 방향 제어를 위한 GPIO와 속도 제어를 위한 PWM을 조합하여 사용합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 1.0
 */

#include <stdbool.h>
#include <stdlib.h>

#include "motor_control.h"
#include "../bsw/pwm_driver.h"
#include "../bsw/gpio_driver.h"
#include "../bsw/system_services.h"
#include "../config.h" // For CONFIG_MOTOR_STBY_PIN

static const char* MOTOR_TAG = "MOTOR_CONTROL";  ///< 로깅 태그
static bool s_direction_hint_logged = false;

/**
 * @brief 모터 제어 초기화 구현
 * 
 * H-브리지 제어용 GPIO 핀들과 PWM 채널을 초기화합니다.
 * 방향 제어 핀은 디지털 출력으로, 속도 제어 핀은 PWM으로 설정됩니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param pin_a 모터 제어 핀 A
 * @param pin_b 모터 제어 핀 B
 * @param enable_pin PWM Enable 핀
 * @param enable_ch PWM 채널 번호
 * @return esp_err_t 초기화 결과
 */
esp_err_t motor_control_init(motor_control_t* motor,
                            bsw_gpio_num_t pin_a, bsw_gpio_num_t pin_b,
                            bsw_gpio_num_t enable_pin, pwm_channel_t enable_ch) {
    // 구조체 멤버 초기화
    motor->motor_pin_a = pin_a;
    motor->motor_pin_b = pin_b;
    motor->enable_pin = enable_pin;
    motor->enable_channel = enable_ch;
    motor->current_speed = 0; // Initialize current speed

    // PWM 드라이버 초기화
    esp_err_t ret = pwm_driver_init();
    if (ret != ESP_OK) {
        return ret;
    }

    // 모터 제어 핀 설정 (디지털 출력)
    bsw_gpio_config_t motor_config = {
        .pin_bit_mask = (1ULL << pin_a) | (1ULL << pin_b),
        .mode = BSW_GPIO_MODE_OUTPUT,
        .pull_up_en = BSW_GPIO_PULLUP_DISABLE,
        .pull_down_en = BSW_GPIO_PULLDOWN_DISABLE,
        .intr_type = BSW_GPIO_INTR_DISABLE,
    };
    ret = bsw_gpio_config(&motor_config);
    if (ret != ESP_OK) {
        BSW_LOGE(MOTOR_TAG, "Failed to configure motor GPIO");
        return ret;
    }

    // STBY 핀 활성화 (TB6612FNG)
    #ifdef CONFIG_MOTOR_STBY_PIN
    bsw_gpio_config_pin(CONFIG_MOTOR_STBY_PIN, BSW_GPIO_MODE_OUTPUT, BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    bsw_gpio_set_level(CONFIG_MOTOR_STBY_PIN, 1); // Driver Enable
    #endif

    // PWM 채널 초기화 (속도 제어용)
    ret = pwm_channel_init(enable_pin, enable_ch);
    if (ret != ESP_OK) {
        return ret;
    }

    BSW_LOGI(MOTOR_TAG, "Motor control initialized");
    return ESP_OK;
}

/**
 * @brief 모터 속도 설정 구현
 * 
 * 모터의 회전 방향과 속도를 제어합니다.
 * H-브리지의 A, B 핀으로 방향을 제어하고 PWM으로 속도를 제어합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param speed 모터 속도 (-255 ~ +255)
 */
void motor_control_set_speed(motor_control_t* motor, int speed) {
    // 속도 범위 제한
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // Update current speed state
    motor->current_speed = speed;

    const int deadzone = CONFIG_MOTOR_DEADZONE;

    if (deadzone > 0) {
        if (speed > 0) {
            speed += deadzone;
            if (speed > 255) speed = 255;
        } else if (speed < 0) {
            speed -= deadzone;
            if (speed < -255) speed = -255;
        }
    }

    // 입력(255) -> PWM(1000) 스케일링
    int abs_speed = (speed > 0) ? speed : -speed;
    uint32_t pwm_duty = (abs_speed * 1000) / 255;

    if (!s_direction_hint_logged && abs_speed >= 100) {
        BSW_LOGI(MOTOR_TAG,
                 "Direction check: speed=%d -> pinA=%d HIGH / pinB=%d LOW for forward motion. Hold the robot and confirm wheel direction.",
                 speed, motor->motor_pin_a, motor->motor_pin_b);
        s_direction_hint_logged = true;
    }

    if (speed > 0) {
        // 전진: A=HIGH, B=LOW
        bsw_gpio_set_level(motor->motor_pin_a, 1);
        bsw_gpio_set_level(motor->motor_pin_b, 0);
    } else if (speed < 0) {
        // 후진: A=LOW, B=HIGH
        bsw_gpio_set_level(motor->motor_pin_a, 0);
        bsw_gpio_set_level(motor->motor_pin_b, 1);
    } else {
        // 정지: A=LOW, B=LOW (브레이크)
        bsw_gpio_set_level(motor->motor_pin_a, 0);
        bsw_gpio_set_level(motor->motor_pin_b, 0);
        pwm_duty = 0;
    }

    // PWM 듀티 사이클 설정으로 속도 제어
    pwm_set_duty(motor->enable_channel, pwm_duty);
}

/**
 * @brief 모터 정지 구현
 * 
 * 모터를 즉시 정지시킵니다.
 * motor_control_set_speed(0)을 호출하여 구현합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 */
void motor_control_stop(motor_control_t* motor) {
    motor_control_set_speed(motor, 0);
}

/**
 * @brief 전압 보상 모터 속도 설정 구현
 * 
 * 현재 배터리 전압을 기반으로 PWM 듀티를 보정하여 모터 속도를 설정합니다.
 * 기준 전압(12V) 대비 현재 전압 비율로 속도 명령을 스케일링합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param speed 목표 속도 (-255 ~ +255)
 * @param current_voltage 현재 배터리 전압 (V)
 */
void motor_control_set_speed_compensated(motor_control_t* motor, int speed, float current_voltage) {
    const float NOMINAL_VOLTAGE = 12.0f; // 기준 전압 (12V)
    int target_speed = speed;

    if (current_voltage > 0.0f && current_voltage <= CONFIG_BATTERY_CRITICAL_THRESHOLD) {
        BSW_LOGW(MOTOR_TAG, "Battery %.2fV below cutoff %.2fV. Motor disabled.",
                 current_voltage, CONFIG_BATTERY_CRITICAL_THRESHOLD);
        motor_control_stop(motor);
        return;
    }
    
    // 전압이 너무 낮으면 보상하지 않음 (0으로 나눔 방지 및 배터리 보호)
    if (current_voltage >= 6.0f) {
        // 전압이 낮을수록 듀티를 더 높여서 보상
        float voltage_factor = NOMINAL_VOLTAGE / current_voltage;
        if (voltage_factor < 1.0f) {
            voltage_factor = 1.0f;
        } else if (voltage_factor > CONFIG_BATTERY_COMPENSATION_MAX_GAIN) {
            voltage_factor = CONFIG_BATTERY_COMPENSATION_MAX_GAIN;
        }
        target_speed = (int)(speed * voltage_factor);
    }
    
    // 255 제한 (Clamp)
    if (target_speed > 255) target_speed = 255;
    if (target_speed < -255) target_speed = -255;
    
    motor_control_set_speed(motor, target_speed);
}