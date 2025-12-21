/**
 * @file motor_control.c
 * @brief TB6612FNG 모터 드라이버 제어 구현
 *
 * TB6612FNG 듀얼 H-브리지 모터 드라이버와 PWM을 사용한 TT 모터 제어 기능을 구현합니다.
 *
 * TB6612FNG 특징:
 * - 듀얼 채널 (2개 모터 독립 제어)
 * - 1.2A 연속, 3.2A 피크 전류
 * - PWM 주파수: 최대 100kHz (TT 모터 최적: 20kHz)
 * - STBY 핀: LOW일 때 저전력 대기 모드
 *
 * 제어 로직 (각 채널):
 * - 전진: IN1=HIGH, IN2=LOW, PWM=듀티
 * - 후진: IN1=LOW, IN2=HIGH, PWM=듀티
 * - 쇼트 브레이크: IN1=HIGH, IN2=HIGH, PWM=듀티
 * - 정지 (Coast): IN1=LOW, IN2=LOW
 *
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0 (TB6612FNG 최적화)
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

    // TB6612FNG STBY 핀 활성화 (필수)
    #ifdef CONFIG_MOTOR_STBY_PIN
    bsw_gpio_config_pin(CONFIG_MOTOR_STBY_PIN, BSW_GPIO_MODE_OUTPUT, BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    bsw_gpio_set_level(CONFIG_MOTOR_STBY_PIN, 1); // TB6612FNG 드라이버 활성화
    BSW_LOGI(MOTOR_TAG, "TB6612FNG STBY pin enabled (GPIO%d)", CONFIG_MOTOR_STBY_PIN);
    #else
    BSW_LOGW(MOTOR_TAG, "TB6612FNG STBY pin not configured - driver may not work!");
    #endif

    // PWM 채널 초기화 (TT 모터 최적 주파수: 20kHz)
    #ifdef CONFIG_MOTOR_PWM_FREQUENCY
    ret = pwm_channel_init_freq(enable_pin, enable_ch, CONFIG_MOTOR_PWM_FREQUENCY);
    #else
    ret = pwm_channel_init_freq(enable_pin, enable_ch, 20000); // 기본 20kHz
    #endif
    if (ret != ESP_OK) {
        BSW_LOGE(MOTOR_TAG, "Failed to initialize PWM channel");
        return ret;
    }

    BSW_LOGI(MOTOR_TAG, "TB6612FNG motor control initialized (PWM: %dHz)",
             CONFIG_MOTOR_PWM_FREQUENCY);
    return ESP_OK;
}

/**
 * @brief TB6612FNG 모터 속도 설정 구현
 *
 * TB6612FNG 모터 드라이버를 통해 TT 모터의 회전 방향과 속도를 제어합니다.
 * IN1/IN2 핀으로 방향을 제어하고 PWM으로 속도를 제어합니다.
 *
 * @param motor 모터 제어 구조체 포인터
 * @param speed 모터 속도 (-255 ~ +255)
 */
void motor_control_set_speed(motor_control_t* motor, int speed) {
    // 속도 범위 제한
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // 현재 속도 상태 업데이트
    motor->current_speed = speed;

    // TT 모터 데드존 보상 (비례 스케일링 방식)
    // 입력 1~255 -> 출력 deadzone~255 (선형 변환)
    // 이 방식은 저속에서 모터를 시작시키면서 고속에서 최대 출력을 보장합니다.
    const int deadzone = CONFIG_MOTOR_DEADZONE;
    int compensated_speed = speed;

    if (deadzone > 0 && speed != 0) {
        int abs_speed = (speed > 0) ? speed : -speed;
        // 비례 스케일링: 1~255 입력을 deadzone~255 출력으로 변환
        // 공식: output = deadzone + (input * (255 - deadzone) / 255)
        int mapped_speed = deadzone + ((abs_speed * (255 - deadzone)) / 255);
        compensated_speed = (speed > 0) ? mapped_speed : -mapped_speed;
    }

    // PWM 듀티 변환 (0-255 -> 0-1000)
    int abs_speed = (compensated_speed > 0) ? compensated_speed : -compensated_speed;
    uint32_t pwm_duty = (abs_speed * 1000) / 255;

    // 방향 확인 로그 (최초 1회만)
    if (!s_direction_hint_logged && abs_speed >= 100) {
        BSW_LOGI(MOTOR_TAG,
                 "TB6612FNG: speed=%d -> IN1(GPIO%d)=HIGH, IN2(GPIO%d)=LOW for forward. Verify motor direction.",
                 speed, motor->motor_pin_a, motor->motor_pin_b);
        s_direction_hint_logged = true;
    }

    // TB6612FNG 제어 로직
    if (compensated_speed > 0) {
        // 전진: IN1=HIGH, IN2=LOW, PWM=듀티
        bsw_gpio_set_level(motor->motor_pin_a, 1);
        bsw_gpio_set_level(motor->motor_pin_b, 0);
    } else if (compensated_speed < 0) {
        // 후진: IN1=LOW, IN2=HIGH, PWM=듀티
        bsw_gpio_set_level(motor->motor_pin_a, 0);
        bsw_gpio_set_level(motor->motor_pin_b, 1);
    } else {
        // 정지 (Coast): IN1=LOW, IN2=LOW (자유 회전)
        bsw_gpio_set_level(motor->motor_pin_a, 0);
        bsw_gpio_set_level(motor->motor_pin_b, 0);
        pwm_duty = 0;
    }

    // PWM 듀티 사이클 설정으로 속도 제어
    pwm_set_duty(motor->enable_channel, pwm_duty);
}

/**
 * @brief 모터 정지 구현 (Coast 모드)
 *
 * 모터를 자유 회전 상태로 정지시킵니다.
 * TB6612FNG: IN1=LOW, IN2=LOW (Coast - 자유 회전)
 *
 * @param motor 모터 제어 구조체 포인터
 */
void motor_control_stop(motor_control_t* motor) {
    motor_control_set_speed(motor, 0);
}

/**
 * @brief 모터 브레이크 구현 (Short Brake 모드)
 *
 * 모터를 쇼트 브레이크로 급정지시킵니다.
 * TB6612FNG: IN1=HIGH, IN2=HIGH (Short Brake - 급제동)
 * Coast 모드보다 빠르게 정지하지만 모터에 부하가 큽니다.
 *
 * @param motor 모터 제어 구조체 포인터
 */
void motor_control_brake(motor_control_t* motor) {
    // Short Brake: IN1=HIGH, IN2=HIGH
    bsw_gpio_set_level(motor->motor_pin_a, 1);
    bsw_gpio_set_level(motor->motor_pin_b, 1);

    // PWM을 최대로 설정하여 브레이크 효과 강화
    pwm_set_duty(motor->enable_channel, 1000);

    // 현재 속도 상태 초기화
    motor->current_speed = 0;
}

/**
 * @brief 전압 보상 모터 속도 설정 구현 (최대 토크 보장)
 *
 * 배터리 전압 변동에도 일정한 토크를 유지하도록 PWM 듀티를 자동 보정합니다.
 * 기준 전압(8.4V) 대비 현재 전압 비율로 속도 명령을 스케일링합니다.
 *
 * 동작 원리 (2S 리튬 배터리 기준):
 * - 배터리 전압 8.4V (만충): 보정 없음 (1.0배)
 * - 배터리 전압 7.4V (공칭): PWM 듀티 1.14배 증가
 * - 배터리 전압 6.0V (저전압): PWM 듀티 1.40배 증가
 * - 최대 보상: 1.5배로 제한 (모터 보호)
 *
 * 이를 통해 배터리 방전 시에도 일정한 토크를 유지하면서 모터를 보호합니다.
 *
 * @param motor 모터 제어 구조체 포인터
 * @param speed 목표 속도 (-255 ~ +255)
 * @param current_voltage 현재 배터리 전압 (V)
 */
void motor_control_set_speed_compensated(motor_control_t* motor, int speed, float current_voltage) {
    const float NOMINAL_VOLTAGE = 8.4f; // 2S 리튬 배터리 만충 전압 (8.4V)
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