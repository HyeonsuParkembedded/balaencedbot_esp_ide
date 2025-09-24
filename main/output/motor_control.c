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

#include "motor_control.h"
#include "../bsw/pwm_driver.h"
#include "../bsw/gpio_driver.h"
#include "../bsw/system_services.h"

static const char* MOTOR_TAG = "MOTOR_CONTROL";  ///< 로깅 태그

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

    if (speed > 0) {
        // 전진: A=HIGH, B=LOW
        bsw_gpio_set_level(motor->motor_pin_a, 1);
        bsw_gpio_set_level(motor->motor_pin_b, 0);
    } else if (speed < 0) {
        // 후진: A=LOW, B=HIGH
        bsw_gpio_set_level(motor->motor_pin_a, 0);
        bsw_gpio_set_level(motor->motor_pin_b, 1);
        speed = -speed;  // 절댓값으로 변환
    } else {
        // 정지: A=LOW, B=LOW (브레이크)
        bsw_gpio_set_level(motor->motor_pin_a, 0);
        bsw_gpio_set_level(motor->motor_pin_b, 0);
    }

    // PWM 듀티 사이클 설정으로 속도 제어
    pwm_set_duty(motor->enable_channel, speed);
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