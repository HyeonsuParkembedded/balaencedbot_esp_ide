/**
 * @file motor_control.h
 * @brief 모터 제어 인터페이스
 * 
 * DC 모터의 방향과 속도를 제어하는 기능을 제공합니다.
 * H-브리지 회로와 PWM을 사용하여 양방향 모터 제어를 구현합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "../bsw/gpio_driver.h"
#include "../bsw/pwm_driver.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 모터 제어 구조체
 * 
 * H-브리지를 통한 DC 모터 제어에 필요한 핀과 채널 정보를 담습니다.
 */
typedef struct {
    bsw_gpio_num_t motor_pin_a;      ///< 모터 제어 핀 A (방향 제어)
    bsw_gpio_num_t motor_pin_b;      ///< 모터 제어 핀 B (방향 제어)
    bsw_gpio_num_t enable_pin;       ///< PWM Enable 핀 (속도 제어)
    pwm_channel_t enable_channel; ///< PWM 채널
    int current_speed;            ///< 현재 모터 속도 (Soft Start용)
} motor_control_t;

/**
 * @brief 모터 제어 초기화
 * 
 * 모터 제어에 필요한 GPIO 핀들을 설정하고 PWM 타이머를 초기화합니다.
 * H-브리지의 방향 제어 핀과 속도 제어용 PWM 핀을 구성합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param pin_a 모터 제어 핀 A (방향 제어)
 * @param pin_b 모터 제어 핀 B (방향 제어)
 * @param enable_pin PWM Enable 핀 (속도 제어)
 * @param enable_ch PWM 채널 번호
 * @return esp_err_t 초기화 결과
 * @retval ESP_OK 성공
 * @retval ESP_FAIL 실패
 */
esp_err_t motor_control_init(motor_control_t* motor,
                            bsw_gpio_num_t pin_a, bsw_gpio_num_t pin_b,
                            bsw_gpio_num_t enable_pin, pwm_channel_t enable_ch);

/**
 * @brief 모터 속도 설정
 * 
 * 모터의 회전 방향과 속도를 설정합니다.
 * 양수값은 전진, 음수값은 후진을 의미합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param speed 모터 속도 (-255 ~ +255)
 *              - 양수: 전진 (pin_a=HIGH, pin_b=LOW)
 *              - 음수: 후진 (pin_a=LOW, pin_b=HIGH)
 *              - 0: 정지
 */
void motor_control_set_speed(motor_control_t* motor, int speed);

/**
 * @brief 모터 정지 (Coast 모드)
 *
 * 모터를 자유 회전 상태로 정지시킵니다.
 * TB6612FNG: IN1=LOW, IN2=LOW (Coast - 자유 회전)
 *
 * @param motor 모터 제어 구조체 포인터
 */
void motor_control_stop(motor_control_t* motor);

/**
 * @brief 모터 브레이크 (Short Brake 모드)
 *
 * 모터를 쇼트 브레이크로 급정지시킵니다.
 * TB6612FNG: IN1=HIGH, IN2=HIGH (Short Brake - 급제동)
 * Coast 모드보다 빠르게 정지하지만 모터에 부하가 큽니다.
 *
 * @param motor 모터 제어 구조체 포인터
 */
void motor_control_brake(motor_control_t* motor);

/**
 * @brief 전압 보상 모터 속도 설정
 * 
 * 배터리 전압 변동에 따라 모터 속도(PWM 듀티)를 보정하여 설정합니다.
 * 전압이 낮아지면 듀티를 높여 일정한 출력을 유지합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param speed 목표 속도 (-255 ~ +255)
 * @param current_voltage 현재 배터리 전압 (V)
 */
void motor_control_set_speed_compensated(motor_control_t* motor, int speed, float current_voltage);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
