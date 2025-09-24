/**
 * @file motor_control.h
 * @brief 모터 제어 인터페이스
 * 
 * DC 모터의 방향과 속도를 제어하는 기능을 제공합니다.
 * H-브리지 회로와 PWM을 사용하여 양방향 모터 제어를 구현합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifndef NATIVE_BUILD
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#else
typedef int esp_err_t;
#ifndef GPIO_NUM_T_DEFINED
typedef int gpio_num_t;
#endif
#ifndef LEDC_CHANNEL_T_DEFINED
typedef int ledc_channel_t;
#endif
#define ESP_OK 0
#define ESP_FAIL -1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 모터 제어 구조체
 * 
 * H-브리지를 통한 DC 모터 제어에 필요한 핀과 채널 정보를 담습니다.
 */
typedef struct {
    gpio_num_t motor_pin_a;      ///< 모터 제어 핀 A (방향 제어)
    gpio_num_t motor_pin_b;      ///< 모터 제어 핀 B (방향 제어)
    gpio_num_t enable_pin;       ///< PWM Enable 핀 (속도 제어)
    ledc_channel_t enable_channel; ///< PWM 채널
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
                            gpio_num_t pin_a, gpio_num_t pin_b,
                            gpio_num_t enable_pin, ledc_channel_t enable_ch);

/**
 * @brief 모터 속도 설정
 * 
 * 모터의 회전 방향과 속도를 설정합니다.
 * 양수값은 전진, 음수값은 후진을 의미합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 * @param speed 모터 속도 (-100 ~ +100)
 *              - 양수: 전진 (pin_a=HIGH, pin_b=LOW)
 *              - 음수: 후진 (pin_a=LOW, pin_b=HIGH)
 *              - 0: 정지
 */
void motor_control_set_speed(motor_control_t* motor, int speed);

/**
 * @brief 모터 정지
 * 
 * 모터를 즉시 정지시킵니다.
 * 모든 제어 핀을 LOW로 설정하여 브레이크 효과를 제공합니다.
 * 
 * @param motor 모터 제어 구조체 포인터
 */
void motor_control_stop(motor_control_t* motor);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H