/**
 * @file pwm_driver.h
 * @brief PWM 신호 생성 드라이버 헤더 파일
 * 
 * ESP32-C6의 LEDC 컨트롤러를 사용한 PWM 신호 생성 드라이버입니다.
 * 모터 제어와 서보 모터 제어에 사용됩니다.
 * 
 * 지원 기능:
 * - 다중 채널 PWM 제어
 * - 가변 듀티 사이클 설정
 * - 고정밀 10비트 해상도
 * - 5kHz 주파수 (모터용)
 * - ESP32-C6 최적화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "esp_err.h"
#include "driver/gpio.h"

// BSW 추상화 계층 - PWM 채널 타입 정의
typedef enum {
    PWM_CHANNEL_0 = 0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2, 
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
    PWM_CHANNEL_5,
    PWM_CHANNEL_6,
    PWM_CHANNEL_7,
    PWM_CHANNEL_MAX
} pwm_channel_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup PWM_DRIVER PWM 드라이버 API
 * @brief LEDC 기반 PWM 신호 생성 함수들
 * @{
 */

/**
 * @brief PWM 드라이버 전역 초기화
 * 
 * LEDC 타이머를 설정하고 PWM 신호 생성을 위한 기본 설정을 수행합니다.
 * ESP32-C6에 최적화된 설정을 적용합니다.
 * 
 * 설정 파라미터:
 * - 주파수: 5kHz (모터 제어에 적합)
 * - 해상도: 10비트 (0-1023)
 * - 타이머: LEDC_TIMER_0
 * - 모드: LEDC_LOW_SPEED_MODE
 * - 클록: LEDC_AUTO_CLK (ESP32-C6 자동 선택)
 * 
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note 모든 PWM 채널 사용 전에 먼저 호출해야 합니다.
 */
esp_err_t pwm_driver_init(void);

/**
 * @brief PWM 채널 초기화
 * 
 * 지정된 GPIO 핀과 PWM 채널을 연결하여 PWM 출력을 설정합니다.
 * 
 * @param gpio PWM 출력할 GPIO 핀 번호
 * @param channel 사용할 PWM 채널 번호 (PWM_CHANNEL_0 ~ PWM_CHANNEL_7)
 * @return esp_err_t 
 *         - ESP_OK: 채널 설정 성공
 *         - ESP_FAIL: 채널 설정 실패
 * 
 * @note pwm_driver_init() 호출 후 사용해야 합니다.
 * @warning 이미 사용 중인 채널을 중복 설정하면 오류가 발생할 수 있습니다.
 */
esp_err_t pwm_channel_init(gpio_num_t gpio, pwm_channel_t channel);

/**
 * @brief PWM 듀티 사이클 설정
 * 
 * 지정된 채널의 PWM 듀티 사이클을 설정합니다.
 * 10비트 해상도로 정밀한 제어가 가능합니다.
 * 
 * @param channel 제어할 PWM 채널 번호
 * @param duty 듀티 사이클 값 (0 ~ 1023, 10비트 해상도)
 *             - 0: 0% 듀티 (항상 LOW)
 *             - 512: 50% 듀티
 *             - 1023: 100% 듀티 (항상 HIGH)
 * @return esp_err_t 
 *         - ESP_OK: 듀티 설정 성공
 *         - ESP_FAIL: 듀티 설정 실패
 * 
 * @note 변경된 듀티는 즉시 적용됩니다.
 * @warning duty 값이 1023을 초과하면 1023으로 제한됩니다.
 */
esp_err_t pwm_set_duty(pwm_channel_t channel, uint32_t duty);

/**
 * @brief 서보 모터용 PWM 채널 초기화
 * 
 * 서보 모터 제어에 특화된 PWM 채널을 초기화합니다.
 * 50Hz 주파수와 14비트 해상도를 사용합니다.
 * 
 * @param gpio PWM 출력할 GPIO 핀 번호
 * @param channel 사용할 PWM 채널 번호 (PWM_CHANNEL_0 ~ PWM_CHANNEL_7)
 * @return esp_err_t 
 *         - ESP_OK: 채널 설정 성공
 *         - ESP_FAIL: 채널 설정 실패
 * 
 * @note 서보 모터 전용 타이머(LEDC_TIMER_1)를 사용합니다.
 */
esp_err_t pwm_servo_init(gpio_num_t gpio, pwm_channel_t channel);

/**
 * @brief 서보 모터용 PWM 듀티 사이클 설정
 * 
 * 서보 모터 제어용 14비트 해상도로 듀티 사이클을 설정합니다.
 * 
 * @param channel 제어할 PWM 채널 번호
 * @param duty 듀티 사이클 값 (0 ~ 16383, 14비트 해상도)
 * @return esp_err_t 
 *         - ESP_OK: 듀티 설정 성공
 *         - ESP_FAIL: 듀티 설정 실패
 */
esp_err_t pwm_servo_set_duty(pwm_channel_t channel, uint32_t duty);

/** @} */ // PWM_DRIVER

#ifdef __cplusplus
}
#endif

#endif // PWM_DRIVER_H