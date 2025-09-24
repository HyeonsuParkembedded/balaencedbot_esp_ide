/**
 * @file pwm_driver.h
 * @brief PWM 신호 생성 드라이버 헤더 파일
 * 
 * ESP32-S3의 LEDC 컨트롤러를 사용한 PWM 신호 생성 드라이버입니다.
 * 모터 제어와 서보 모터 제어에 사용됩니다.
 * 
 * 지원 기능:
 * - 다중 채널 PWM 제어
 * - 가변 듀티 사이클 설정
 * - 고정밀 13비트 해상도
 * - 5kHz 주파수 (모터용)
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#ifndef NATIVE_BUILD
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#else
typedef int esp_err_t;
typedef int ledc_channel_t;
typedef int gpio_num_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif

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
 * 
 * 설정 파라미터:
 * - 주파수: 5kHz (모터 제어에 적합)
 * - 해상도: 13비트 (0-8191)
 * - 타이머: LEDC_TIMER_0
 * - 모드: LEDC_LOW_SPEED_MODE
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
 * 지정된 GPIO 핀과 LEDC 채널을 연결하여 PWM 출력을 설정합니다.
 * 
 * @param gpio PWM 출력할 GPIO 핀 번호
 * @param channel 사용할 LEDC 채널 번호 (LEDC_CHANNEL_0 ~ LEDC_CHANNEL_7)
 * @return esp_err_t 
 *         - ESP_OK: 채널 설정 성공
 *         - ESP_FAIL: 채널 설정 실패
 * 
 * @note pwm_driver_init() 호출 후 사용해야 합니다.
 * @warning 이미 사용 중인 채널을 중복 설정하면 오류가 발생할 수 있습니다.
 */
esp_err_t pwm_channel_init(gpio_num_t gpio, ledc_channel_t channel);

/**
 * @brief PWM 듀티 사이클 설정
 * 
 * 지정된 채널의 PWM 듀티 사이클을 설정합니다.
 * 
 * @param channel 제어할 LEDC 채널 번호
 * @param duty 듀티 사이클 값 (0 ~ 8191, 13비트 해상도)
 *             - 0: 0% 듀티 (항상 LOW)
 *             - 4095: 50% 듀티
 *             - 8191: 100% 듀티 (항상 HIGH)
 * @return esp_err_t 
 *         - ESP_OK: 듀티 설정 성공
 *         - ESP_FAIL: 듀티 설정 실패
 * 
 * @note 변경된 듀티는 즉시 적용됩니다.
 * @warning duty 값이 8191을 초과하면 8191로 제한됩니다.
 */
esp_err_t pwm_set_duty(ledc_channel_t channel, uint32_t duty);

/** @} */ // PWM_DRIVER

#ifdef __cplusplus
}
#endif

#endif // PWM_DRIVER_H