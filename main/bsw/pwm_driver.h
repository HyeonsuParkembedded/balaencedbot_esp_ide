/**
 * @file pwm_driver.h
 * @brief ESP32-C6 타이머 레지스터 직접 제어 PWM 드라이버 헤더 파일
 * 
 * 순수 비트연산과 레지스터 직접 조작을 통한 고성능 PWM 구현입니다.
 * HAL 드라이버 없이 하드웨어를 직접 제어하여 최고 성능을 달성합니다.
 * 
 * 구현 특징:
 * - ESP32-C6 타이머 레지스터 직접 제어
 * - GPIO 레지스터 비트연산 직접 조작
 * - HAL 의존성 완전 제거
 * - 1000단계 해상도 (0.1% 단위)
 * - 사용자 정의 주파수 지원
 * - 하드웨어 타이머 레지스터 ISR 기반 정밀 제어
 * - 최대 8채널 동시 지원
 * - 1μs 정밀도 비트연산 타이머
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 4.0 (비트연산 직접 제어 PWM)
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "esp_err.h"
#include "gpio_driver.h"
#include <stdint.h>
#include <stdbool.h>

// ESP32-C6 타이머 그룹 레지스터 주소 정의
#define TIMER_GROUP0_BASE_ADDR    0x60008000
#define TIMER_GROUP1_BASE_ADDR    0x60009000

// 타이머 레지스터 오프셋 (각 타이머는 0x24 바이트 간격)
#define TIMER_CONFIG_REG_OFFSET   0x0000
#define TIMER_LO_REG_OFFSET       0x0004
#define TIMER_HI_REG_OFFSET       0x0008
#define TIMER_UPDATE_REG_OFFSET   0x000C
#define TIMER_ALARMLO_REG_OFFSET  0x0010
#define TIMER_ALARMHI_REG_OFFSET  0x0014
#define TIMER_LOADLO_REG_OFFSET   0x0018
#define TIMER_LOADHI_REG_OFFSET   0x001C
#define TIMER_LOAD_REG_OFFSET     0x0020

// 타이머 인터럽트 레지스터 오프셋
#define TIMER_INT_ENA_REG_OFFSET  0x0098
#define TIMER_INT_RAW_REG_OFFSET  0x009C
#define TIMER_INT_ST_REG_OFFSET   0x00A0
#define TIMER_INT_CLR_REG_OFFSET  0x00A4

// 타이머 설정 레지스터 비트 필드 (unsigned to prevent undefined behavior)
#define TIMER_EN_BIT              (1U << 31)  ///< 타이머 활성화
#define TIMER_INCREASE_BIT        (1U << 30)  ///< 증가 모드
#define TIMER_AUTORELOAD_BIT      (1U << 29)  ///< 자동 리로드
#define TIMER_DIVIDER_SHIFT       13         ///< 분주기 시프트
#define TIMER_DIVIDER_MASK        0xFFFF     ///< 분주기 마스크
#define TIMER_EDGE_INT_EN_BIT     (1U << 12)  ///< 엣지 인터럽트 활성화
#define TIMER_LEVEL_INT_EN_BIT    (1U << 11)  ///< 레벨 인터럽트 활성화
#define TIMER_ALARM_EN_BIT        (1U << 10)  ///< 알람 활성화

// 타이머 레지스터 직접 액세스 매크로
#define TIMER_GROUP_REG_BASE(group)  ((group) == 0 ? TIMER_GROUP0_BASE_ADDR : TIMER_GROUP1_BASE_ADDR)
#define TIMER_REG_ADDR(group, timer, offset) (TIMER_GROUP_REG_BASE(group) + (timer * 0x24) + offset)

#define TIMER_WRITE_REG(group, timer, offset, value) \
    (*((volatile uint32_t*)TIMER_REG_ADDR(group, timer, offset)) = (value))

#define TIMER_READ_REG(group, timer, offset) \
    (*((volatile uint32_t*)TIMER_REG_ADDR(group, timer, offset)))

#define TIMER_SET_BITS(group, timer, offset, bits) \
    (*((volatile uint32_t*)TIMER_REG_ADDR(group, timer, offset)) |= (bits))

#define TIMER_CLEAR_BITS(group, timer, offset, bits) \
    (*((volatile uint32_t*)TIMER_REG_ADDR(group, timer, offset)) &= ~(bits))

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

/**
 * @brief PWM 채널 구조체
 */
typedef struct {
    bsw_gpio_num_t gpio_num;        ///< GPIO 핀 번호
    uint32_t frequency;             ///< PWM 주파수 (Hz)
    uint16_t duty_cycle;            ///< 듀티 사이클 (0-1000, 0.1% 단위)
    bool enabled;                   ///< 채널 활성화 상태
    uint32_t period_us;             ///< PWM 주기 (마이크로초)
    uint32_t high_time_us;          ///< HIGH 시간 (마이크로초)
    uint32_t low_time_us;           ///< LOW 시간 (마이크로초)
} pwm_channel_config_t;

/**
 * @brief 소프트웨어 PWM 설정 상수
 */
#define PWM_DEFAULT_FREQUENCY   5000    ///< 기본 PWM 주파수 (5kHz)
#define PWM_SERVO_FREQUENCY     50      ///< 서보 PWM 주파수 (50Hz)
#define PWM_RESOLUTION          1000    ///< PWM 해상도 (0-1000, 0.1% 단위)
#define PWM_TIMER_RESOLUTION    1000000 ///< 타이머 해상도 (1MHz = 1us)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup PWM_DRIVER GPIO 직접 제어 PWM 드라이버 API
 * @brief 소프트웨어 PWM 신호 생성 함수들
 * @{
 */

/**
 * @brief PWM 드라이버 전역 초기화
 * 
 * 하드웨어 타이머를 설정하고 소프트웨어 PWM 신호 생성을 위한 기본 설정을 수행합니다.
 * ESP32-C6의 고정밀 타이머를 사용하여 정확한 PWM 타이밍을 보장합니다.
 * 
 * 설정 파라미터:
 * - 타이머 해상도: 1MHz (1μs 정밀도)
 * - 기본 주파수: 5kHz (모터 제어에 적합)
 * - PWM 해상도: 1000단계 (0.1% 단위)
 * - 최대 8채널 지원
 * 
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note 모든 PWM 채널 사용 전에 먼저 호출해야 합니다.
 */
esp_err_t pwm_driver_init(void);

/**
 * @brief PWM 채널 초기화 (기본 5kHz)
 * 
 * 지정된 GPIO 핀과 PWM 채널을 연결하여 소프트웨어 PWM 출력을 설정합니다.
 * 
 * @param gpio PWM 출력할 GPIO 핀 번호
 * @param channel 사용할 PWM 채널 번호 (PWM_CHANNEL_0 ~ PWM_CHANNEL_7)
 * @return esp_err_t 
 *         - ESP_OK: 채널 설정 성공
 *         - ESP_FAIL: 채널 설정 실패
 * 
 * @note pwm_driver_init() 호출 후 사용해야 합니다.
 * @note 기본 5kHz 주파수로 설정됩니다.
 */
esp_err_t pwm_channel_init(bsw_gpio_num_t gpio, pwm_channel_t channel);

/**
 * @brief PWM 채널 초기화 (사용자 정의 주파수)
 * 
 * 지정된 GPIO 핀, PWM 채널, 주파수로 소프트웨어 PWM을 설정합니다.
 * 
 * @param gpio PWM 출력할 GPIO 핀 번호
 * @param channel 사용할 PWM 채널 번호 (PWM_CHANNEL_0 ~ PWM_CHANNEL_7)
 * @param frequency PWM 주파수 (Hz)
 * @return esp_err_t 
 *         - ESP_OK: 채널 설정 성공
 *         - ESP_FAIL: 채널 설정 실패
 */
esp_err_t pwm_channel_init_freq(bsw_gpio_num_t gpio, pwm_channel_t channel, uint32_t frequency);

/**
 * @brief PWM 듀티 사이클 설정
 * 
 * 지정된 채널의 PWM 듀티 사이클을 설정합니다.
 * 1000단계 해상도로 0.1% 단위 정밀한 제어가 가능합니다.
 * 
 * @param channel 제어할 PWM 채널 번호
 * @param duty 듀티 사이클 값 (0 ~ 1000, 0.1% 단위)
 *             - 0: 0% 듀티 (항상 LOW)
 *             - 500: 50% 듀티
 *             - 1000: 100% 듀티 (항상 HIGH)
 * @return esp_err_t 
 *         - ESP_OK: 듀티 설정 성공
 *         - ESP_FAIL: 듀티 설정 실패
 * 
 * @note 변경된 듀티는 다음 PWM 주기부터 적용됩니다.
 * @warning duty 값이 1000을 초과하면 1000으로 제한됩니다.
 */
esp_err_t pwm_set_duty(pwm_channel_t channel, uint32_t duty);

/**
 * @brief PWM 채널 활성화/비활성화
 * 
 * @param channel PWM 채널 번호
 * @param enable true: 활성화, false: 비활성화
 * @return esp_err_t 성공/실패
 */
esp_err_t pwm_set_enable(pwm_channel_t channel, bool enable);

/**
 * @brief 서보 모터용 PWM 채널 초기화
 * 
 * 서보 모터 제어에 특화된 소프트웨어 PWM 채널을 초기화합니다.
 * 50Hz 주파수와 0.1% 해상도를 사용합니다.
 * 
 * @param gpio PWM 출력할 GPIO 핀 번호
 * @param channel 사용할 PWM 채널 번호 (PWM_CHANNEL_0 ~ PWM_CHANNEL_7)
 * @return esp_err_t 
 *         - ESP_OK: 채널 설정 성공
 *         - ESP_FAIL: 채널 설정 실패
 * 
 * @note 50Hz 주파수로 자동 설정됩니다.
 */
esp_err_t pwm_servo_init(bsw_gpio_num_t gpio, pwm_channel_t channel);

/**
 * @brief 서보 모터 각도 설정 (편의 함수)
 * 
 * 서보 모터의 각도를 직접 설정합니다 (0-180도).
 * 표준 서보 모터 (1-2ms 펄스 폭)에 최적화되어 있습니다.
 * 
 * @param channel 제어할 PWM 채널 번호
 * @param angle 서보 각도 (0 ~ 180도)
 * @return esp_err_t 
 *         - ESP_OK: 각도 설정 성공
 *         - ESP_FAIL: 각도 설정 실패
 * 
 * @note 0도 = 1ms 펄스, 90도 = 1.5ms 펄스, 180도 = 2ms 펄스
 */
esp_err_t pwm_servo_set_angle(pwm_channel_t channel, uint16_t angle);

/**
 * @brief PWM 드라이버 해제
 * 
 * 모든 PWM 채널을 비활성화하고 타이머 자원을 해제합니다.
 * 
 * @return esp_err_t 성공/실패
 */
esp_err_t pwm_driver_deinit(void);

// === 비트연산 타이머 직접 제어 함수들 ===

/**
 * @brief 하드웨어 타이머 레지스터 직접 초기화
 * 
 * @param group 타이머 그룹 (0 또는 1)
 * @param timer 타이머 번호 (0 또는 1)
 * @param resolution_hz 타이머 해상도 (Hz)
 * @return esp_err_t 성공/실패
 */
esp_err_t pwm_timer_raw_init(uint32_t group, uint32_t timer, uint32_t resolution_hz);

/**
 * @brief 하드웨어 타이머 시작 (순수 레지스터 제어)
 * 
 * @param group 타이머 그룹
 * @param timer 타이머 번호
 */
void pwm_timer_raw_start(uint32_t group, uint32_t timer);

/**
 * @brief 하드웨어 타이머 정지 (순수 레지스터 제어)
 * 
 * @param group 타이머 그룹
 * @param timer 타이머 번호
 */
void pwm_timer_raw_stop(uint32_t group, uint32_t timer);

/**
 * @brief 타이머 카운터 값 읽기 (64비트)
 * 
 * @param group 타이머 그룹
 * @param timer 타이머 번호
 * @return uint64_t 현재 카운터 값
 */
uint64_t pwm_timer_raw_get_count(uint32_t group, uint32_t timer);

/**
 * @brief 타이머 알람 값 설정 (64비트)
 * 
 * @param group 타이머 그룹
 * @param timer 타이머 번호
 * @param alarm_value 알람 카운터 값
 */
void pwm_timer_raw_set_alarm(uint32_t group, uint32_t timer, uint64_t alarm_value);

/**
 * @brief 타이머 인터럽트 활성화/비활성화
 * 
 * @param group 타이머 그룹
 * @param timer 타이머 번호
 * @param enable true: 활성화, false: 비활성화
 */
void pwm_timer_raw_interrupt_enable(uint32_t group, uint32_t timer, bool enable);

/**
 * @brief 타이머 인터럽트 상태 클리어
 * 
 * @param group 타이머 그룹
 * @param timer 타이머 번호
 */
void pwm_timer_raw_interrupt_clear(uint32_t group, uint32_t timer);

/**
 * @brief 마이크로초 지연 (순수 비트연산)
 * 
 * @param us 지연 시간 (마이크로초)
 */
void pwm_delay_us_bitwise(uint32_t us);

/**
 * @brief 고정밀 마이크로초 타이밍 (CPU 사이클 기반)
 * 
 * @param start_time 시작 시간 포인터
 */
void pwm_timing_start(uint32_t* start_time);

/**
 * @brief 경과 시간 확인 (마이크로초)
 * 
 * @param start_time 시작 시간
 * @return uint32_t 경과 시간 (μs)
 */
uint32_t pwm_timing_elapsed_us(uint32_t start_time);

// === 순수 레지스터 조작 매크로들 ===

/**
 * @brief 빠른 GPIO PWM 출력 (인라인 비트연산)
 */
#define PWM_FAST_GPIO_HIGH(gpio) bsw_gpio_fast_set_high(gpio)
#define PWM_FAST_GPIO_LOW(gpio)  bsw_gpio_fast_set_low(gpio)
#define PWM_FAST_GPIO_TOGGLE(gpio) bsw_gpio_fast_toggle(gpio)

/**
 * @brief 타이머 레지스터 빠른 접근
 */
#define PWM_TIMER_FAST_START(group, timer) \
    TIMER_SET_BITS(group, timer, TIMER_CONFIG_REG_OFFSET, TIMER_EN_BIT)

#define PWM_TIMER_FAST_STOP(group, timer) \
    TIMER_CLEAR_BITS(group, timer, TIMER_CONFIG_REG_OFFSET, TIMER_EN_BIT)

#define PWM_TIMER_FAST_CLEAR_INT(group, timer) \
    TIMER_WRITE_REG(group, timer, TIMER_INT_CLR_REG_OFFSET, (1 << timer))

/** @} */ // PWM_DRIVER

#ifdef __cplusplus
}
#endif

#endif // PWM_DRIVER_H