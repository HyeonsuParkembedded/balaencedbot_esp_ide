/**
 * @file encoder_sensor.h
 * @brief 로터리 엔코더 센서 드라이버 헤더 파일
 * 
 * 모터 회전을 감지하는 로터리 엔코더 센서 드라이버입니다.
 * Quadrature 엔코더(A/B 상)를 사용하여 모터의 위치, 거리, 속도를 측정합니다.
 * 
 * 주요 기능:
 * - A/B 상 신호 처리 (Quadrature decoding)
 * - 회전 위치 카운팅
 * - 이동 거리 계산
 * - 회전 속도 측정
 * - 인터럽트 기반 실시간 처리
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef ENCODER_SENSOR_H
#define ENCODER_SENSOR_H

#ifndef NATIVE_BUILD
#include "driver/gpio.h"
#include "esp_err.h"
#else
typedef int esp_err_t;
typedef int gpio_num_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define IRAM_ATTR
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct encoder_sensor_t
 * @brief 엔코더 센서 제어 구조체
 * 
 * 로터리 엔코더의 하드웨어 설정과 측정 데이터를 관리합니다.
 */
typedef struct {
    gpio_num_t encoder_pin_a;    ///< A상 신호 GPIO 핀
    gpio_num_t encoder_pin_b;    ///< B상 신호 GPIO 핀
    volatile int32_t encoder_count; ///< 현재 엔코더 카운트 (인터럽트에서 업데이트)
    volatile int last_encoded;   ///< 마지막 인코딩 상태 (A/B 조합)
    int ppr;                     ///< 회전당 펄스 수 (Pulses Per Revolution)
    float wheel_diameter;        ///< 바퀴 직경 (cm)
    uint32_t last_time;          ///< 마지막 속도 계산 시간 (ms)
    int32_t last_position;       ///< 마지막 위치 (속도 계산용)
    float current_speed;         ///< 현재 속도 (cm/s)
} encoder_sensor_t;

/**
 * @defgroup ENCODER_SENSOR_API 엔코더 센서 API
 * @brief 로터리 엔코더 센서 제어 함수들
 * @{
 */

/**
 * @brief 엔코더 센서 초기화
 * 
 * GPIO 핀을 설정하고 인터럽트를 등록하여 엔코더 신호를 감지합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @param pin_a A상 신호 GPIO 핀
 * @param pin_b B상 신호 GPIO 핀
 * @param pulses_per_rev 회전당 펄스 수 (일반적으로 360)
 * @param wheel_diam 바퀴 직경 (cm)
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 */
esp_err_t encoder_sensor_init(encoder_sensor_t* encoder,
                             gpio_num_t pin_a, gpio_num_t pin_b,
                             int pulses_per_rev, float wheel_diam);

/**
 * @brief 엔코더 카운트 리셋
 * 
 * 엔코더 카운트를 0으로 재설정하고 속도 측정을 초기화합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 */
void encoder_sensor_reset(encoder_sensor_t* encoder);

/**
 * @brief 엔코더 위치 읽기
 * 
 * 현재 엔코더 카운트 값을 반환합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return int32_t 현재 엔코더 카운트 (펄스 단위)
 *         - 양수: 정방향 회전
 *         - 음수: 역방향 회전
 */
int32_t encoder_sensor_get_position(const encoder_sensor_t* encoder);

/**
 * @brief 이동 거리 계산
 * 
 * 엔코더 카운트를 바퀴 둘레를 이용해 실제 이동 거리로 변환합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return float 이동 거리 (cm)
 *         - 양수: 전진
 *         - 음수: 후진
 */
float encoder_sensor_get_distance(const encoder_sensor_t* encoder);

/**
 * @brief 현재 속도 읽기
 * 
 * 가장 최근에 계산된 속도 값을 반환합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return float 현재 속도 (cm/s)
 *         - 양수: 전진 속도
 *         - 음수: 후진 속도
 */
float encoder_sensor_get_speed(const encoder_sensor_t* encoder);

/**
 * @brief 속도 계산 업데이트
 * 
 * 현재 위치와 시간을 기반으로 속도를 계산하여 업데이트합니다.
 * 주기적으로 호출해야 정확한 속도 측정이 가능합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return esp_err_t 
 *         - ESP_OK: 업데이트 성공
 *         - ESP_FAIL: 업데이트 실패
 * 
 * @note 센서 태스크에서 50Hz 주기로 호출하는 것을 권장합니다.
 */
esp_err_t encoder_sensor_update_speed(encoder_sensor_t* encoder);

/** @} */ // ENCODER_SENSOR_API

#ifdef __cplusplus
}
#endif

#endif // ENCODER_SENSOR_H