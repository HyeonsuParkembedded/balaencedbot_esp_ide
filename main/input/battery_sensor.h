/**
 * @file battery_sensor.h
 * @brief 배터리 전압 측정 센서 헤더 파일
 * 
 * ADC를 사용하여 배터리 전압을 측정하고 모니터링하는 기능을 제공합니다.
 * 전압 분배 저항을 사용하여 실제 배터리 전압을 계산합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 1.0
 */

#ifndef BATTERY_SENSOR_H
#define BATTERY_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @enum battery_level_t
 * @brief 배터리 상태 레벨
 */
typedef enum {
    BATTERY_LEVEL_CRITICAL = 0, ///< 위험 수준 (즉시 충전 필요)
    BATTERY_LEVEL_LOW,          ///< 낮음 (충전 권장)
    BATTERY_LEVEL_NORMAL,       ///< 정상
    BATTERY_LEVEL_HIGH,         ///< 높음
    BATTERY_LEVEL_FULL          ///< 완충
} battery_level_t;

/**
 * @struct battery_sensor_t
 * @brief 배터리 센서 구조체
 */
typedef struct {
    adc_oneshot_unit_handle_t adc_handle;    ///< ADC 핸들
    adc_cali_handle_t adc_cali_handle;       ///< ADC 교정 핸들
    adc_channel_t channel;                   ///< ADC 채널
    float voltage_divider_ratio;             ///< 전압 분배비 (R1+R2)/R2
    float last_voltage;                      ///< 최근 측정 전압
    battery_level_t battery_level;           ///< 현재 배터리 수준
    bool initialized;                        ///< 초기화 플래그
} battery_sensor_t;

/**
 * @defgroup BATTERY_SENSOR_API 배터리 센서 API
 * @brief 배터리 전압 측정 함수들
 * @{
 */

/**
 * @brief 배터리 센서 초기화
 * 
 * ADC를 설정하고 전압 분배비를 계산하여 배터리 센서를 초기화합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @param channel ADC 채널
 * @param r1_kohm 전압분배 상단 저항 값 (kΩ)
 * @param r2_kohm 전압분배 하단 저항 값 (kΩ)
 * @return esp_err_t 초기화 결과
 */
esp_err_t battery_sensor_init(battery_sensor_t* battery, adc_channel_t channel, 
                             float r1_kohm, float r2_kohm);

/**
 * @brief 배터리 전압 측정
 * 
 * ADC를 사용하여 현재 배터리 전압을 측정합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @return float 측정된 배터리 전압 (V), 오류 시 -1.0
 */
float battery_sensor_read_voltage(battery_sensor_t* battery);

/**
 * @brief 배터리 레벨 확인
 * 
 * 현재 배터리 전압을 기준으로 배터리 레벨을 판단합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @return battery_level_t 현재 배터리 레벨
 */
battery_level_t battery_sensor_get_level(battery_sensor_t* battery);

/**
 * @brief 배터리 퍼센트 계산
 * 
 * 현재 배터리 전압을 기준으로 배터리 잔량을 퍼센트로 계산합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @return int 배터리 잔량 (0-100%), 오류 시 -1
 */
int battery_sensor_get_percentage(battery_sensor_t* battery);

/**
 * @brief 저전압 알림 확인
 * 
 * 배터리 전압이 저전압 임계값 이하인지 확인합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @return bool 저전압 상태 시 true
 */
bool battery_sensor_is_low(battery_sensor_t* battery);

/**
 * @brief 위험 전압 알림 확인
 * 
 * 배터리 전압이 위험 임계값 이하인지 확인합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @return bool 위험 상태 시 true
 */
bool battery_sensor_is_critical(battery_sensor_t* battery);

/**
 * @brief 배터리 센서 해제
 * 
 * ADC 리소스를 해제합니다.
 * 
 * @param battery 배터리 센서 구조체 포인터
 * @return esp_err_t 해제 결과
 */
esp_err_t battery_sensor_deinit(battery_sensor_t* battery);

/** @} */ // BATTERY_SENSOR_API

#ifdef __cplusplus
}
#endif

#endif // BATTERY_SENSOR_H