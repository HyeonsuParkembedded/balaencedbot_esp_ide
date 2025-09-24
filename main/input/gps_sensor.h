/**
 * @file gps_sensor.h
 * @brief GPS 위성 위치 센서 드라이버 헤더 파일
 * 
 * UART 기반 GPS 모듈을 위한 드라이버입니다.
 * NMEA 0183 프로토콜을 파싱하여 위치, 고도, 위성 정보를 제공합니다.
 * 
 * 지원 기능:
 * - NMEA 문장 파싱 (GPGGA, GPRMC)
 * - 위도/경도 좌표 변환
 * - GPS Fix 상태 확인
 * - 위성 개수 모니터링
 * - 고도 정보 읽기
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#ifndef NATIVE_BUILD
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#else
typedef int esp_err_t;
typedef int uart_port_t;
typedef int gpio_num_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup GPS_SENSOR_STRUCTS GPS 센서 데이터 구조체
 * @brief GPS 센서 데이터 및 설정을 위한 구조체 정의
 * @{
 */

/**
 * @struct gps_data_t
 * @brief GPS 위치 데이터 구조체
 * 
 * GPS 모듈에서 파싱된 위치 정보와 상태를 저장합니다.
 */
typedef struct {
    double latitude;    ///< 위도 (도 단위, WGS84 좌표계)
    double longitude;   ///< 경도 (도 단위, WGS84 좌표계)
    float altitude;     ///< 고도 (미터 단위, 해수면 기준)
    int satellites;     ///< 사용 중인 위성 개수
    bool fix_valid;     ///< GPS Fix 유효성 (true: 유효한 위치)
    bool initialized;   ///< 센서 초기화 상태
} gps_data_t;

/**
 * @struct gps_sensor_t
 * @brief GPS 센서 제어 구조체
 * 
 * GPS 모듈의 UART 포트 정보와 위치 데이터를 관리합니다.
 */
typedef struct {
    uart_port_t uart_port; ///< UART 포트 번호
    gps_data_t data;       ///< GPS 위치 데이터
} gps_sensor_t;

/** @} */ // GPS_SENSOR_STRUCTS

/**
 * @defgroup GPS_SENSOR_API GPS 센서 API
 * @brief GPS 위성 위치 센서 제어 함수들
 * @{
 */

/**
 * @brief GPS 센서 초기화
 * 
 * GPS 모듈과의 UART 통신을 설정하고 센서를 초기화합니다.
 * 
 * @param gps GPS 센서 구조체 포인터
 * @param port UART 포트 번호
 * @param tx_pin UART TX 핀 번호
 * @param rx_pin UART RX 핀 번호
 * @param baudrate 통신 속도 (일반적으로 9600bps)
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 */
esp_err_t gps_sensor_init(gps_sensor_t* gps, uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baudrate);

/**
 * @brief GPS 데이터 업데이트
 * 
 * UART에서 NMEA 문장을 읽어와 파싱하여 위치 데이터를 업데이트합니다.
 * 
 * @param gps GPS 센서 구조체 포인터
 * @return esp_err_t 
 *         - ESP_OK: 업데이트 성공
 *         - ESP_FAIL: 업데이트 실패 (통신 오류 또는 파싱 실패)
 * 
 * @note 1Hz 주기로 호출하는 것을 권장합니다.
 */
esp_err_t gps_sensor_update(gps_sensor_t* gps);

/**
 * @brief 위도 읽기
 * @param gps GPS 센서 구조체 포인터
 * @return double 위도 (도 단위, -90 ~ +90)
 */
double gps_sensor_get_latitude(gps_sensor_t* gps);

/**
 * @brief 경도 읽기
 * @param gps GPS 센서 구조체 포인터
 * @return double 경도 (도 단위, -180 ~ +180)
 */
double gps_sensor_get_longitude(gps_sensor_t* gps);

/**
 * @brief 고도 읽기
 * @param gps GPS 센서 구조체 포인터
 * @return float 고도 (미터 단위, 해수면 기준)
 */
float gps_sensor_get_altitude(gps_sensor_t* gps);

/**
 * @brief 사용 중인 위성 개수 읽기
 * @param gps GPS 센서 구조체 포인터
 * @return int 위성 개수 (0 ~ 12개)
 */
int gps_sensor_get_satellites(gps_sensor_t* gps);

/**
 * @brief GPS Fix 상태 확인
 * @param gps GPS 센서 구조체 포인터
 * @return bool 
 *         - true: 유효한 GPS Fix (위치 정보 신뢰 가능)
 *         - false: GPS Fix 없음 (위치 정보 부정확)
 */
bool gps_sensor_has_fix(gps_sensor_t* gps);

/**
 * @brief GPS 센서 초기화 상태 확인
 * @param gps GPS 센서 구조체 포인터
 * @return bool 
 *         - true: 초기화 완료
 *         - false: 초기화 미완료 또는 오류
 */
bool gps_sensor_is_initialized(gps_sensor_t* gps);

/** @} */ // GPS_SENSOR_API

#ifdef __cplusplus
}
#endif

#endif // GPS_SENSOR_H