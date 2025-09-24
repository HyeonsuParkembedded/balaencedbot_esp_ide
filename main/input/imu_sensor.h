/**
 * @file imu_sensor.h
 * @brief MPU6050 IMU 센서 드라이버 헤더 파일
 * 
 * MPU6050 6축 관성 측정 장치(IMU) 센서를 위한 드라이버입니다.
 * 가속도계와 자이로스코프 데이터를 읽어 피치, 롤 각도를 계산합니다.
 * 
 * 주요 기능:
 * - MPU6050 초기화 및 설정
 * - 가속도계 데이터 읽기 (3축)
 * - 자이로스코프 데이터 읽기 (3축)
 * - 피치/롤 각도 계산
 * - 데이터 유효성 검증
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#ifndef NATIVE_BUILD
#include "driver/i2c.h"
#include "esp_err.h"
#else
typedef int esp_err_t;
typedef int i2c_port_t;
typedef int gpio_num_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup IMU_SENSOR_STRUCTS IMU 센서 데이터 구조체
 * @brief IMU 센서 데이터 및 설정을 위한 구조체 정의
 * @{
 */

/**
 * @struct imu_data_t
 * @brief IMU 센서 측정 데이터 구조체
 * 
 * MPU6050에서 읽은 가속도계, 자이로스코프 원시 데이터와
 * 계산된 피치/롤 각도를 저장합니다.
 */
typedef struct {
    float accel_x;    ///< X축 가속도 (g)
    float accel_y;    ///< Y축 가속도 (g)
    float accel_z;    ///< Z축 가속도 (g)
    float gyro_x;     ///< X축 각속도 (deg/s)
    float gyro_y;     ///< Y축 각속도 (deg/s)
    float gyro_z;     ///< Z축 각속도 (deg/s)
    float pitch;      ///< 피치 각도 (degree, 전후 기울기)
    float roll;       ///< 롤 각도 (degree, 좌우 기울기)
    bool initialized; ///< 센서 초기화 상태
} imu_data_t;

/**
 * @struct imu_sensor_t
 * @brief IMU 센서 제어 구조체
 * 
 * IMU 센서의 I2C 포트 정보와 측정 데이터를 관리합니다.
 */
typedef struct {
    i2c_port_t i2c_port; ///< I2C 포트 번호
    imu_data_t data;      ///< 센서 측정 데이터
} imu_sensor_t;

/** @} */ // IMU_SENSOR_STRUCTS

/**
 * @defgroup IMU_SENSOR_API IMU 센서 API
 * @brief MPU6050 IMU 센서 제어 함수들
 * @{
 */

/**
 * @brief IMU 센서 초기화
 * 
 * MPU6050 IMU 센서를 초기화하고 I2C 통신을 설정합니다.
 * 
 * 초기화 과정:
 * 1. I2C 드라이버 초기화
 * 2. MPU6050 디바이스 ID 확인
 * 3. 전원 관리 설정 (슬립 모드 해제)
 * 4. 가속도계/자이로스코프 범위 설정
 * 5. 저역 통과 필터 설정
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @param port I2C 포트 번호
 * @param sda_pin I2C SDA 핀 번호
 * @param scl_pin I2C SCL 핀 번호
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패 (통신 오류 또는 디바이스 미검출)
 */
esp_err_t imu_sensor_init(imu_sensor_t* sensor, i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief IMU 센서 데이터 업데이트
 * 
 * MPU6050에서 최신 가속도계/자이로스코프 데이터를 읽어와
 * 피치/롤 각도를 계산합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return esp_err_t 
 *         - ESP_OK: 업데이트 성공
 *         - ESP_FAIL: 업데이트 실패 (통신 오류)
 * 
 * @note 밸런싱 제어를 위해 50Hz 주기로 호출하는 것을 권장합니다.
 */
esp_err_t imu_sensor_update(imu_sensor_t* sensor);

/**
 * @brief 피치 각도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float 피치 각도 (degree, 전후 기울기)
 *         - 양수: 앞으로 기울어짐
 *         - 음수: 뒤로 기울어짐
 */
float imu_sensor_get_pitch(imu_sensor_t* sensor);

/**
 * @brief 롤 각도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float 롤 각도 (degree, 좌우 기울기)
 *         - 양수: 오른쪽으로 기울어짐
 *         - 음수: 왼쪽으로 기울어짐
 */
float imu_sensor_get_roll(imu_sensor_t* sensor);

/**
 * @brief X축 각속도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float X축 각속도 (deg/s, 롤 방향)
 */
float imu_sensor_get_gyro_x(imu_sensor_t* sensor);

/**
 * @brief Y축 각속도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float Y축 각속도 (deg/s, 피치 방향)
 */
float imu_sensor_get_gyro_y(imu_sensor_t* sensor);

/**
 * @brief Z축 각속도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float Z축 각속도 (deg/s, 요 방향)
 */
float imu_sensor_get_gyro_z(imu_sensor_t* sensor);

/**
 * @brief X축 가속도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float X축 가속도 (g)
 */
float imu_sensor_get_accel_x(imu_sensor_t* sensor);

/**
 * @brief Y축 가속도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float Y축 가속도 (g)
 */
float imu_sensor_get_accel_y(imu_sensor_t* sensor);

/**
 * @brief Z축 가속도 읽기
 * @param sensor IMU 센서 구조체 포인터
 * @return float Z축 가속도 (g, 중력 방향에서 약 1.0)
 */
float imu_sensor_get_accel_z(imu_sensor_t* sensor);

/**
 * @brief IMU 센서 초기화 상태 확인
 * @param sensor IMU 센서 구조체 포인터
 * @return bool 
 *         - true: 초기화 완료, 데이터 유효
 *         - false: 초기화 미완료 또는 오류
 */
bool imu_sensor_is_initialized(imu_sensor_t* sensor);

/** @} */ // IMU_SENSOR_API

#ifdef __cplusplus
}
#endif

#endif // IMU_SENSOR_H