/**
 * @file i2c_driver.h
 * @brief I2C 통신 드라이버 헤더 파일
 * 
 * ESP32-S3의 I2C 인터페이스를 추상화한 드라이버입니다.
 * MPU6050 IMU 센서와의 통신에 사용됩니다.
 * 
 * 주요 기능:
 * - I2C 마스터 모드 초기화
 * - 디바이스 레지스터 읽기/쓰기
 * - 오류 처리 및 타임아웃 관리
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

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

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup I2C_DRIVER I2C 드라이버 API
 * @brief I2C 마스터 인터페이스 함수들
 * @{
 */

/**
 * @brief I2C 인터페이스 초기화
 * 
 * 지정된 핀으로 I2C 마스터 모드를 초기화합니다.
 * 400kHz 클록 속도로 설정되며, 내부 풀업 저항을 활성화합니다.
 * 
 * @param port I2C 포트 번호 (I2C_NUM_0 또는 I2C_NUM_1)
 * @param sda_pin SDA 핀 번호 (GPIO_NUM_x)
 * @param scl_pin SCL 핀 번호 (GPIO_NUM_x)
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note 사용 전에 반드시 호출해야 합니다.
 * @warning 동일한 포트에 대해 중복 초기화하면 오류가 발생할 수 있습니다.
 */
esp_err_t i2c_driver_init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief I2C 디바이스 레지스터에 데이터 쓰기
 * 
 * 지정된 I2C 디바이스의 레지스터에 1바이트 데이터를 씁니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트 주소)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 (1바이트)
 * @return esp_err_t 
 *         - ESP_OK: 쓰기 성공
 *         - ESP_FAIL: 통신 실패 또는 디바이스 응답 없음
 * 
 * @note 타임아웃은 1초로 설정됩니다.
 */
esp_err_t i2c_write_register(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

/**
 * @brief I2C 디바이스 레지스터에서 데이터 읽기
 * 
 * 지정된 I2C 디바이스의 레지스터에서 데이터를 읽습니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트 주소)
 * @param reg_addr 레지스터 주소
 * @param data 읽은 데이터를 저장할 버퍼 포인터
 * @param len 읽을 데이터 길이 (바이트)
 * @return esp_err_t 
 *         - ESP_OK: 읽기 성공
 *         - ESP_FAIL: 통신 실패 또는 디바이스 응답 없음
 * 
 * @note 타임아웃은 1초로 설정됩니다.
 * @warning data 버퍼는 len 바이트 이상의 크기를 가져야 합니다.
 */
esp_err_t i2c_read_register(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);

/** @} */ // I2C_DRIVER

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H