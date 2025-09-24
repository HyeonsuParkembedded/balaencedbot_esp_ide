/**
 * @file i2c_driver.h
 * @brief ESP32-C6 I2C 통신 드라이버 헤더 파일
 * 
 * ESP32-C6의 I2C 인터페이스를 추상화한 드라이버입니다.
 * MPU6050 IMU 센서와의 통신에 사용됩니다.
 * 새로운 ESP-IDF v5.x I2C API를 기반으로 구현되었습니다.
 * 
 * 주요 기능:
 * - I2C 마스터 버스 초기화 (새로운 API)
 * - 디바이스 레지스터 읽기/쓰기
 * - 오류 처리 및 타임아웃 관리
 * - ESP32-C6 최적화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#ifndef NATIVE_BUILD
#include "driver/gpio.h"       // GPIO 타입 정의 (gpio_num_t)
#include "esp_err.h"
#else
typedef int esp_err_t;
typedef int gpio_num_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif

#include <stdint.h>

// BSW 추상화 계층 - I2C 포트 타입 정의
typedef enum {
    BSW_I2C_PORT_0 = 0,
    BSW_I2C_PORT_1,
    BSW_I2C_PORT_MAX
} bsw_i2c_port_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup I2C_DRIVER I2C 드라이버 API
 * @brief I2C 마스터 인터페이스 함수들
 * @{
 */

/**
 * @brief I2C 인터페이스 초기화 (새로운 ESP-IDF v5.x API)
 * 
 * 지정된 핀으로 I2C 마스터 버스를 초기화합니다.
 * 400kHz 클록 속도로 설정되며, 내부 풀업 저항을 활성화합니다.
 * 
 * @param port I2C 포트 번호 (새 API에서는 자동 할당됨)
 * @param sda_pin SDA 핀 번호 (GPIO_NUM_x)
 * @param scl_pin SCL 핀 번호 (GPIO_NUM_x)
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note 사용 전에 반드시 호출해야 합니다.
 * @note 새로운 API는 버스/디바이스 아키텍처를 사용합니다.
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief I2C 디바이스 레지스터에 데이터 쓰기 (새로운 API)
 * 
 * 지정된 I2C 디바이스의 레지스터에 1바이트 데이터를 씁니다.
 * 새로운 ESP-IDF v5.x API를 사용하여 자동으로 디바이스 핸들을 관리합니다.
 * 
 * @param port I2C 포트 번호 (새 API에서는 사용하지 않음)
 * @param device_addr I2C 디바이스 주소 (7비트 주소)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 (1바이트)
 * @return esp_err_t 
 *         - ESP_OK: 쓰기 성공
 *         - ESP_FAIL: 통신 실패 또는 디바이스 응답 없음
 * 
 * @note 무제한 타임아웃(-1)으로 설정됩니다.
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

/**
 * @brief I2C 디바이스 레지스터에서 데이터 읽기 (새로운 API)
 * 
 * 지정된 I2C 디바이스의 레지스터에서 데이터를 읽습니다.
 * 새로운 ESP-IDF v5.x API의 transmit_receive를 사용하여 효율적으로 읽습니다.
 * 
 * @param port I2C 포트 번호 (새 API에서는 사용하지 않음)
 * @param device_addr I2C 디바이스 주소 (7비트 주소)
 * @param reg_addr 레지스터 주소
 * @param data 읽은 데이터를 저장할 버퍼 포인터
 * @param len 읽을 데이터 길이 (바이트)
 * @return esp_err_t 
 *         - ESP_OK: 읽기 성공
 *         - ESP_FAIL: 통신 실패 또는 디바이스 응답 없음
 * 
 * @note 무제한 타임아웃(-1)으로 설정됩니다.
 * @warning data 버퍼는 len 바이트 이상의 크기를 가져야 합니다.
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);

/**
 * @brief I2C 드라이버 해제
 * 
 * I2C 마스터 버스와 관련 자원을 해제합니다.
 * 
 * @return esp_err_t 
 *         - ESP_OK: 해제 성공
 *         - ESP_FAIL: 해제 실패
 */
esp_err_t i2c_driver_deinit(void);

/** @} */ // I2C_DRIVER

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H