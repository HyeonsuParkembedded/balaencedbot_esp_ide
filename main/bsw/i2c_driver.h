/**
 * @file i2c_driver.h
 * @brief GPIO 직접 제어 Bit-banging I2C 통신 드라이버 헤더 파일
 * 
 * GPIO 레지스터 직접 조작을 통한 소프트웨어 I2C 구현입니다.
 * MPU6050 IMU 센서와의 통신에 사용되며, 하드웨어 I2C 없이도 동작합니다.
 * 
 * 주요 기능:
 * - GPIO 직접 제어 bit-banging I2C
 * - 사용자 정의 클럭 속도 지원
 * - 디바이스 레지스터 읽기/쓰기
 * - 표준 I2C 프로토콜 준수
 * - 오픈 드레인 출력 지원
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 3.0
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "gpio_driver.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// BSW 추상화 계층 - I2C 포트 타입 정의
typedef enum {
    BSW_I2C_PORT_0 = 0,
    BSW_I2C_PORT_1,
    BSW_I2C_PORT_MAX
} bsw_i2c_port_t;

/**
 * @brief I2C 설정 구조체
 */
typedef struct {
    bsw_gpio_num_t sda_pin;     ///< SDA 핀 번호
    bsw_gpio_num_t scl_pin;     ///< SCL 핀 번호
    uint32_t clock_speed_hz;    ///< 클럭 속도 (Hz)
    bool use_pullup;            ///< 내부 풀업 사용 여부
} i2c_bitbang_config_t;

/**
 * @brief I2C 기본 설정 상수
 */
#define I2C_DEFAULT_CLOCK_SPEED     100000  ///< 기본 클럭 속도 100kHz
#define I2C_FAST_CLOCK_SPEED        400000  ///< 고속 클럭 속도 400kHz
#define I2C_CLOCK_STRETCH_TIMEOUT   1000    ///< 클럭 스트레치 타임아웃 (μs)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup I2C_DRIVER GPIO Bit-banging I2C 드라이버 API
 * @brief 소프트웨어 I2C 마스터 인터페이스 함수들
 * @{
 */

/**
 * @brief I2C 인터페이스 초기화 (기본 100kHz)
 * 
 * 지정된 핀으로 bit-banging I2C 마스터를 초기화합니다.
 * GPIO 직접 제어를 통해 I2C 프로토콜을 구현합니다.
 * 
 * @param port I2C 포트 번호 (소프트웨어 구현에서는 인덱스 용도)
 * @param sda_pin SDA 핀 번호
 * @param scl_pin SCL 핀 번호
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note 사용 전에 반드시 호출해야 합니다.
 * @note 핀은 오픈 드레인 모드로 설정됩니다.
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin);

/**
 * @brief I2C 인터페이스 초기화 (사용자 정의 설정)
 * 
 * 사용자 정의 설정으로 bit-banging I2C 마스터를 초기화합니다.
 * 
 * @param port I2C 포트 번호
 * @param config I2C 설정 구조체
 * @return esp_err_t 성공/실패
 */
esp_err_t i2c_driver_init_config(bsw_i2c_port_t port, const i2c_bitbang_config_t* config);

/**
 * @brief I2C 디바이스 레지스터에 데이터 쓰기 (bit-banging)
 * 
 * 지정된 I2C 디바이스의 레지스터에 1바이트 데이터를 씁니다.
 * GPIO 직접 제어를 통한 I2C 프로토콜 구현으로 통신합니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트 주소)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 (1바이트)
 * @return esp_err_t 
 *         - ESP_OK: 쓰기 성공
 *         - ESP_FAIL: 통신 실패 또는 디바이스 응답 없음
 * 
 * @note I2C 프로토콜: START -> ADDR+W -> REG -> DATA -> STOP
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

/**
 * @brief I2C 디바이스 레지스터에서 데이터 읽기 (bit-banging)
 * 
 * 지정된 I2C 디바이스의 레지스터에서 데이터를 읽습니다.
 * GPIO 직접 제어를 통한 I2C 프로토콜 구현으로 통신합니다.
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
 * @note I2C 프로토콜: START -> ADDR+W -> REG -> RESTART -> ADDR+R -> DATA -> STOP
 * @warning data 버퍼는 len 바이트 이상의 크기를 가져야 합니다.
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);

/**
 * @brief I2C 원시 데이터 쓰기 (bit-banging)
 * 
 * I2C 버스에 원시 데이터를 직접 씁니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param data 쓸 데이터 버퍼
 * @param len 데이터 길이
 * @return esp_err_t 성공/실패
 */
esp_err_t i2c_write_raw(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len);

/**
 * @brief I2C 원시 데이터 읽기 (bit-banging)
 * 
 * I2C 버스에서 원시 데이터를 직접 읽습니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param data 읽을 데이터 버퍼
 * @param len 데이터 길이
 * @return esp_err_t 성공/실패
 */
esp_err_t i2c_read_raw(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len);

/**
 * @brief I2C 드라이버 해제
 * 
 * I2C 관련 자원을 해제하고 GPIO 핀을 원래 상태로 복원합니다.
 * 
 * @param port I2C 포트 번호
 * @return esp_err_t 
 *         - ESP_OK: 해제 성공
 *         - ESP_FAIL: 해제 실패
 */
esp_err_t i2c_driver_deinit(bsw_i2c_port_t port);

/** @} */ // I2C_DRIVER

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H