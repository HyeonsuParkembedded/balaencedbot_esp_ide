/**
 * @file uart_driver.h
 * @brief UART 통신 드라이버 헤더 파일
 * 
 * ESP32-S3의 UART 인터페이스를 추상화한 드라이버입니다.
 * GPS 모듈과의 시리얼 통신에 사용됩니다.
 * 
 * 지원 기능:
 * - 가변 보드레이트 설정
 * - 논블로킹/블로킹 읽기 모드
 * - 타임아웃 기반 읽기
 * - 효율적인 버퍼 관리
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

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

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup UART_DRIVER UART 드라이버 API
 * @brief UART 시리얼 통신 인터페이스 함수들
 * @{
 */

/**
 * @brief UART 인터페이스 초기화
 * 
 * 지정된 핀과 보드레이트로 UART 인터페이스를 초기화합니다.
 * 
 * 설정 파라미터:
 * - 데이터 비트: 8비트
 * - 패리티: 없음
 * - 스톱 비트: 1비트
 * - 플로우 제어: 없음
 * - RX 버퍼: 1024바이트
 * - TX 버퍼: 1024바이트
 * 
 * @param port UART 포트 번호 (UART_NUM_0, UART_NUM_1, UART_NUM_2)
 * @param tx_pin TX 핀 번호 (GPIO_NUM_x)
 * @param rx_pin RX 핀 번호 (GPIO_NUM_x)
 * @param baudrate 통신 속도 (bps, 예: 9600, 115200)
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note GPS 모듈은 일반적으로 9600bps를 사용합니다.
 */
esp_err_t uart_driver_init(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baudrate);

/**
 * @brief UART에서 데이터 읽기
 * 
 * 지정된 타임아웃 내에서 UART 버퍼에서 데이터를 읽습니다.
 * 논블로킹 모드로 동작하며, 사용 가능한 데이터만 읽습니다.
 * 
 * @param port UART 포트 번호
 * @param data 읽은 데이터를 저장할 버퍼
 * @param max_len 최대 읽을 데이터 길이 (바이트)
 * @param timeout_ms 타임아웃 (밀리초, 0이면 논블로킹)
 * @return int 실제 읽은 데이터 길이 (바이트)
 *         - 양수: 읽은 바이트 수
 *         - 0: 읽을 데이터 없음 또는 타임아웃
 *         - 음수: 오류 발생
 * 
 * @warning data 버퍼는 max_len 바이트 이상의 크기를 가져야 합니다.
 */
int uart_read_data(uart_port_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms);

/**
 * @brief UART로 데이터 전송
 * 
 * 지정된 데이터를 UART를 통해 전송합니다.
 * 블로킹 모드로 동작하며, 모든 데이터가 전송될 때까지 대기합니다.
 * 
 * @param port UART 포트 번호
 * @param data 전송할 데이터 버퍼
 * @param len 전송할 데이터 길이 (바이트)
 * @return esp_err_t 
 *         - ESP_OK: 전송 성공
 *         - ESP_FAIL: 전송 실패
 * 
 * @note 전송 타임아웃은 내부적으로 관리됩니다.
 */
esp_err_t uart_write_data(uart_port_t port, const uint8_t* data, size_t len);

/** @} */ // UART_DRIVER

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H