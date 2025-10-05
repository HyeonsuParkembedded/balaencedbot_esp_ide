/**
 * @file uart_driver.h
 * @brief GPIO 및 UART 레지스터 직접 제어 통신 드라이버 헤더 파일
 * 
 * ESP32-C6의 UART 하드웨어 레지스터를 직접 조작하는 비트연산 기반 드라이버입니다.
 * GPS 모듈과의 고성능 시리얼 통신 및 소프트웨어 UART 구현을 지원합니다.
 * 
 * 지원 기능:
 * - UART 레지스터 직접 비트 조작
 * - 소프트웨어 UART (GPIO 비트뱅잉)
 * - 하드웨어 UART 직접 제어
 * - 사용자 정의 보드레이트 및 프레임 설정
 * - 인터럽트 기반 RX (링 버퍼)
 * - DMA 지원 (대용량 전송)
 * - FreeRTOS 멀티태스킹 안전
 * - 마이크로초 정밀 타이밍 제어
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 4.0 (FreeRTOS 멀티태스킹 안전 + 인터럽트 + DMA)
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "gpio_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP32-C6 UART 하드웨어 레지스터 직접 접근 정의
#define UART_BASE_ADDR          0x60000000UL    ///< UART 베이스 주소
#define UART0_BASE             (UART_BASE_ADDR + 0x0000)  ///< UART0 베이스
#define UART1_BASE             (UART_BASE_ADDR + 0x10000) ///< UART1 베이스

// UART 레지스터 오프셋 (ESP32-C6 기준)
#define UART_FIFO_REG_OFFSET        0x0000  ///< FIFO 데이터 레지스터
#define UART_INT_RAW_REG_OFFSET     0x0004  ///< 원시 인터럽트 상태
#define UART_INT_ST_REG_OFFSET      0x0008  ///< 인터럽트 상태
#define UART_INT_ENA_REG_OFFSET     0x000C  ///< 인터럽트 활성화
#define UART_INT_CLR_REG_OFFSET     0x0010  ///< 인터럽트 클리어
#define UART_CLKDIV_REG_OFFSET      0x0014  ///< 클럭 분주기
#define UART_STATUS_REG_OFFSET      0x001C  ///< 상태 레지스터
#define UART_CONF0_REG_OFFSET       0x0020  ///< 설정 레지스터 0
#define UART_CONF1_REG_OFFSET       0x0024  ///< 설정 레지스터 1

// UART 레지스터 비트 필드 정의
#define UART_TXFIFO_CNT_MASK        0x3FF   ///< TX FIFO 카운트 마스크
#define UART_RXFIFO_CNT_MASK        0x3FF   ///< RX FIFO 카운트 마스크
#define UART_TXFIFO_CNT_SHIFT       16      ///< TX FIFO 카운트 시프트
#define UART_RXFIFO_CNT_SHIFT       0       ///< RX FIFO 카운트 시프트

// BSW 추상화 계층 - UART 포트 타입 정의
typedef enum {
    BSW_UART_PORT_0 = 0,     ///< UART0 (하드웨어)
    BSW_UART_PORT_1,         ///< UART1 (하드웨어) 
    BSW_UART_SOFTWARE,       ///< 소프트웨어 UART (GPIO 비트뱅잉)
    BSW_UART_PORT_MAX
} bsw_uart_num_t;

/**
 * @brief UART 통신 파라미터 설정 구조체
 */
typedef struct {
    uint32_t baud_rate;          ///< 통신 속도 (bps)
    uint8_t data_bits;           ///< 데이터 비트 수 (5-8)
    uint8_t stop_bits;           ///< 스톱 비트 수 (1-2)
    uint8_t parity;              ///< 패리티 (0:없음, 1:홀수, 2:짝수)
    bsw_gpio_num_t tx_pin;       ///< TX 핀 번호
    bsw_gpio_num_t rx_pin;       ///< RX 핀 번호
    bool use_hardware;           ///< true: 하드웨어 UART, false: 소프트웨어 UART
} uart_bitwise_config_t;

// UART 기본 설정 상수
#define UART_DEFAULT_BAUD_RATE      115200  ///< 기본 보드레이트
#define UART_GPS_BAUD_RATE          9600    ///< GPS 보드레이트
#define UART_DATA_BITS_8            8       ///< 8 데이터 비트
#define UART_STOP_BITS_1            1       ///< 1 스톱 비트
#define UART_PARITY_NONE            0       ///< 패리티 없음
#define UART_FIFO_SIZE              128     ///< FIFO 크기

// 링 버퍼 설정
#define UART_RING_BUFFER_SIZE       512     ///< 링 버퍼 크기 (GPS NMEA 문장 여유롭게 저장)
#define UART_RX_TASK_STACK_SIZE     2048    ///< RX 태스크 스택 크기
#define UART_RX_TASK_PRIORITY       6       ///< RX 태스크 우선순위 (센서보다 높음)

// UART 인터럽트 비트 정의
#define UART_RXFIFO_FULL_INT_BIT    (1 << 0)  ///< RX FIFO 가득참 인터럽트
#define UART_TXFIFO_EMPTY_INT_BIT   (1 << 1)  ///< TX FIFO 비움 인터럽트
#define UART_RXFIFO_TOUT_INT_BIT    (1 << 8)  ///< RX 타임아웃 인터럽트

/**
 * @brief UART 링 버퍼 구조체
 */
typedef struct {
    uint8_t buffer[UART_RING_BUFFER_SIZE];  ///< 데이터 버퍼
    volatile uint32_t head;                 ///< 쓰기 위치
    volatile uint32_t tail;                 ///< 읽기 위치
    volatile uint32_t count;                ///< 현재 데이터 개수
    SemaphoreHandle_t mutex;                ///< 뮤텍스 (멀티태스킹 보호)
    volatile uint32_t overrun_count;        ///< 오버런 카운터
} uart_ring_buffer_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup UART_DRIVER UART 드라이버 API
 * @brief UART 시리얼 통신 인터페이스 함수들
 * @{
 */

/**
 * @brief UART 인터페이스 초기화 (기본 하드웨어 방식)
 * 
 * ESP32-C6의 하드웨어 UART 레지스터를 직접 조작하여 초기화합니다.
 * 
 * @param port UART 포트 번호 (BSW_UART_PORT_0, BSW_UART_PORT_1)
 * @param baudrate 통신 속도 (bps, 예: 9600, 115200)
 * @param tx_pin TX 핀 번호 (bsw_gpio_num_t)
 * @param rx_pin RX 핀 번호 (bsw_gpio_num_t)
 * @return esp_err_t 초기화 결과
 */
esp_err_t uart_driver_init(bsw_uart_num_t port, uint32_t baudrate, bsw_gpio_num_t tx_pin, bsw_gpio_num_t rx_pin);

/**
 * @brief UART 인터페이스 초기화 (사용자 정의 설정)
 * 
 * 비트연산을 통한 직접 레지스터 제어로 UART를 초기화합니다.
 * 
 * @param port UART 포트 번호
 * @param config UART 설정 구조체
 * @return esp_err_t 초기화 결과
 */
esp_err_t uart_driver_init_config(bsw_uart_num_t port, const uart_bitwise_config_t* config);

/**
 * @brief UART에서 데이터 읽기 (비트연산 직접 제어)
 * 
 * UART FIFO 레지스터를 직접 읽어 데이터를 수신합니다.
 * 
 * @param port UART 포트 번호
 * @param data 읽은 데이터를 저장할 버퍼
 * @param max_len 최대 읽을 데이터 길이 (바이트)
 * @param timeout_ms 타임아웃 (밀리초, 0이면 논블로킹)
 * @return int 실제 읽은 데이터 길이
 */
int uart_read_data(bsw_uart_num_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms);

/**
 * @brief UART로 데이터 전송 (비트연산 직접 제어)
 * 
 * UART FIFO 레지스터에 직접 쓰기로 데이터를 전송합니다.
 * 
 * @param port UART 포트 번호
 * @param data 전송할 데이터 버퍼
 * @param len 전송할 데이터 길이 (바이트)
 * @return int 실제 전송한 데이터 길이
 */
int uart_write_data(bsw_uart_num_t port, const uint8_t* data, size_t len);

/**
 * @brief 소프트웨어 UART 데이터 전송 (GPIO 비트뱅잉)
 * 
 * GPIO를 직접 제어하여 UART 프로토콜로 데이터를 전송합니다.
 * 
 * @param tx_pin TX 핀 번호
 * @param data 전송할 바이트
 * @param baud_rate 통신 속도
 * @return esp_err_t 전송 결과
 */
esp_err_t uart_software_write_byte(bsw_gpio_num_t tx_pin, uint8_t data, uint32_t baud_rate);

/**
 * @brief 소프트웨어 UART 데이터 수신 (GPIO 비트뱅잉)
 * 
 * GPIO를 직접 읽어서 UART 프로토콜로 데이터를 수신합니다.
 * 
 * @param rx_pin RX 핀 번호
 * @param data 수신한 데이터를 저장할 포인터
 * @param baud_rate 통신 속도
 * @param timeout_us 타임아웃 (마이크로초)
 * @return esp_err_t 수신 결과
 */
esp_err_t uart_software_read_byte(bsw_gpio_num_t rx_pin, uint8_t* data, uint32_t baud_rate, uint32_t timeout_us);

// 순수 비트연산 UART 레지스터 제어 함수들
void uart_raw_write_reg(bsw_uart_num_t port, uint32_t reg_offset, uint32_t value);
uint32_t uart_raw_read_reg(bsw_uart_num_t port, uint32_t reg_offset);
void uart_raw_set_bits(bsw_uart_num_t port, uint32_t reg_offset, uint32_t bit_mask);
void uart_raw_clear_bits(bsw_uart_num_t port, uint32_t reg_offset, uint32_t bit_mask);

// UART FIFO 직접 제어 함수들
bool uart_tx_fifo_full(bsw_uart_num_t port);
bool uart_rx_fifo_empty(bsw_uart_num_t port);
uint32_t uart_get_tx_fifo_count(bsw_uart_num_t port);
uint32_t uart_get_rx_fifo_count(bsw_uart_num_t port);
void uart_write_fifo_byte(bsw_uart_num_t port, uint8_t data);
uint8_t uart_read_fifo_byte(bsw_uart_num_t port);

/**
 * @brief UART 드라이버 해제
 * 
 * @param port UART 포트 번호
 * @return esp_err_t 해제 결과
 */
esp_err_t uart_driver_deinit(bsw_uart_num_t port);

/**
 * @defgroup UART_RING_BUFFER 링 버퍼 API (v4.0 신규)
 * @brief 인터럽트 기반 비동기 수신을 위한 링 버퍼
 * @{
 */

/**
 * @brief 링 버퍼에서 데이터 읽기 (멀티태스킹 안전)
 * 
 * @param port UART 포트 번호
 * @param data 읽은 데이터를 저장할 버퍼
 * @param max_len 최대 읽을 데이터 길이
 * @param timeout_ms 타임아웃 (밀리초)
 * @return int 실제 읽은 바이트 수 (-1: 에러)
 */
int uart_ring_buffer_read(bsw_uart_num_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms);

/**
 * @brief 링 버퍼의 현재 데이터 개수 조회
 * 
 * @param port UART 포트 번호
 * @return uint32_t 현재 저장된 바이트 수
 */
uint32_t uart_ring_buffer_available(bsw_uart_num_t port);

/**
 * @brief 링 버퍼 초기화 (내부 사용)
 * 
 * @param port UART 포트 번호
 * @return esp_err_t 초기화 결과
 */
esp_err_t uart_ring_buffer_init(bsw_uart_num_t port);

/**
 * @brief 링 버퍼에 데이터 쓰기 (인터럽트/태스크 내부 사용)
 * 
 * @param port UART 포트 번호
 * @param data 쓸 데이터
 * @return esp_err_t 쓰기 결과
 */
esp_err_t uart_ring_buffer_write_byte(bsw_uart_num_t port, uint8_t data);

/**
 * @brief 링 버퍼 통계 정보 출력
 * 
 * @param port UART 포트 번호
 */
void uart_ring_buffer_print_stats(bsw_uart_num_t port);

/** @} */ // UART_RING_BUFFER

/**
 * @defgroup UART_INTERRUPT 인터럽트 제어 API (v4.0 신규)
 * @brief UART 인터럽트 기반 RX 처리
 * @{
 */

/**
 * @brief UART RX 인터럽트 활성화
 * 
 * @param port UART 포트 번호
 * @return esp_err_t 활성화 결과
 */
esp_err_t uart_enable_rx_interrupt(bsw_uart_num_t port);

/**
 * @brief UART RX 인터럽트 비활성화
 * 
 * @param port UART 포트 번호
 * @return esp_err_t 비활성화 결과
 */
esp_err_t uart_disable_rx_interrupt(bsw_uart_num_t port);

/** @} */ // UART_INTERRUPT

/**
 * @defgroup UART_DMA DMA 전송 API (v4.0 신규)
 * @brief 대용량 데이터 전송 최적화
 * @{
 */

/**
 * @brief DMA를 사용한 UART 전송
 * 
 * @param port UART 포트 번호
 * @param data 전송할 데이터 버퍼
 * @param len 전송할 데이터 길이
 * @return esp_err_t 전송 결과
 */
esp_err_t uart_write_dma(bsw_uart_num_t port, const uint8_t* data, size_t len);

/**
 * @brief DMA 전송 완료 대기
 * 
 * @param port UART 포트 번호
 * @param timeout_ms 타임아웃 (밀리초)
 * @return esp_err_t 대기 결과
 */
esp_err_t bsw_uart_wait_tx_done(bsw_uart_num_t port, uint32_t timeout_ms);

/** @} */ // UART_DMA

// 순수 비트연산 UART 레지스터 접근 매크로
#define UART_REG_WRITE(port, offset, val) do { \
    uint32_t base = ((port) == BSW_UART_PORT_0) ? UART0_BASE : UART1_BASE; \
    *(volatile uint32_t*)(base + (offset)) = (val); \
} while(0)

#define UART_REG_READ(port, offset) \
    (*(volatile uint32_t*)(((port) == BSW_UART_PORT_0 ? UART0_BASE : UART1_BASE) + (offset)))

#define UART_REG_SET_BITS(port, offset, mask) do { \
    uint32_t base = ((port) == BSW_UART_PORT_0) ? UART0_BASE : UART1_BASE; \
    *(volatile uint32_t*)(base + (offset)) |= (mask); \
} while(0)

#define UART_REG_CLEAR_BITS(port, offset, mask) do { \
    uint32_t base = ((port) == BSW_UART_PORT_0) ? UART0_BASE : UART1_BASE; \
    *(volatile uint32_t*)(base + (offset)) &= ~(mask); \
} while(0)

// 초고속 UART FIFO 접근 인라인 함수들
static inline void uart_fast_write_byte(bsw_uart_num_t port, uint8_t data) {
    UART_REG_WRITE(port, UART_FIFO_REG_OFFSET, data);
}

static inline uint8_t uart_fast_read_byte(bsw_uart_num_t port) {
    return (uint8_t)UART_REG_READ(port, UART_FIFO_REG_OFFSET);
}

static inline bool uart_fast_tx_ready(bsw_uart_num_t port) {
    uint32_t status = UART_REG_READ(port, UART_STATUS_REG_OFFSET);
    return ((status >> UART_TXFIFO_CNT_SHIFT) & UART_TXFIFO_CNT_MASK) < UART_FIFO_SIZE;
}

static inline bool uart_fast_rx_available(bsw_uart_num_t port) {
    uint32_t status = UART_REG_READ(port, UART_STATUS_REG_OFFSET);
    return ((status >> UART_RXFIFO_CNT_SHIFT) & UART_RXFIFO_CNT_MASK) > 0;
}

static inline void uart_fast_set_baud_rate(bsw_uart_num_t port, uint32_t baud_rate) {
    uint32_t clk_div = 80000000 / baud_rate; // APB 클럭 80MHz 기준
    UART_REG_WRITE(port, UART_CLKDIV_REG_OFFSET, clk_div);
}

/** @} */ // UART_DRIVER

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H