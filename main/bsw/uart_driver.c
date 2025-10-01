/**
 * @file uart_driver.c
 * @brief GPIO 및 UART 레지스터 직접 제어 통신 드라이버 구현 파일
 * 
 * ESP32-C6의 UART 하드웨어 레지스터를 직접 조작하는 비트연산 기반 드라이버와
 * GPIO 비트뱅킹을 통한 소프트웨어 UART 구현입니다.
 * 
 * 구현 특징:
 * - UART 레지스터 직접 비트 조작
 * - 소프트웨어 UART (GPIO 비트뱅킹)
 * - 하드웨어 UART 직접 제어
 * - 마이크로초 정밀 타이밍
 * - 인터럽트 없는 폴링 방식
 * - 최고 성능 최적화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 3.0 (비트연산 직접 제어)
 */

#include "uart_driver.h"
#include "gpio_driver.h"
#include "system_services.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"  // esp_timer_get_time 함수를 위해 필요
#include <string.h>

static const char* UART_TAG = "UART_BITWISE";

// UART 포트별 설정 저장
static uart_bitwise_config_t uart_configs[BSW_UART_PORT_MAX];
static bool uart_initialized[BSW_UART_PORT_MAX] = {false};

// 소프트웨어 UART 타이밍 계산 헬퍼
static inline uint32_t calculate_bit_delay_us(uint32_t baud_rate) {
    return 1000000 / baud_rate; // 1비트당 마이크로초
}

// 마이크로초 지연 함수
static inline void uart_delay_us(uint32_t delay_us) {
    esp_rom_delay_us(delay_us);
}

/**
 * @brief UART 인터페이스 초기화 구현 (기본 하드웨어 방식)
 */
esp_err_t uart_driver_init(bsw_uart_num_t port, uint32_t baudrate, bsw_gpio_num_t tx_pin, bsw_gpio_num_t rx_pin) {
    uart_bitwise_config_t config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE,
        .tx_pin = tx_pin,
        .rx_pin = rx_pin,
        .use_hardware = true
    };
    
    return uart_driver_init_config(port, &config);
}

/**
 * @brief UART 인터페이스 초기화 구현 (사용자 정의 설정)
 */
esp_err_t uart_driver_init_config(bsw_uart_num_t port, const uart_bitwise_config_t* config) {
    if (port >= BSW_UART_PORT_MAX || !config) {
        bsw_log_bitwise(BSW_LOG_ERROR, UART_TAG, "Invalid UART port %d or config", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 설정 저장
    uart_configs[port] = *config;
    
    if (config->use_hardware && port < BSW_UART_SOFTWARE) {
        // 하드웨어 UART 직접 레지스터 설정
        bsw_log_bitwise(BSW_LOG_INFO, UART_TAG, "Initializing hardware UART %d with bitwise register control", port);
        
        // GPIO 핀을 UART 기능으로 설정
        bsw_gpio_configure_iomux(config->tx_pin, 1, false, false); // UART TX 기능
        bsw_gpio_configure_iomux(config->rx_pin, 1, true, false);  // UART RX 기능, 풀업
        
        // 클럭 분주기 설정 (보드레이트 계산)
        uint32_t clk_div = 80000000 / config->baud_rate; // APB 클럭 80MHz 기준
        UART_REG_WRITE(port, UART_CLKDIV_REG_OFFSET, clk_div);
        
        // UART 설정 레지스터 (CONF0) 직접 조작
        uint32_t conf0_val = 0;
        
        // 데이터 비트 설정 (비트 3:2)
        conf0_val |= ((config->data_bits - 5) & 0x3) << 2;
        
        // 스톱 비트 설정 (비트 5:4)
        conf0_val |= ((config->stop_bits - 1) & 0x3) << 4;
        
        // 패리티 설정 (비트 0: 활성화, 비트 1: 홀수/짝수)
        if (config->parity != UART_PARITY_NONE) {
            conf0_val |= (1 << 0); // 패리티 활성화
            if (config->parity == 1) { // 홀수 패리티
                conf0_val |= (1 << 1);
            }
        }
        
        UART_REG_WRITE(port, UART_CONF0_REG_OFFSET, conf0_val);
        
        // UART 활성화
        UART_REG_SET_BITS(port, UART_CONF0_REG_OFFSET, (1 << 20)); // UART 활성화 비트
        
    } else {
        // 소프트웨어 UART (GPIO 비트뱅킹)
        BSW_LOGI(UART_TAG, "Initializing software UART with GPIO bitbanging");
        
        // TX 핀을 출력으로 설정, 초기 상태 HIGH (IDLE)
        bsw_gpio_config_pin(config->tx_pin, BSW_GPIO_MODE_OUTPUT, 
                            BSW_GPIO_PULLUP_ENABLE, BSW_GPIO_PULLDOWN_DISABLE);
        bsw_gpio_set_high(config->tx_pin);
        
        // RX 핀을 입력으로 설정, 풀업 활성화
        bsw_gpio_config_pin(config->rx_pin, BSW_GPIO_MODE_INPUT, 
                            BSW_GPIO_PULLUP_ENABLE, BSW_GPIO_PULLDOWN_DISABLE);
    }
    
    uart_initialized[port] = true;
    
    BSW_LOGI(UART_TAG, "UART %d initialized: %s mode, baud=%lu, TX=%d, RX=%d", 
             port, config->use_hardware ? "Hardware" : "Software",
             config->baud_rate, config->tx_pin, config->rx_pin);
    
    return ESP_OK;
}

/**
 * @brief UART 데이터 읽기 구현 (비트연산 직접 제어)
 */
int uart_read_data(bsw_uart_num_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms) {
    if (port >= BSW_UART_PORT_MAX || !uart_initialized[port] || !data) {
        BSW_LOGE(UART_TAG, "Invalid UART port %d or uninitialized", port);
        return -1;
    }
    
    if (uart_configs[port].use_hardware && port < BSW_UART_SOFTWARE) {
        // 하드웨어 UART 직접 레지스터 읽기
        int bytes_read = 0;
        uint32_t start_time = bsw_get_time_ms();
        
        while (bytes_read < max_len) {
            // 타임아웃 체크
            if (timeout_ms > 0 && (bsw_get_time_ms() - start_time) > timeout_ms) {
                break;
            }
            
            // RX FIFO에 데이터가 있는지 확인
            if (uart_fast_rx_available(port)) {
                data[bytes_read++] = uart_fast_read_byte(port);
            } else if (timeout_ms == 0) {
                // 논블로킹 모드에서 데이터가 없으면 즉시 리턴
                break;
            }
        }
        
        return bytes_read;
        
    } else {
        // 소프트웨어 UART (GPIO 비트뱅킹)
        int bytes_read = 0;
        uint32_t start_time = bsw_get_time_ms();
        
        while (bytes_read < max_len) {
            // 타임아웃 체크
            if (timeout_ms > 0 && (bsw_get_time_ms() - start_time) > timeout_ms) {
                break;
            }
            
            esp_err_t ret = uart_software_read_byte(uart_configs[port].rx_pin, 
                                                   &data[bytes_read], 
                                                   uart_configs[port].baud_rate, 
                                                   timeout_ms * 1000);
            if (ret == ESP_OK) {
                bytes_read++;
            } else if (timeout_ms == 0) {
                break; // 논블로킹 모드
            }
        }
        
        return bytes_read;
    }
}

/**
 * @brief UART 데이터 쓰기 구현 (비트연산 직접 제어)
 */
int uart_write_data(bsw_uart_num_t port, const uint8_t* data, size_t len) {
    if (port >= BSW_UART_PORT_MAX || !uart_initialized[port] || !data) {
        BSW_LOGE(UART_TAG, "Invalid UART port %d or uninitialized", port);
        return -1;
    }
    
    if (uart_configs[port].use_hardware && port < BSW_UART_SOFTWARE) {
        // 하드웨어 UART 직접 레지스터 쓰기
        int bytes_written = 0;
        
        for (int i = 0; i < len; i++) {
            // TX FIFO가 가득 찰 때까지 대기
            while (!uart_fast_tx_ready(port)) {
                uart_delay_us(10); // 10us 대기
            }
            
            uart_fast_write_byte(port, data[i]);
            bytes_written++;
        }
        
        return bytes_written;
        
    } else {
        // 소프트웨어 UART (GPIO 비트뱅킹)
        int bytes_written = 0;
        
        for (int i = 0; i < len; i++) {
            esp_err_t ret = uart_software_write_byte(uart_configs[port].tx_pin, 
                                                    data[i], 
                                                    uart_configs[port].baud_rate);
            if (ret == ESP_OK) {
                bytes_written++;
            } else {
                break;
            }
        }
        
        return bytes_written;
    }
}

/**
 * @brief 소프트웨어 UART 데이터 전송 (GPIO 비트뱅킹)
 */
esp_err_t uart_software_write_byte(bsw_gpio_num_t tx_pin, uint8_t data, uint32_t baud_rate) {
    uint32_t bit_delay = calculate_bit_delay_us(baud_rate);
    
    // START 비트 (LOW)
    bsw_gpio_fast_set_low(tx_pin);
    uart_delay_us(bit_delay);
    
    // 데이터 비트 8개 (LSB first)
    for (int i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            bsw_gpio_fast_set_high(tx_pin);
        } else {
            bsw_gpio_fast_set_low(tx_pin);
        }
        uart_delay_us(bit_delay);
    }
    
    // STOP 비트 (HIGH)
    bsw_gpio_fast_set_high(tx_pin);
    uart_delay_us(bit_delay);
    
    return ESP_OK;
}

/**
 * @brief 소프트웨어 UART 데이터 수신 (GPIO 비트뱅킹)
 */
esp_err_t uart_software_read_byte(bsw_gpio_num_t rx_pin, uint8_t* data, uint32_t baud_rate, uint32_t timeout_us) {
    uint32_t bit_delay = calculate_bit_delay_us(baud_rate);
    uint32_t half_bit_delay = bit_delay / 2;
    uint32_t start_time = esp_timer_get_time();
    
    // START 비트 대기 (HIGH에서 LOW로 떨어지는 엣지)
    while (bsw_gpio_fast_read_input(rx_pin)) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // START 비트 중간에서 샘플링 (안정성 향상)
    uart_delay_us(half_bit_delay);
    
    // START 비트 확인
    if (bsw_gpio_fast_read_input(rx_pin)) {
        return ESP_FAIL; // 잘못된 START 비트
    }
    
    uart_delay_us(bit_delay); // 첫 번째 데이터 비트로 이동
    
    // 데이터 비트 8개 읽기 (LSB first)
    uint8_t received_data = 0;
    for (int i = 0; i < 8; i++) {
        if (bsw_gpio_fast_read_input(rx_pin)) {
            received_data |= (1 << i);
        }
        uart_delay_us(bit_delay);
    }
    
    // STOP 비트 확인 (HIGH이어야 함)
    if (!bsw_gpio_fast_read_input(rx_pin)) {
        return ESP_FAIL; // 잘못된 STOP 비트
    }
    
    *data = received_data;
    return ESP_OK;
}

// 순수 비트연산 UART 레지스터 제어 함수들
void uart_raw_write_reg(bsw_uart_num_t port, uint32_t reg_offset, uint32_t value) {
    UART_REG_WRITE(port, reg_offset, value);
}

uint32_t uart_raw_read_reg(bsw_uart_num_t port, uint32_t reg_offset) {
    return UART_REG_READ(port, reg_offset);
}

void uart_raw_set_bits(bsw_uart_num_t port, uint32_t reg_offset, uint32_t bit_mask) {
    UART_REG_SET_BITS(port, reg_offset, bit_mask);
}

void uart_raw_clear_bits(bsw_uart_num_t port, uint32_t reg_offset, uint32_t bit_mask) {
    UART_REG_CLEAR_BITS(port, reg_offset, bit_mask);
}

// UART FIFO 직접 제어 함수들
bool uart_tx_fifo_full(bsw_uart_num_t port) {
    return !uart_fast_tx_ready(port);
}

bool uart_rx_fifo_empty(bsw_uart_num_t port) {
    return !uart_fast_rx_available(port);
}

uint32_t uart_get_tx_fifo_count(bsw_uart_num_t port) {
    uint32_t status = UART_REG_READ(port, UART_STATUS_REG_OFFSET);
    return (status >> UART_TXFIFO_CNT_SHIFT) & UART_TXFIFO_CNT_MASK;
}

uint32_t uart_get_rx_fifo_count(bsw_uart_num_t port) {
    uint32_t status = UART_REG_READ(port, UART_STATUS_REG_OFFSET);
    return (status >> UART_RXFIFO_CNT_SHIFT) & UART_RXFIFO_CNT_MASK;
}

void uart_write_fifo_byte(bsw_uart_num_t port, uint8_t data) {
    uart_fast_write_byte(port, data);
}

uint8_t uart_read_fifo_byte(bsw_uart_num_t port) {
    return uart_fast_read_byte(port);
}

/**
 * @brief UART 드라이버 해제
 */
esp_err_t uart_driver_deinit(bsw_uart_num_t port) {
    if (port >= BSW_UART_PORT_MAX) {
        BSW_LOGE(UART_TAG, "Invalid UART port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!uart_initialized[port]) {
        return ESP_OK; // 이미 해제됨
    }
    
    // 하드웨어 UART 비활성화
    if (uart_configs[port].use_hardware && port < BSW_UART_SOFTWARE) {
        UART_REG_CLEAR_BITS(port, UART_CONF0_REG_OFFSET, (1 << 20)); // UART 비활성화
    }
    
    // GPIO 핀 초기화
    bsw_gpio_config_pin(uart_configs[port].tx_pin, BSW_GPIO_MODE_DISABLE, 
                        BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    bsw_gpio_config_pin(uart_configs[port].rx_pin, BSW_GPIO_MODE_DISABLE, 
                        BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    
    uart_initialized[port] = false;
    
    BSW_LOGI(UART_TAG, "UART %d deinitialized", port);
    return ESP_OK;
}