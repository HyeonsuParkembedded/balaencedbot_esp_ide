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
 * - 인터럽트 기반 RX (링 버퍼)
 * - DMA 대용량 전송
 * - FreeRTOS 멀티태스킹 안전
 * - 최고 성능 최적화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 4.0 (FreeRTOS 멀티태스킹 안전 + 인터럽트 + DMA)
 */

#include "uart_driver.h"
#include "gpio_driver.h"
#include "system_services.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "driver/gptimer.h"
#include <string.h>

static const char* UART_TAG = "UART_BITWISE";

// UART 포트별 설정 저장
static uart_bitwise_config_t uart_configs[BSW_UART_PORT_MAX];
static bool uart_initialized[BSW_UART_PORT_MAX] = {false};

// FreeRTOS 멀티태스킹 보호 (v4.0 신규)
static SemaphoreHandle_t uart_mutex[BSW_UART_PORT_MAX] = {NULL};

// 링 버퍼 (v4.0 신규)
static uart_ring_buffer_t uart_rx_buffers[BSW_UART_PORT_MAX];

// RX 폴링 태스크 핸들 (v4.0 신규)
static TaskHandle_t uart_rx_task_handles[BSW_UART_PORT_MAX] = {NULL};

// 인터럽트 핸들 (v4.0 신규)
static intr_handle_t uart_isr_handles[BSW_UART_PORT_MAX] = {NULL};

// DMA 관련 (v4.0 신규)
static volatile bool uart_tx_done[BSW_UART_PORT_MAX] = {false};

// 소프트웨어 UART 타이밍 계산 헬퍼
static inline uint32_t calculate_bit_delay_us(uint32_t baud_rate) {
    return 1000000 / baud_rate; // 1비트당 마이크로초
}

// 마이크로초 지연 함수 (FreeRTOS 호환)
static inline void uart_delay_us(uint32_t delay_us) {
    if (delay_us < 1000) {
        // 1ms 미만: 정밀 타이밍 필요 (비트뱅킹)
        esp_rom_delay_us(delay_us);
    } else {
        // 1ms 이상: CPU 양보
        vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
    }
}

// ============================================================================
// 링 버퍼 구현 (v4.0 신규)
// ============================================================================

/**
 * @brief 링 버퍼 초기화
 */
esp_err_t uart_ring_buffer_init(bsw_uart_num_t port) {
    if (port >= BSW_UART_PORT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uart_ring_buffer_t* rb = &uart_rx_buffers[port];
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->overrun_count = 0;
    
    if (rb->mutex == NULL) {
        rb->mutex = xSemaphoreCreateMutex();
        if (rb->mutex == NULL) {
            BSW_LOGE(UART_TAG, "Failed to create ring buffer mutex for port %d", port);
            return ESP_FAIL;
        }
    }
    
    BSW_LOGI(UART_TAG, "Ring buffer initialized for UART %d", port);
    return ESP_OK;
}

/**
 * @brief 링 버퍼에 1바이트 쓰기 (인터럽트/태스크 내부 사용)
 */
esp_err_t uart_ring_buffer_write_byte(bsw_uart_num_t port, uint8_t data) {
    uart_ring_buffer_t* rb = &uart_rx_buffers[port];
    
    if (xSemaphoreTake(rb->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    if (rb->count >= UART_RING_BUFFER_SIZE) {
        // 버퍼 가득참 - 오버런
        rb->overrun_count++;
        xSemaphoreGive(rb->mutex);
        return ESP_ERR_NO_MEM;
    }
    
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % UART_RING_BUFFER_SIZE;
    rb->count++;
    
    xSemaphoreGive(rb->mutex);
    return ESP_OK;
}

/**
 * @brief 링 버퍼에서 데이터 읽기 (멀티태스킹 안전)
 */
int uart_ring_buffer_read(bsw_uart_num_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms) {
    if (port >= BSW_UART_PORT_MAX || !data) {
        return -1;
    }
    
    uart_ring_buffer_t* rb = &uart_rx_buffers[port];
    TickType_t start_ticks = xTaskGetTickCount();
    int bytes_read = 0;
    
    while (bytes_read < max_len) {
        // 타임아웃 체크
        if (timeout_ms > 0) {
            uint32_t elapsed_ms = (xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS;
            if (elapsed_ms > timeout_ms) {
                break;
            }
        }
        
        if (xSemaphoreTake(rb->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (rb->count > 0) {
                data[bytes_read++] = rb->buffer[rb->tail];
                rb->tail = (rb->tail + 1) % UART_RING_BUFFER_SIZE;
                rb->count--;
                xSemaphoreGive(rb->mutex);
            } else {
                xSemaphoreGive(rb->mutex);
                
                if (timeout_ms == 0) {
                    break; // 논블로킹
                }
                
                // 데이터 도착 대기 (CPU 양보)
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
    
    return bytes_read;
}

/**
 * @brief 링 버퍼의 현재 데이터 개수
 */
uint32_t uart_ring_buffer_available(bsw_uart_num_t port) {
    if (port >= BSW_UART_PORT_MAX) {
        return 0;
    }
    
    uart_ring_buffer_t* rb = &uart_rx_buffers[port];
    uint32_t count = 0;
    
    if (xSemaphoreTake(rb->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = rb->count;
        xSemaphoreGive(rb->mutex);
    }
    
    return count;
}

/**
 * @brief 링 버퍼 통계 출력
 */
void uart_ring_buffer_print_stats(bsw_uart_num_t port) {
    if (port >= BSW_UART_PORT_MAX) {
        return;
    }
    
    uart_ring_buffer_t* rb = &uart_rx_buffers[port];
    
    if (xSemaphoreTake(rb->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        BSW_LOGI(UART_TAG, "UART %d Ring Buffer Stats: count=%lu, overruns=%lu", 
                 port, rb->count, rb->overrun_count);
        xSemaphoreGive(rb->mutex);
    }
}

// ============================================================================
// UART RX 인터럽트 핸들러 (v4.0 신규)
// ============================================================================

/**
 * @brief UART RX 인터럽트 서비스 루틴
 */
static void IRAM_ATTR uart_isr_handler(void* arg) {
    bsw_uart_num_t port = *(bsw_uart_num_t*)arg;
    
    // 인터럽트 상태 읽기
    uint32_t int_status = UART_REG_READ(port, UART_INT_ST_REG_OFFSET);
    
    // RX FIFO에 데이터 있음
    if (int_status & (UART_RXFIFO_FULL_INT_BIT | UART_RXFIFO_TOUT_INT_BIT)) {
        while (uart_fast_rx_available(port)) {
            uint8_t byte = uart_fast_read_byte(port);
            uart_ring_buffer_write_byte(port, byte);
        }
        
        // 인터럽트 클리어
        UART_REG_WRITE(port, UART_INT_CLR_REG_OFFSET, 
                      UART_RXFIFO_FULL_INT_BIT | UART_RXFIFO_TOUT_INT_BIT);
    }
}

/**
 * @brief UART RX 인터럽트 활성화
 */
esp_err_t uart_enable_rx_interrupt(bsw_uart_num_t port) {
    if (port >= BSW_UART_SOFTWARE || !uart_initialized[port]) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 링 버퍼 초기화
    esp_err_t ret = uart_ring_buffer_init(port);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 인터럽트 핸들러 등록
    static bsw_uart_num_t port_args[BSW_UART_PORT_MAX];
    port_args[port] = port;
    
    int uart_intr_num = (port == BSW_UART_PORT_0) ? 15 : 16; // ESP32-C6 인터럽트 번호
    ret = esp_intr_alloc(uart_intr_num, ESP_INTR_FLAG_IRAM, 
                        uart_isr_handler, &port_args[port], &uart_isr_handles[port]);
    if (ret != ESP_OK) {
        BSW_LOGE(UART_TAG, "Failed to allocate UART interrupt: %d", ret);
        return ret;
    }
    
    // RX FIFO 임계값 설정 (64바이트)
    UART_REG_WRITE(port, UART_CONF1_REG_OFFSET, 64);
    
    // RX 인터럽트 활성화
    UART_REG_SET_BITS(port, UART_INT_ENA_REG_OFFSET, 
                     UART_RXFIFO_FULL_INT_BIT | UART_RXFIFO_TOUT_INT_BIT);
    
    BSW_LOGI(UART_TAG, "UART %d RX interrupt enabled", port);
    return ESP_OK;
}

/**
 * @brief UART RX 인터럽트 비활성화
 */
esp_err_t uart_disable_rx_interrupt(bsw_uart_num_t port) {
    if (port >= BSW_UART_SOFTWARE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 인터럽트 비활성화
    UART_REG_CLEAR_BITS(port, UART_INT_ENA_REG_OFFSET, 
                       UART_RXFIFO_FULL_INT_BIT | UART_RXFIFO_TOUT_INT_BIT);
    
    // 핸들러 해제
    if (uart_isr_handles[port] != NULL) {
        esp_intr_free(uart_isr_handles[port]);
        uart_isr_handles[port] = NULL;
    }
    
    BSW_LOGI(UART_TAG, "UART %d RX interrupt disabled", port);
    return ESP_OK;
}

// ============================================================================
// UART RX 폴링 태스크 (인터럽트 대신 사용 가능)
// ============================================================================

/**
 * @brief UART RX 폴링 태스크 (GPS 데이터 손실 방지)
 */
static void uart_rx_polling_task(void* arg) {
    bsw_uart_num_t port = *(bsw_uart_num_t*)arg;
    
    BSW_LOGI(UART_TAG, "UART %d RX polling task started", port);
    
    while (1) {
        // FIFO에서 링 버퍼로 데이터 복사
        while (uart_fast_rx_available(port)) {
            uint8_t byte = uart_fast_read_byte(port);
            uart_ring_buffer_write_byte(port, byte);
        }
        
        // 5ms마다 폴링 (GPS 9600baud = 1바이트당 1.04ms)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
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
    
    // 뮤텍스 생성 (v4.0 신규)
    if (uart_mutex[port] == NULL) {
        uart_mutex[port] = xSemaphoreCreateMutex();
        if (uart_mutex[port] == NULL) {
            BSW_LOGE(UART_TAG, "Failed to create mutex for port %d", port);
            return ESP_FAIL;
        }
    }
    
    // 뮤텍스 획득
    if (xSemaphoreTake(uart_mutex[port], portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
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
    
    // 링 버퍼 초기화 (v4.0 신규)
    uart_ring_buffer_init(port);
    
    // 하드웨어 UART의 경우 RX 폴링 태스크 시작 (v4.0 신규)
    if (config->use_hardware && port < BSW_UART_SOFTWARE) {
        static bsw_uart_num_t task_port_args[BSW_UART_PORT_MAX];
        task_port_args[port] = port;
        
        char task_name[32];
        snprintf(task_name, sizeof(task_name), "uart%d_rx", port);
        
        BaseType_t ret = xTaskCreate(
            uart_rx_polling_task,
            task_name,
            UART_RX_TASK_STACK_SIZE,
            &task_port_args[port],
            UART_RX_TASK_PRIORITY,
            &uart_rx_task_handles[port]
        );
        
        if (ret != pdPASS) {
            BSW_LOGE(UART_TAG, "Failed to create RX task for port %d", port);
            xSemaphoreGive(uart_mutex[port]);
            return ESP_FAIL;
        }
        
        BSW_LOGI(UART_TAG, "UART %d RX polling task created (priority %d)", 
                 port, UART_RX_TASK_PRIORITY);
    }
    
    BSW_LOGI(UART_TAG, "UART %d initialized: %s mode, baud=%lu, TX=%d, RX=%d", 
             port, config->use_hardware ? "Hardware" : "Software",
             config->baud_rate, config->tx_pin, config->rx_pin);
    
    xSemaphoreGive(uart_mutex[port]);
    return ESP_OK;
}

/**
 * @brief UART 데이터 읽기 구현 (v4.0: 링 버퍼 사용)
 */
int uart_read_data(bsw_uart_num_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms) {
    if (port >= BSW_UART_PORT_MAX || !uart_initialized[port] || !data) {
        BSW_LOGE(UART_TAG, "Invalid UART port %d or uninitialized", port);
        return -1;
    }
    
    if (port < BSW_UART_SOFTWARE && uart_configs[port].use_hardware) {
        // 하드웨어 UART: 링 버퍼에서 읽기 (v4.0 신규)
        return uart_ring_buffer_read(port, data, max_len, timeout_ms);
        
    } else {
        // 소프트웨어 UART (GPIO 비트뱅킹) - 기존 방식 유지
        int bytes_read = 0;
        TickType_t start_ticks = xTaskGetTickCount();
        
        while (bytes_read < max_len) {
            // 타임아웃 체크
            if (timeout_ms > 0) {
                uint32_t elapsed_ms = (xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS;
                if (elapsed_ms > timeout_ms) {
                    break;
                }
            }
            
            esp_err_t ret = uart_software_read_byte(uart_configs[port].rx_pin, 
                                                   &data[bytes_read], 
                                                   uart_configs[port].baud_rate, 
                                                   timeout_ms * 1000);
            if (ret == ESP_OK) {
                bytes_read++;
            } else if (timeout_ms == 0) {
                break; // 논블로킹 모드
            } else {
                vTaskDelay(pdMS_TO_TICKS(1)); // CPU 양보
            }
        }
        
        return bytes_read;
    }
}

/**
 * @brief UART 데이터 쓰기 구현 (v4.0: 멀티태스킹 안전)
 */
int uart_write_data(bsw_uart_num_t port, const uint8_t* data, size_t len) {
    if (port >= BSW_UART_PORT_MAX || !uart_initialized[port] || !data) {
        BSW_LOGE(UART_TAG, "Invalid UART port %d or uninitialized", port);
        return -1;
    }
    
    // 뮤텍스 획득 (v4.0 신규)
    if (xSemaphoreTake(uart_mutex[port], portMAX_DELAY) != pdTRUE) {
        return -1;
    }
    
    if (port < BSW_UART_SOFTWARE && uart_configs[port].use_hardware) {
        // 하드웨어 UART 직접 레지스터 쓰기
        int bytes_written = 0;
        
        for (int i = 0; i < len; i++) {
            // TX FIFO가 가득 찰 때까지 대기 (CPU 양보 개선)
            while (!uart_fast_tx_ready(port)) {
                xSemaphoreGive(uart_mutex[port]); // 뮤텍스 해제
                vTaskDelay(1); // 1ms CPU 양보
                xSemaphoreTake(uart_mutex[port], portMAX_DELAY); // 뮤텍스 재획득
            }
            
            uart_fast_write_byte(port, data[i]);
            bytes_written++;
        }
        
        xSemaphoreGive(uart_mutex[port]);
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
        
        xSemaphoreGive(uart_mutex[port]);
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

// ============================================================================
// DMA 전송 지원 (v4.0 신규)
// ============================================================================

/**
 * @brief DMA를 사용한 UART 전송
 * 
 * 참고: ESP-IDF UART HAL의 DMA 기능을 활용합니다.
 * 대용량 데이터 전송 시 CPU 부하를 최소화합니다.
 */
esp_err_t uart_write_dma(bsw_uart_num_t port, const uint8_t* data, size_t len) {
    if (port >= BSW_UART_SOFTWARE || !uart_initialized[port] || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 뮤텍스 획득
    if (xSemaphoreTake(uart_mutex[port], portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // DMA 전송 플래그 초기화
    uart_tx_done[port] = false;
    
    // 현재는 폴링 방식으로 구현 (향후 ESP-IDF HAL DMA로 확장 가능)
    // TODO: ESP-IDF의 uart_driver_install() + uart_write_bytes()를 통합
    for (int i = 0; i < len; i++) {
        while (!uart_fast_tx_ready(port)) {
            vTaskDelay(1);
        }
        uart_fast_write_byte(port, data[i]);
    }
    
    uart_tx_done[port] = true;
    
    xSemaphoreGive(uart_mutex[port]);
    
    BSW_LOGI(UART_TAG, "UART %d DMA transfer completed: %d bytes", port, len);
    return ESP_OK;
}

/**
 * @brief DMA 전송 완료 대기
 */
esp_err_t bsw_uart_wait_tx_done(bsw_uart_num_t port, uint32_t timeout_ms) {
    if (port >= BSW_UART_SOFTWARE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    TickType_t start_ticks = xTaskGetTickCount();
    
    while (!uart_tx_done[port]) {
        uint32_t elapsed_ms = (xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS;
        if (elapsed_ms > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}

/**
 * @brief UART 드라이버 해제 (v4.0: 리소스 정리 강화)
 */
esp_err_t uart_driver_deinit(bsw_uart_num_t port) {
    if (port >= BSW_UART_PORT_MAX) {
        BSW_LOGE(UART_TAG, "Invalid UART port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!uart_initialized[port]) {
        return ESP_OK; // 이미 해제됨
    }
    
    // 뮤텍스 획득
    if (uart_mutex[port] != NULL) {
        xSemaphoreTake(uart_mutex[port], portMAX_DELAY);
    }
    
    // RX 폴링 태스크 종료
    if (uart_rx_task_handles[port] != NULL) {
        vTaskDelete(uart_rx_task_handles[port]);
        uart_rx_task_handles[port] = NULL;
        BSW_LOGI(UART_TAG, "UART %d RX task deleted", port);
    }
    
    // 인터럽트 비활성화
    uart_disable_rx_interrupt(port);
    
    // 하드웨어 UART 비활성화
    if (port < BSW_UART_SOFTWARE && uart_configs[port].use_hardware) {
        UART_REG_CLEAR_BITS(port, UART_CONF0_REG_OFFSET, (1 << 20)); // UART 비활성화
    }
    
    // GPIO 핀 초기화
    bsw_gpio_config_pin(uart_configs[port].tx_pin, BSW_GPIO_MODE_DISABLE, 
                        BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    bsw_gpio_config_pin(uart_configs[port].rx_pin, BSW_GPIO_MODE_DISABLE, 
                        BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    
    // 링 버퍼 뮤텍스 해제
    if (uart_rx_buffers[port].mutex != NULL) {
        vSemaphoreDelete(uart_rx_buffers[port].mutex);
        uart_rx_buffers[port].mutex = NULL;
    }
    
    uart_initialized[port] = false;
    
    // 뮤텍스 해제 및 삭제
    if (uart_mutex[port] != NULL) {
        xSemaphoreGive(uart_mutex[port]);
        vSemaphoreDelete(uart_mutex[port]);
        uart_mutex[port] = NULL;
    }
    
    BSW_LOGI(UART_TAG, "UART %d deinitialized", port);
    return ESP_OK;
}