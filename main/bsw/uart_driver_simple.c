/**
 * @file uart_driver.c
 * @brief UART 추상화 드라이버 구현 파일 (간소화 버전)
 * 
 * ESP32-C6와 ESP-IDF v5.x 호환성을 위한 간소화된 UART 드라이버 구현입니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 2.0
 */

#include "uart_driver.h"

#ifndef NATIVE_BUILD
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* UART_TAG = "UART_DRIVER";

// ESP-IDF UART 포트 매핑
static const uart_port_t esp_uart_port_map[] = {
    UART_NUM_0,
    UART_NUM_1
};
#else
#define UART_TAG "UART_DRIVER"
#endif

/**
 * @brief UART 인터페이스 초기화 구현
 */
esp_err_t uart_driver_init(uart_port_t port, uint32_t baudrate, gpio_num_t tx_pin, gpio_num_t rx_pin) {
#ifndef NATIVE_BUILD
    // 포트 검증
    if (port >= UART_PORT_MAX) {
        ESP_LOGE(UART_TAG, "Invalid UART port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    uart_port_t esp_port = esp_uart_port_map[port];
    
    // UART 설정 구조체
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // UART 드라이버 설치
    esp_err_t ret = uart_driver_install(esp_port, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // UART 파라미터 설정
    ret = uart_param_config(esp_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // UART 핀 설정
    ret = uart_set_pin(esp_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(UART_TAG, "UART %d initialized with baud %lu, TX=%d, RX=%d", 
             port, baudrate, tx_pin, rx_pin);
    
    return ESP_OK;
#else
    // 네이티브 빌드에서는 성공 반환
    return 0; // ESP_OK 상당
#endif
}

/**
 * @brief UART 데이터 읽기 구현
 */
int uart_read_data(uart_port_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms) {
#ifndef NATIVE_BUILD
    if (port >= UART_PORT_MAX) {
        ESP_LOGE(UART_TAG, "Invalid UART port: %d", port);
        return -1;
    }
    
    uart_port_t esp_port = esp_uart_port_map[port];
    int len = uart_read_bytes(esp_port, data, max_len, timeout_ms / portTICK_PERIOD_MS);
    
    if (len > 0) {
        ESP_LOGD(UART_TAG, "Read %d bytes from UART %d", len, port);
    }
    
    return len;
#else
    // 네이티브 빌드에서는 0 반환
    return 0;
#endif
}

/**
 * @brief UART 데이터 쓰기 구현
 */
int uart_write_data(uart_port_t port, const uint8_t* data, size_t len) {
#ifndef NATIVE_BUILD
    if (port >= UART_PORT_MAX) {
        ESP_LOGE(UART_TAG, "Invalid UART port: %d", port);
        return -1;
    }
    
    uart_port_t esp_port = esp_uart_port_map[port];
    int written = uart_write_bytes(esp_port, (const char*)data, len);
    
    if (written > 0) {
        ESP_LOGD(UART_TAG, "Written %d bytes to UART %d", written, port);
    }
    
    return written;
#else
    // 네이티브 빌드에서는 입력 길이 반환
    return len;
#endif
}