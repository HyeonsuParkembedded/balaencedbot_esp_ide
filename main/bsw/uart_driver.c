/**
 * @file uart_driver.c
 * @brief UART 통신 드라이버 구현 파일
 * 
 * ESP32-S3의 UART 인터페이스를 사용하여 GPS 모듈과 시리얼 통신을 수행합니다.
 * 표준 8N1 설정과 효율적인 버퍼 관리를 제공합니다.
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "uart_driver.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#endif

#ifndef NATIVE_BUILD
static const char* UART_TAG = "UART_DRIVER"; ///< ESP-IDF 로깅 태그
#else
#define UART_TAG "UART_DRIVER" ///< 네이티브 빌드용 로깅 태그
#endif

/**
 * @brief UART 인터페이스 초기화 구현
 * 
 * ESP-IDF UART 드라이버를 사용하여 시리얼 통신을 설정합니다.
 * 
 * 설정 파라미터:
 * - 데이터 비트: 8비트
 * - 패리티: 없음
 * - 스톱 비트: 1비트
 * - 하드웨어 플로우 제어: 비활성화
 * - 클록 소스: APB 클록
 * - RX 버퍼: 1024바이트
 * 
 * @param port UART 포트 번호
 * @param tx_pin TX 핀 번호
 * @param rx_pin RX 핀 번호
 * @param baudrate 통신 속도
 * @return esp_err_t 초기화 결과
 */
esp_err_t uart_driver_init(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baudrate) {
#ifndef NATIVE_BUILD
    uart_config_t uart_config = {
        .baud_rate = baudrate,                    // 사용자 지정 보드레이트
        .data_bits = UART_DATA_8_BITS,           // 8비트 데이터
        .parity = UART_PARITY_DISABLE,           // 패리티 없음
        .stop_bits = UART_STOP_BITS_1,           // 1 스톱 비트
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,   // 하드웨어 플로우 제어 없음
        .source_clk = UART_SCLK_APB,             // APB 클록 사용
    };

    // UART 드라이버 설치 (RX 버퍼 1024바이트, TX 버퍼 없음)
    esp_err_t ret = uart_driver_install(port, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART driver install failed");
        return ret;
    }

    // UART 파라미터 설정
    ret = uart_param_config(port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART param config failed");
        return ret;
    }

    // GPIO 핀 할당 (RTS/CTS 핀은 사용하지 않음)
    ret = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART set pin failed");
        return ret;
    }

    ESP_LOGI(UART_TAG, "UART driver initialized");
#endif
    return ESP_OK;
}

/**
 * @brief UART 데이터 읽기 구현
 * 
 * UART 수신 버퍼에서 데이터를 읽어옵니다.
 * 논블로킹 모드로 동작하며, 지정된 타임아웃 내에서 사용 가능한 데이터만 읽습니다.
 * 
 * @param port UART 포트 번호
 * @param data 데이터 저장 버퍼
 * @param max_len 최대 읽기 길이
 * @param timeout_ms 타임아웃 (밀리초)
 * @return int 실제 읽은 바이트 수
 */
int uart_read_data(uart_port_t port, uint8_t* data, size_t max_len, uint32_t timeout_ms) {
#ifndef NATIVE_BUILD
    // 타임아웃을 밀리초에서 틱으로 변환하여 데이터 읽기
    return uart_read_bytes(port, data, max_len, timeout_ms / portTICK_PERIOD_MS);
#else
    // 네이티브 빌드를 위한 모의 GPS 데이터
    const char* mock_gps = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    int len = strlen(mock_gps);
    if (len > max_len) len = max_len;
    memcpy(data, mock_gps, len);
    return len;
#endif
}

/**
 * @brief UART 데이터 전송 구현
 * 
 * UART 포트를 통해 데이터를 전송합니다.
 * 전송 완료까지 블로킹됩니다.
 * 
 * @param port UART 포트 번호
 * @param data 전송할 데이터 버퍼
 * @param len 전송할 데이터 길이
 * @return esp_err_t 전송 결과 (ESP_OK: 성공)
 */
esp_err_t uart_write_data(uart_port_t port, const uint8_t* data, size_t len) {
#ifndef NATIVE_BUILD
    // UART 포트로 데이터 전송
    int written = uart_write_bytes(port, (const char*)data, len);
    return (written == len) ? ESP_OK : ESP_FAIL;
#else
    // 네이티브 빌드에서는 성공 반환
    return ESP_OK;
#endif
}