/**
 * @file system_services.c
 * @brief BSW 시스템 서비스 구현
 * 
 * 하드웨어 종속적인 시스템 서비스들을 BSW 인터페이스로
 * 래핑하여 상위 계층의 하드웨어 독립성을 보장합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 1.0
 */

#include "system_services.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/**
 * @brief BSW 로깅 구현
 * 
 * 플랫폼별 로깅 시스템을 사용하여 로그를 출력합니다.
 */
void bsw_log(bsw_log_level_t level, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    // ESP-IDF 로깅 사용
    switch (level) {
        case BSW_LOG_ERROR:
            esp_log_writev(ESP_LOG_ERROR, tag, format, args);
            break;
        case BSW_LOG_WARN:
            esp_log_writev(ESP_LOG_WARN, tag, format, args);
            break;
        case BSW_LOG_INFO:
            esp_log_writev(ESP_LOG_INFO, tag, format, args);
            break;
        case BSW_LOG_DEBUG:
            esp_log_writev(ESP_LOG_DEBUG, tag, format, args);
            break;
    }
    
    va_end(args);
}

/**
 * @brief BSW 지연 구현
 * 
 * 플랫폼별 지연 함수를 사용합니다.
 */
void bsw_delay_ms(uint32_t delay_ms) {
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

/**
 * @brief BSW 시간 측정 구현
 * 
 * 시스템 부팅 후 경과 시간을 반환합니다.
 */
uint32_t bsw_get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);  // 마이크로초를 밀리초로 변환
}

/**
 * @brief BSW 시스템 재시작 구현
 * 
 * 플랫폼별 재시작 함수를 호출합니다.
 */
void bsw_system_restart(void) {
    esp_restart();
}

/**
 * @brief BSW 리셋 원인 조회 구현
 * 
 * 마지막 시스템 리셋의 원인을 조회합니다.
 */
bsw_reset_reason_t bsw_get_reset_reason(void) {
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON:
        case ESP_RST_SW:
            return BSW_RESET_SOFTWARE;
        case ESP_RST_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_INT_WDT:
            return BSW_RESET_WATCHDOG;
        case ESP_RST_BROWNOUT:
            return BSW_RESET_BROWNOUT;
        default:
            return BSW_RESET_UNKNOWN;
    }
}

/**
 * @brief BSW 메모리 할당 구현
 * 
 * 플랫폼별 메모리 할당 함수를 사용합니다.
 */
void* bsw_malloc(size_t size) {
    return malloc(size);  // ESP-IDF도 표준 malloc 사용 가능
}

/**
 * @brief BSW 메모리 해제 구현
 * 
 * 플랫폼별 메모리 해제 함수를 사용합니다.
 */
void bsw_free(void* ptr) {
    if (ptr != NULL) {
        free(ptr);
    }
}

/**
 * @brief BSW 안전한 문자열 복사 구현
 * 
 * 버퍼 오버플로우를 방지하는 안전한 문자열 복사를 수행합니다.
 */
bool bsw_safe_strcpy(char* dest, const char* src, size_t size) {
    if (dest == NULL || src == NULL || size == 0) {
        return false;
    }
    
    strncpy(dest, src, size - 1);
    dest[size - 1] = '\0';  // 널 터미네이터 보장
    
    return true;
}

/**
 * @brief BSW 문자열 비교 구현
 * 
 * 표준 strcmp를 래핑합니다.
 */
int bsw_strcmp(const char* str1, const char* str2) {
    if (str1 == NULL && str2 == NULL) {
        return 0;
    }
    if (str1 == NULL) {
        return -1;
    }
    if (str2 == NULL) {
        return 1;
    }
    
    return strcmp(str1, str2);
}