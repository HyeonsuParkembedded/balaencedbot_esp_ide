/**
 * @file system_services.h
 * @brief BSW 시스템 서비스 추상화 인터페이스
 * 
 * 운영체제, 메모리 관리, 타이머, 로깅 등 시스템 서비스를 
 * 하드웨어 독립적으로 추상화합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 1.0
 */

#ifndef BSW_SYSTEM_SERVICES_H
#define BSW_SYSTEM_SERVICES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup BSW_SYSTEM_SERVICES BSW 시스템 서비스 추상화
 * @brief 하드웨어 독립적인 시스템 서비스 인터페이스
 * @{
 */

// BSW 로그 레벨 정의
typedef enum {
    BSW_LOG_ERROR = 0,    ///< 오류 로그
    BSW_LOG_WARN,         ///< 경고 로그
    BSW_LOG_INFO,         ///< 정보 로그
    BSW_LOG_DEBUG         ///< 디버그 로그
} bsw_log_level_t;

// BSW 시스템 리셋 타입
typedef enum {
    BSW_RESET_SOFTWARE,   ///< 소프트웨어 리셋
    BSW_RESET_WATCHDOG,   ///< 워치독 리셋
    BSW_RESET_BROWNOUT,   ///< 브라운아웃 리셋
    BSW_RESET_UNKNOWN     ///< 알 수 없는 리셋
} bsw_reset_reason_t;

/**
 * @brief BSW 로깅 함수
 * 
 * 하드웨어 독립적인 로깅 인터페이스입니다.
 * 
 * @param level 로그 레벨
 * @param tag 로그 태그
 * @param format 포맷 문자열
 * @param ... 가변 인수
 */
void bsw_log(bsw_log_level_t level, const char* tag, const char* format, ...);

/**
 * @brief BSW 지연 함수
 * 
 * 지정된 시간만큼 현재 태스크를 지연합니다.
 * 
 * @param delay_ms 지연 시간 (밀리초)
 */
void bsw_delay_ms(uint32_t delay_ms);

/**
 * @brief BSW 시간 측정 함수
 * 
 * 시스템 부팅 후 경과 시간을 밀리초 단위로 반환합니다.
 * 
 * @return uint32_t 경과 시간 (밀리초)
 */
uint32_t bsw_get_time_ms(void);

/**
 * @brief BSW 시스템 리셋
 * 
 * 시스템을 재시작합니다.
 */
void bsw_system_restart(void);

/**
 * @brief BSW 리셋 원인 조회
 * 
 * 마지막 시스템 리셋의 원인을 조회합니다.
 * 
 * @return bsw_reset_reason_t 리셋 원인
 */
bsw_reset_reason_t bsw_get_reset_reason(void);

/**
 * @brief BSW 메모리 할당
 * 
 * 동적 메모리를 할당합니다.
 * 
 * @param size 할당할 메모리 크기 (바이트)
 * @return void* 할당된 메모리 포인터 (실패시 NULL)
 */
void* bsw_malloc(size_t size);

/**
 * @brief BSW 메모리 해제
 * 
 * 동적 할당된 메모리를 해제합니다.
 * 
 * @param ptr 해제할 메모리 포인터
 */
void bsw_free(void* ptr);

/**
 * @brief BSW 문자열 복사
 * 
 * 안전한 문자열 복사 함수입니다.
 * 
 * @param dest 대상 버퍼
 * @param src 원본 문자열
 * @param size 대상 버퍼 크기
 * @return bool 복사 성공 여부
 */
bool bsw_safe_strcpy(char* dest, const char* src, size_t size);

/**
 * @brief BSW 문자열 비교
 * 
 * 두 문자열을 비교합니다.
 * 
 * @param str1 첫 번째 문자열
 * @param str2 두 번째 문자열
 * @return int 비교 결과 (0: 동일, 음수: str1 < str2, 양수: str1 > str2)
 */
int bsw_strcmp(const char* str1, const char* str2);

// 편의 매크로 정의
#define BSW_LOGE(tag, format, ...) bsw_log(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGW(tag, format, ...) bsw_log(BSW_LOG_WARN, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
#define BSW_LOGD(tag, format, ...) bsw_log(BSW_LOG_DEBUG, tag, format, ##__VA_ARGS__)

/** @} */ // BSW_SYSTEM_SERVICES

#ifdef __cplusplus
}
#endif

#endif // BSW_SYSTEM_SERVICES_H