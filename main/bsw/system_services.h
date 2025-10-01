/**
 * @file system_services.h
 * @brief BSW 시스템 서비스 비트연산 추상화 인터페이스
 * 
 * ESP32-C6 시스템 레지스터를 직접 제어하여 운영체제, 메모리 관리, 
 * 타이머, 로깅 등의 서비스를 하드웨어 독립적으로 추상화합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 2.0 (비트연산 기반)
 */

#ifndef BSW_SYSTEM_SERVICES_H
#define BSW_SYSTEM_SERVICES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
extern "C" {
#endif

// BSW 오류 코드 열거형 (가장 먼저 정의)
typedef enum {
    BSW_OK = 0,                  ///< 성공
    BSW_FAIL = -1,              ///< 일반적인 실패
    BSW_ERR_INVALID_ARG = -2,   ///< 잘못된 인수
    BSW_ERR_NO_MEM = -3,        ///< 메모리 부족
    BSW_ERR_INVALID_STATE = -4, ///< 잘못된 상태
    BSW_ERR_TIMEOUT = -5        ///< 타임아웃
} bsw_err_t;

// BSW 로그 레벨 정의
typedef enum {
    BSW_LOG_ERROR = 0,    ///< 오류 로그
    BSW_LOG_WARN,         ///< 경고 로그
    BSW_LOG_INFO,         ///< 정보 로그
    BSW_LOG_DEBUG         ///< 디버그 로그
} bsw_log_level_t;

// ESP32-C6 시스템 클록 레지스터 정의 (비트연산 방식)
#define BSW_SYSTEM_BASE             0x60026000  // 시스템 레지스터 베이스
#define BSW_SYSTEM_CPU_FREQ_CONF    (BSW_SYSTEM_BASE + 0x000)
#define BSW_SYSTEM_SYSCLK_CONF      (BSW_SYSTEM_BASE + 0x004)
#define BSW_SYSTEM_MEM_CONF         (BSW_SYSTEM_BASE + 0x008)
#define BSW_SYSTEM_RST_EN           (BSW_SYSTEM_BASE + 0x00C)
#define BSW_SYSTEM_CLK_EN           (BSW_SYSTEM_BASE + 0x010)

// ESP32-C6 타이머 레지스터 정의
#define BSW_SYSTIMER_BASE           0x60023000
#define BSW_SYSTIMER_CONF           (BSW_SYSTIMER_BASE + 0x000)
#define BSW_SYSTIMER_UNIT0_VALUE_LO (BSW_SYSTIMER_BASE + 0x004)
#define BSW_SYSTIMER_UNIT0_VALUE_HI (BSW_SYSTIMER_BASE + 0x008)

// ESP32-C6 워치독 레지스터 정의
#define BSW_WDT_BASE                0x6001F000
#define BSW_WDT_CONFIG0             (BSW_WDT_BASE + 0x000)
#define BSW_WDT_CONFIG1             (BSW_WDT_BASE + 0x004)
#define BSW_WDT_FEED                (BSW_WDT_BASE + 0x008)

// 비트연산 기반 레지스터 접근 매크로
#define BSW_SYS_REG_READ(addr)      (*((volatile uint32_t*)(addr)))
#define BSW_SYS_REG_WRITE(addr, val) (*((volatile uint32_t*)(addr)) = (val))
#define BSW_SYS_REG_SET_BIT(addr, bit) (*((volatile uint32_t*)(addr)) |= (bit))
#define BSW_SYS_REG_CLEAR_BIT(addr, bit) (*((volatile uint32_t*)(addr)) &= ~(bit))

// ESP-IDF 호환성 매크로 (ESP-IDF에서 이미 정의된 경우 재정의하지 않음)
#ifndef IRAM_ATTR
#define IRAM_ATTR __attribute__((section(".iram1")))
#endif

/**
 * @defgroup BSW_SYSTEM_SERVICES BSW 시스템 서비스 추상화
 * @brief 하드웨어 독립적인 시스템 서비스 인터페이스
 * @{
 */

// BSW 시스템 리셋 타입
typedef enum {
    BSW_RESET_SOFTWARE,   ///< 소프트웨어 리셋
    BSW_RESET_WATCHDOG,   ///< 워치독 리셋
    BSW_RESET_BROWNOUT,   ///< 브라운아웃 리셋
    BSW_RESET_UNKNOWN     ///< 알 수 없는 리셋
} bsw_reset_reason_t;

/**
 * @brief BSW 로깅 함수 (비트연산 방식)
 * 
 * UART 레지스터를 직접 제어하여 로깅을 수행합니다.
 * 
 * @param level 로그 레벨
 * @param tag 로그 태그
 * @param format 포맷 문자열
 * @param ... 가변 인수
 */
void bsw_log_bitwise(bsw_log_level_t level, const char* tag, const char* format, ...);

/**
 * @brief BSW 지연 함수 (비트연산 방식)
 * 
 * CPU 사이클을 직접 카운팅하여 지연을 구현합니다.
 * 
 * @param delay_ms 지연 시간 (밀리초)
 */
void bsw_delay_ms(uint32_t delay_ms);

/**
 * @brief BSW 시간 측정 함수 (비트연산 방식)
 * 
 * 시스템 타이머 레지스터를 직접 읽어 경과 시간을 반환합니다.
 * 
 * @return uint32_t 경과 시간 (밀리초)
 */
uint32_t bsw_get_time_ms(void);

/**
 * @brief BSW 시스템 리셋 (비트연산 방식)
 * 
 * 시스템 리셋 레지스터를 직접 제어하여 재시작합니다.
 */
void bsw_system_restart(void);

/**
 * @brief BSW 리셋 원인 조회 (비트연산 방식)
 * 
 * 시스템 레지스터에서 마지막 리셋 원인을 읽어옵니다.
 * 
 * @return bsw_reset_reason_t 리셋 원인
 */
bsw_reset_reason_t bsw_get_reset_reason(void);

/**
 * @brief BSW 메모리 할당 (비트연산 방식)
 * 
 * 정적 메모리 풀을 사용하여 메모리를 할당합니다.
 * 
 * @param size 할당할 메모리 크기 (바이트)
 * @return void* 할당된 메모리 포인터 (실패시 NULL)
 */
void* bsw_malloc_bitwise(size_t size);

/**
 * @brief BSW 메모리 해제 (비트연산 방식)
 * 
 * 정적 메모리 풀에서 메모리를 해제합니다.
 * 
 * @param ptr 해제할 메모리 포인터
 */
void bsw_free_bitwise(void* ptr);

/**
 * @brief BSW 문자열 복사 (비트연산 방식)
 * 
 * 안전한 문자열 복사 함수입니다.
 * 
 * @param dest 대상 버퍼
 * @param src 원본 문자열
 * @param size 대상 버퍼 크기
 * @return bool 복사 성공 여부
 */
bool bsw_safe_strcpy_bitwise(char* dest, const char* src, size_t size);

/**
 * @brief BSW 문자열 비교 (비트연산 방식)
 * 
 * 두 문자열을 비교합니다.
 * 
 * @param str1 첫 번째 문자열
 * @param str2 두 번째 문자열
 * @return int 비교 결과 (0: 동일, 음수: str1 < str2, 양수: str1 > str2)
 */
int bsw_strcmp_bitwise(const char* str1, const char* str2);

/**
 * @brief BSW 워치독 초기화 (비트연산 방식)
 * 
 * 워치독 타이머를 레지스터 직접 제어로 초기화합니다.
 * 
 * @param timeout_ms 워치독 타임아웃 (밀리초)
 * @return bool 초기화 성공 여부
 */
bool bsw_watchdog_init_bitwise(uint32_t timeout_ms);

/**
 * @brief BSW 워치독 피드 (비트연산 방식)
 * 
 * 워치독 타이머를 리셋합니다.
 */
void bsw_watchdog_feed_bitwise(void);

// 편의 매크로 정의 (비트연산 기반)
#define BSW_LOGE_BITWISE(tag, format, ...) bsw_log_bitwise(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGW_BITWISE(tag, format, ...) bsw_log_bitwise(BSW_LOG_WARN, tag, format, ##__VA_ARGS__)
#define BSW_LOGI_BITWISE(tag, format, ...) bsw_log_bitwise(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
#define BSW_LOGD_BITWISE(tag, format, ...) bsw_log_bitwise(BSW_LOG_DEBUG, tag, format, ##__VA_ARGS__)

// 통합 BSW_LOG 매크로 (레벨 지정 방식)
#define BSW_LOGE(tag, format, ...) bsw_log_bitwise(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGW(tag, format, ...) bsw_log_bitwise(BSW_LOG_WARN, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log_bitwise(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
#define BSW_LOGD(tag, format, ...) bsw_log_bitwise(BSW_LOG_DEBUG, tag, format, ##__VA_ARGS__)

/** @} */ // BSW_SYSTEM_SERVICES

#ifdef __cplusplus
}
#endif

#endif // BSW_SYSTEM_SERVICES_H