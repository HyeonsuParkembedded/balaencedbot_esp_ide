/**
 * @file system_services.c
 * @brief BSW 시스템 서비스 비트연산 구현
 * 
 * ESP32-C6 시스템 레지스터를 직접 제어하여 시스템 서비스들을 
 * BSW 인터페이스로 구현하여 상위 계층의 하드웨어 독립성을 보장합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 2.0 (비트연산 기반)
 */

#include "system_services.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// BSW 정적 메모리 풀 (비트연산 기반)
#define BSW_MEMORY_POOL_SIZE    8192    // 8KB 정적 메모리 풀
#define BSW_MEMORY_BLOCK_SIZE   64      // 64바이트 블록 단위
#define BSW_MEMORY_BLOCKS       (BSW_MEMORY_POOL_SIZE / BSW_MEMORY_BLOCK_SIZE)

static uint8_t g_bsw_memory_pool[BSW_MEMORY_POOL_SIZE];
static uint32_t g_bsw_memory_bitmap = 0;  // 블록 할당 상태 (32개 블록)
static bool g_bsw_memory_initialized = false;

// BSW 시스템 타이머 상태
static uint32_t g_bsw_timer_base_ms = 0;
static bool g_bsw_timer_initialized = false;

// BSW UART 출력을 위한 레지스터 정의
#define BSW_UART0_BASE          0x60000000
#define BSW_UART0_FIFO          (BSW_UART0_BASE + 0x000)
#define BSW_UART0_STATUS        (BSW_UART0_BASE + 0x01C)
#define BSW_UART0_CONF0         (BSW_UART0_BASE + 0x020)
#define BSW_UART0_TXFIFO_FULL   (1 << 16)

// 내부 함수 선언
static void bsw_init_memory_pool(void);
static void bsw_init_system_timer(void);
static void bsw_uart_putchar_bitwise(char c);
static void bsw_uart_puts_bitwise(const char* str);

// BSW 메모리 풀 초기화
static void bsw_init_memory_pool(void)
{
    if (!g_bsw_memory_initialized) {
        // 메모리 풀 초기화 (모든 블록 사용 가능)
        g_bsw_memory_bitmap = 0;
        for (int i = 0; i < BSW_MEMORY_POOL_SIZE; i++) {
            g_bsw_memory_pool[i] = 0;
        }
        g_bsw_memory_initialized = true;
    }
}

// BSW 시스템 타이머 초기화
static void bsw_init_system_timer(void)
{
    if (!g_bsw_timer_initialized) {
        // 시스템 타이머 활성화
        BSW_SYS_REG_SET_BIT(BSW_SYSTIMER_CONF, (1 << 0));
        g_bsw_timer_base_ms = 0;
        g_bsw_timer_initialized = true;
    }
}

// BSW UART 단일 문자 출력 (비트연산)
static void bsw_uart_putchar_bitwise(char c)
{
    // UART FIFO가 가득 찰 때까지 대기
    while (BSW_SYS_REG_READ(BSW_UART0_STATUS) & BSW_UART0_TXFIFO_FULL) {
        // 대기 (FIFO 여유 공간 확보까지)
    }
    
    // 문자를 UART FIFO에 기록
    BSW_SYS_REG_WRITE(BSW_UART0_FIFO, c);
}

// BSW UART 문자열 출력 (비트연산)
static void bsw_uart_puts_bitwise(const char* str)
{
    if (!str) return;
    
    while (*str) {
        bsw_uart_putchar_bitwise(*str++);
    }
}

/**
 * @brief BSW 로깅 구현 (비트연산 방식)
 * 
 * UART 레지스터를 직접 제어하여 로그를 출력합니다.
 */
void bsw_log_bitwise(bsw_log_level_t level, const char* tag, const char* format, ...) {
    const char* level_str[] = {"E", "W", "I", "D"};
    
    if (level > BSW_LOG_DEBUG) return;
    
    // 로그 헤더 출력 (시간은 생략)
    bsw_uart_putchar_bitwise('[');
    bsw_uart_puts_bitwise(level_str[level]);
    bsw_uart_puts_bitwise("] ");
    bsw_uart_puts_bitwise(tag);
    bsw_uart_puts_bitwise(": ");
    
    // 간단한 형식화된 출력 (printf 기능 제한적 구현)
    va_list args;
    va_start(args, format);
    
    const char* p = format;
    while (*p) {
        if (*p == '%' && *(p + 1)) {
            p++;
            switch (*p) {
                case 'd': {
                    int value = va_arg(args, int);
                    char buffer[12];
                    int len = 0;
                    
                    if (value < 0) {
                        bsw_uart_putchar_bitwise('-');
                        value = -value;
                    }
                    
                    // 숫자를 문자열로 변환
                    do {
                        buffer[len++] = '0' + (value % 10);
                        value /= 10;
                    } while (value > 0);
                    
                    // 역순으로 출력
                    for (int i = len - 1; i >= 0; i--) {
                        bsw_uart_putchar_bitwise(buffer[i]);
                    }
                    break;
                }
                case 's': {
                    const char* str = va_arg(args, const char*);
                    if (str) {
                        bsw_uart_puts_bitwise(str);
                    }
                    break;
                }
                case 'c': {
                    char c = (char)va_arg(args, int);
                    bsw_uart_putchar_bitwise(c);
                    break;
                }
                default:
                    bsw_uart_putchar_bitwise('%');
                    bsw_uart_putchar_bitwise(*p);
                    break;
            }
        } else {
            bsw_uart_putchar_bitwise(*p);
        }
        p++;
    }
    
    bsw_uart_putchar_bitwise('\r');
    bsw_uart_putchar_bitwise('\n');
    
    va_end(args);
}

/**
 * @brief BSW 지연 구현 (비트연산 방식)
 * 
 * CPU 사이클을 직접 카운팅하여 지연을 구현합니다.
 */
void bsw_delay_ms(uint32_t delay_ms) {
    // ESP32-C6 @ 160MHz 기준: 1ms = 160,000 사이클
    volatile uint32_t cycles = delay_ms * 160000;
    
    while (cycles--) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief BSW 시간 측정 구현 (비트연산 방식)
 * 
 * 시스템 타이머 레지스터를 직접 읽어 경과 시간을 반환합니다.
 */
uint32_t bsw_get_time_ms(void) {
    bsw_init_system_timer();
    
    // 시스템 타이머 값 읽기 (64비트)
    uint32_t timer_lo = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_LO);
    uint32_t timer_hi = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_HI);
    
    // 16MHz 클록 기준으로 밀리초 변환 (타이머 클록은 보통 16MHz)
    uint64_t timer_us = ((uint64_t)timer_hi << 32) | timer_lo;
    return (uint32_t)(timer_us / 16000);  // 마이크로초를 밀리초로 변환
}

/**
 * @brief BSW 시스템 재시작 구현 (비트연산 방식)
 * 
 * 시스템 리셋 레지스터를 직접 제어하여 재시작합니다.
 */
void bsw_system_restart(void) {
    // 소프트웨어 리셋 비트 설정
    BSW_SYS_REG_SET_BIT(BSW_SYSTEM_RST_EN, (1 << 0));
    
    // 무한 루프 (리셋 발생까지 대기)
    while (1) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief BSW 리셋 원인 조회 구현 (비트연산 방식)
 * 
 * 시스템 레지스터에서 마지막 리셋 원인을 읽어옵니다.
 */
bsw_reset_reason_t bsw_get_reset_reason_bitwise(void) {
    // 시스템 리셋 원인 레지스터 읽기 (가상 구현)
    uint32_t reset_reason_reg = BSW_SYS_REG_READ(BSW_SYSTEM_RST_EN);
    
    // 비트 패턴에 따른 리셋 원인 판별
    if (reset_reason_reg & (1 << 4)) {
        return BSW_RESET_WATCHDOG;
    } else if (reset_reason_reg & (1 << 3)) {
        return BSW_RESET_BROWNOUT;
    } else if (reset_reason_reg & (1 << 1)) {
        return BSW_RESET_SOFTWARE;
    } else {
        return BSW_RESET_UNKNOWN;
    }
}

/**
 * @brief BSW 메모리 할당 구현 (비트연산 방식)
 * 
 * 정적 메모리 풀을 사용하여 메모리를 할당합니다.
 */
void* bsw_malloc_bitwise(size_t size) {
    bsw_init_memory_pool();
    
    if (size == 0 || size > BSW_MEMORY_BLOCK_SIZE) {
        return NULL;  // 크기 제한 초과
    }
    
    // 사용 가능한 블록 찾기 (32개 블록까지 지원)
    for (int i = 0; i < 32 && i < BSW_MEMORY_BLOCKS; i++) {
        if (!(g_bsw_memory_bitmap & (1U << i))) {
            // 블록 할당 마킹
            g_bsw_memory_bitmap |= (1U << i);
            return &g_bsw_memory_pool[i * BSW_MEMORY_BLOCK_SIZE];
        }
    }
    
    return NULL;  // 사용 가능한 블록 없음
}

/**
 * @brief BSW 메모리 해제 구현 (비트연산 방식)
 * 
 * 정적 메모리 풀에서 메모리를 해제합니다.
 */
void bsw_free_bitwise(void* ptr) {
    if (ptr == NULL) {
        return;
    }
    
    // 메모리 풀 범위 내 포인터인지 확인
    if (ptr < (void*)g_bsw_memory_pool || 
        ptr >= (void*)(g_bsw_memory_pool + BSW_MEMORY_POOL_SIZE)) {
        return;  // 잘못된 포인터
    }
    
    // 블록 인덱스 계산
    uintptr_t offset = (uintptr_t)ptr - (uintptr_t)g_bsw_memory_pool;
    int block_index = offset / BSW_MEMORY_BLOCK_SIZE;
    
    if (block_index >= 0 && block_index < 32 && block_index < BSW_MEMORY_BLOCKS) {
        // 블록 해제 마킹
        g_bsw_memory_bitmap &= ~(1 << block_index);
    }
}

/**
 * @brief BSW 안전한 문자열 복사 구현 (비트연산 방식)
 * 
 * 버퍼 오버플로우를 방지하는 안전한 문자열 복사를 수행합니다.
 */
bool bsw_safe_strcpy_bitwise(char* dest, const char* src, size_t size) {
    if (dest == NULL || src == NULL || size == 0) {
        return false;
    }
    
    size_t i = 0;
    while (i < size - 1 && src[i] != '\0') {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0';  // 널 터미네이터 보장
    
    return true;
}

/**
 * @brief BSW 문자열 비교 구현 (비트연산 방식)
 * 
 * 표준 strcmp와 동일한 동작을 구현합니다.
 */
int bsw_strcmp_bitwise(const char* str1, const char* str2) {
    if (str1 == NULL && str2 == NULL) {
        return 0;
    }
    if (str1 == NULL) {
        return -1;
    }
    if (str2 == NULL) {
        return 1;
    }
    
    while (*str1 && *str2 && *str1 == *str2) {
        str1++;
        str2++;
    }
    
    return (unsigned char)*str1 - (unsigned char)*str2;
}

/**
 * @brief BSW 워치독 초기화 구현 (비트연산 방식)
 * 
 * 워치독 타이머를 레지스터 직접 제어로 초기화합니다.
 */
bool bsw_watchdog_init_bitwise(uint32_t timeout_ms) {
    // 워치독 타이머 비활성화
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0);
    
    // 타임아웃 설정 (밀리초를 클록 사이클로 변환)
    uint32_t timeout_cycles = timeout_ms * 40000;  // 40MHz 클록 기준
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG1, timeout_cycles);
    
    // 워치독 타이머 활성화
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0x83);  // 활성화 + 시스템 리셋 모드
    
    return true;
}

/**
 * @brief BSW 워치독 피드 구현 (비트연산 방식)
 * 
 * 워치독 타이머를 리셋합니다.
 */
void bsw_watchdog_feed_bitwise(void) {
    // 워치독 피드 시퀀스 (특정 값 순서로 기록)
    BSW_SYS_REG_WRITE(BSW_WDT_FEED, 0x50);
    BSW_SYS_REG_WRITE(BSW_WDT_FEED, 0xA0);
}