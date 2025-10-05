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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// BSW 정적 메모리 풀 (비트연산 기반)
#define BSW_MEMORY_POOL_SIZE    8192    // 8KB 정적 메모리 풀
#define BSW_MEMORY_BLOCK_SIZE   64      // 64바이트 블록 단위
#define BSW_MEMORY_BLOCKS       (BSW_MEMORY_POOL_SIZE / BSW_MEMORY_BLOCK_SIZE)  // 128개 블록
#define BSW_BITMAP_WORDS        ((BSW_MEMORY_BLOCKS + 31) / 32)  // 4개 워드 (128비트)

static uint8_t g_bsw_memory_pool[BSW_MEMORY_POOL_SIZE];
static uint32_t g_bsw_memory_bitmap[BSW_BITMAP_WORDS] = {0};  // 128비트 비트맵 (4×32비트)
static bool g_bsw_memory_initialized = false;
static SemaphoreHandle_t g_bsw_memory_mutex = NULL;  // 메모리 할당 뮤텍스

// BSW 시스템 타이머 상태
static uint32_t g_bsw_timer_base_ms = 0;
static bool g_bsw_timer_initialized = false;

// BSW UART 출력을 위한 레지스터 정의
#define BSW_UART0_BASE          0x60000000
#define BSW_UART0_FIFO          (BSW_UART0_BASE + 0x000)
#define BSW_UART0_STATUS        (BSW_UART0_BASE + 0x01C)
#define BSW_UART0_CONF0         (BSW_UART0_BASE + 0x020)
#define BSW_UART0_TXFIFO_FULL   (1 << 16)
static SemaphoreHandle_t g_bsw_uart_mutex = NULL;  // UART 출력 뮤텍스

// 내부 함수 선언
static void bsw_init_memory_pool(void);
static void bsw_init_system_timer(void);
static void bsw_init_uart(void);
static void bsw_uart_putchar_bitwise(char c);
static void bsw_uart_puts_bitwise(const char* str);

// BSW 메모리 풀 초기화
static void bsw_init_memory_pool(void)
{
    if (!g_bsw_memory_initialized) {
        // 뮤텍스 생성 (멀티태스킹 안전)
        if (g_bsw_memory_mutex == NULL) {
            g_bsw_memory_mutex = xSemaphoreCreateMutex();
        }
        
        // 비트맵 초기화 (모든 블록 사용 가능)
        for (int i = 0; i < BSW_BITMAP_WORDS; i++) {
            g_bsw_memory_bitmap[i] = 0;
        }
        // 메모리 풀 초기화
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

// BSW UART 초기화
static void bsw_init_uart(void)
{
    if (g_bsw_uart_mutex == NULL) {
        g_bsw_uart_mutex = xSemaphoreCreateMutex();
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
 * @brief BSW 로깅 구현 (비트연산 방식, 멀티태스킹 안전)
 * 
 * UART 레지스터를 직접 제어하여 로그를 출력합니다.
 */
void bsw_log_bitwise(bsw_log_level_t level, const char* tag, const char* format, ...) {
    const char* level_str[] = {"E", "W", "I", "D"};
    
    if (level > BSW_LOG_DEBUG) return;
    
    // UART 초기화 및 뮤텍스 획듥
    bsw_init_uart();
    if (g_bsw_uart_mutex && xSemaphoreTake(g_bsw_uart_mutex, portMAX_DELAY) == pdTRUE) {
    
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
                case 'f': {
                    // Float 지원 (소수점 2자리)
                    double value = va_arg(args, double);
                    int int_part = (int)value;
                    int frac_part = (int)((value - int_part) * 100);
                    if (frac_part < 0) frac_part = -frac_part;
                    
                    // 정수 부분 출력
                    if (int_part < 0) {
                        bsw_uart_putchar_bitwise('-');
                        int_part = -int_part;
                    }
                    
                    char buffer[12];
                    int len = 0;
                    do {
                        buffer[len++] = '0' + (int_part % 10);
                        int_part /= 10;
                    } while (int_part > 0);
                    
                    for (int i = len - 1; i >= 0; i--) {
                        bsw_uart_putchar_bitwise(buffer[i]);
                    }
                    
                    // 소수점 출력
                    bsw_uart_putchar_bitwise('.');
                    bsw_uart_putchar_bitwise('0' + (frac_part / 10));
                    bsw_uart_putchar_bitwise('0' + (frac_part % 10));
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
        
        // 뮤텍스 해제
        xSemaphoreGive(g_bsw_uart_mutex);
    }
}

/**
 * @brief BSW 지연 구현 (멀티태스킹 안전)
 * 
 * FreeRTOS vTaskDelay를 사용하여 다른 태스크에 CPU를 양보합니다.
 */
void bsw_delay_ms(uint32_t delay_ms) {
    // FreeRTOS 태스크 지연 (CPU 양보)
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

/**
 * @brief BSW 시간 측정 구현 (비트연산 방식)
 * 
 * 시스템 타이머 레지스터를 직접 읽어 경과 시간을 반환합니다.
 * ESP32-C6 SYSTIMER는 40MHz XTAL 클럭 사용.
 */
uint32_t bsw_get_time_ms(void) {
    bsw_init_system_timer();
    
    // UPDATE 레지스터 트리거 (값 동기화)
    #define BSW_SYSTIMER_UNIT0_OP (BSW_SYSTIMER_BASE + 0x00C)
    BSW_SYS_REG_WRITE(BSW_SYSTIMER_UNIT0_OP, 1);
    
    // UPDATE 완료 대기 (약간의 지연)
    for (volatile int i = 0; i < 10; i++);
    
    // 시스템 타이머 값 읽기 (64비트)
    uint32_t timer_lo = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_LO);
    uint32_t timer_hi = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_HI);
    
    // 40MHz XTAL 클록 기준으로 밀리초 변환
    uint64_t timer_us = ((uint64_t)timer_hi << 32) | timer_lo;
    return (uint32_t)(timer_us / 40000);  // 마이크로초를 밀리초로 변환 (40MHz)
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
 * @brief BSW 메모리 할당 구현 (비트연산 방식, 멀티태스킹 안전)
 * 
 * 정적 메모리 풀을 사용하여 메모리를 할당합니다.
 * 128개 블록(8KB) 모두 사용 가능.
 */
void* bsw_malloc_bitwise(size_t size) {
    bsw_init_memory_pool();
    
    if (size == 0 || size > BSW_MEMORY_BLOCK_SIZE) {
        return NULL;  // 크기 제한 초과
    }
    
    void* ptr = NULL;
    
    // 뮤텍스 획듩 (멀티스레드 안전)
    if (g_bsw_memory_mutex && xSemaphoreTake(g_bsw_memory_mutex, portMAX_DELAY) == pdTRUE) {
        // 사용 가능한 블록 찾기 (128개 블록 지원)
        for (int i = 0; i < BSW_MEMORY_BLOCKS; i++) {
            int word_index = i / 32;  // 비트맵 워드 인덱스
            int bit_index = i % 32;   // 워드 내 비트 인덱스
            
            if (!(g_bsw_memory_bitmap[word_index] & (1U << bit_index))) {
                // 블록 할당 마킹
                g_bsw_memory_bitmap[word_index] |= (1U << bit_index);
                ptr = &g_bsw_memory_pool[i * BSW_MEMORY_BLOCK_SIZE];
                break;
            }
        }
        
        // 뮤텍스 해제
        xSemaphoreGive(g_bsw_memory_mutex);
    }
    
    return ptr;
}

/**
 * @brief BSW 메모리 해제 구현 (비트연산 방식, 멀티태스킹 안전)
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
    
    if (block_index >= 0 && block_index < BSW_MEMORY_BLOCKS) {
        // 뮤텍스 획듩 (멀티스레드 안전)
        if (g_bsw_memory_mutex && xSemaphoreTake(g_bsw_memory_mutex, portMAX_DELAY) == pdTRUE) {
            // 비트맵에서 블록 해제
            int word_index = block_index / 32;
            int bit_index = block_index % 32;
            g_bsw_memory_bitmap[word_index] &= ~(1U << bit_index);
            
            // 뮤텍스 해제
            xSemaphoreGive(g_bsw_memory_mutex);
        }
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
 * ESP32-C6 워치독 초기화 절차 준수 (WKEY 보호 해제 필수).
 */
bool bsw_watchdog_init_bitwise(uint32_t timeout_ms) {
    // WDT 보호 키 값 (TRM 참조)
    #define BSW_WDT_WKEY (BSW_WDT_BASE + 0x00C)
    #define WDT_WKEY_VALUE 0x50D83AA1
    
    // 1. 보호 해제 (WKEY 레지스터에 매직 넘버 쓰기)
    BSW_SYS_REG_WRITE(BSW_WDT_WKEY, WDT_WKEY_VALUE);
    
    // 2. 워치독 타이머 비활성화
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0);
    
    // 3. 타임아웃 설정 (APB 클록 80MHz 기준)
    uint32_t timeout_cycles = timeout_ms * 80000;  // 80MHz APB 클록
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG1, timeout_cycles);
    
    // 4. 워치독 타이머 활성화 (시스템 리셋 모드)
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0x83);
    
    // 5. 초기 FEED (타이머 시작)
    BSW_SYS_REG_WRITE(BSW_WDT_FEED, 0x50);
    BSW_SYS_REG_WRITE(BSW_WDT_FEED, 0xA0);
    
    // 6. 보호 다시 활성화
    BSW_SYS_REG_WRITE(BSW_WDT_WKEY, 0);
    
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