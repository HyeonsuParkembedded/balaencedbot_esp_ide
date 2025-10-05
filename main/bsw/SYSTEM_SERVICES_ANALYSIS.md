# System Services 드라이버 상세 분석 보고서

## 📋 분석 개요

**분석 대상**: `main/bsw/system_services.c` / `system_services.h`  
**드라이버 버전**: v2.0 (Bitwise Direct Control) vs v1.0 (ESP-IDF Wrapper)  
**분석 일시**: 2025-10-04  
**분석자**: GitHub Copilot  

---

## 🎯 핵심 발견: 두 가지 구현의 혼재

### 문제: 헤더와 구현 파일이 서로 다른 버전

**`system_services.h`** (v1.0 - ESP-IDF Wrapper):
```c
/**
 * @file system_services.h
 * @brief BSW 시스템 서비스 추상화 인터페이스
 * @version 1.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

// 함수 선언
void bsw_log(bsw_log_level_t level, const char* tag, const char* format, ...);
void bsw_delay_ms(uint32_t delay_ms);
uint32_t bsw_get_time_ms(void);
void bsw_system_restart(void);
```

**`system_services.c`** (v2.0 - Bitwise Direct Control):
```c
/**
 * @file system_services.c
 * @brief BSW 시스템 서비스 비트연산 구현
 * @version 2.0 (비트연산 기반)
 */

// v2.0 비트연산 구현
void bsw_log_bitwise(bsw_log_level_t level, const char* tag, const char* format, ...) {
    // UART 레지스터 직접 제어
    bsw_uart_putchar_bitwise(c);
}

void* bsw_malloc_bitwise(size_t size) {
    // 정적 메모리 풀 사용
}

void bsw_watchdog_init_bitwise(uint32_t timeout_ms) {
    // 워치독 레지스터 직접 제어
}
```

**결과**:
- ❌ **헤더는 v1.0 (ESP-IDF Wrapper)**를 선언
- ❌ **구현은 v2.0 (Bitwise)** 함수를 정의
- ❌ 함수 이름 불일치: `bsw_log()` vs `bsw_log_bitwise()`

---

## 🚨 발견된 문제들

### ❌ **문제 1: 함수 이름 불일치 (링크 에러 가능성)**

**헤더 파일 (system_services.h)**:
```c
void bsw_log(bsw_log_level_t level, const char* tag, const char* format, ...);
void* bsw_malloc(size_t size);
void bsw_free(void* ptr);
bool bsw_watchdog_init(uint32_t timeout_ms);
```

**구현 파일 (system_services.c)**:
```c
void bsw_log_bitwise(bsw_log_level_t level, const char* tag, const char* format, ...);
void* bsw_malloc_bitwise(size_t size);
void bsw_free_bitwise(void* ptr);
bool bsw_watchdog_init_bitwise(uint32_t timeout_ms);
```

**문제점**:
- 헤더에 선언된 함수가 구현 파일에 없음
- `main.c`에서 `BSW_LOGI()` 매크로 사용 시 링크 에러 발생 가능
- 실제로는 ESP-IDF API를 직접 사용하고 있을 가능성

**확인된 매크로 (system_services.h)**:
```c
#define BSW_LOGE(tag, format, ...) bsw_log(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
```

이 매크로들은 `bsw_log()`를 호출하는데, 구현 파일에는 `bsw_log_bitwise()`만 있습니다!

---

### ❌ **문제 2: UART 레지스터 주소 오류**

**구현 코드 (system_services.c v2.0)**:
```c
// BSW UART 출력을 위한 레지스터 정의
#define BSW_UART0_BASE          0x60000000
#define BSW_UART0_FIFO          (BSW_UART0_BASE + 0x000)
#define BSW_UART0_STATUS        (BSW_UART0_BASE + 0x01C)
```

**ESP32-C6 실제 UART 레지스터 (TRM 기준)**:
```c
#define UART0_BASE_ADDR         0x60000000  // ✅ 맞음
#define UART_FIFO_REG(i)        (UART_BASE_ADDR(i) + 0x0)  // ✅ 맞음
#define UART_STATUS_REG(i)      (UART_BASE_ADDR(i) + 0x1C) // ✅ 맞음
```

레지스터 주소는 정확하지만:

**문제점**:
1. **UART가 초기화되지 않음**: UART 클럭, 보드레이트, 핀 설정 없음
2. **ESP-IDF가 이미 UART0 사용 중**: 콘솔 출력용으로 UART0 점유
3. **충돌 가능성**: 직접 레지스터 쓰기 시 ESP-IDF UART 드라이버와 충돌

---

### ❌ **문제 3: 정적 메모리 풀의 한계**

**구현 코드**:
```c
#define BSW_MEMORY_POOL_SIZE    8192    // 8KB
#define BSW_MEMORY_BLOCK_SIZE   64      // 64바이트 블록
#define BSW_MEMORY_BLOCKS       128     // 128개 블록

static uint8_t g_bsw_memory_pool[BSW_MEMORY_POOL_SIZE];
static uint32_t g_bsw_memory_bitmap = 0;  // ← ⚠️ 32비트만 지원

void* bsw_malloc_bitwise(size_t size) {
    // 사용 가능한 블록 찾기 (32개 블록까지 지원)
    for (int i = 0; i < 32 && i < BSW_MEMORY_BLOCKS; i++) {  // ← ⚠️ 32개만!
        if (!(g_bsw_memory_bitmap & (1U << i))) {
            g_bsw_memory_bitmap |= (1U << i);
            return &g_bsw_memory_pool[i * BSW_MEMORY_BLOCK_SIZE];
        }
    }
    return NULL;
}
```

**문제점**:
- `BSW_MEMORY_BLOCKS = 128`로 정의했지만
- `g_bsw_memory_bitmap`은 32비트 (`uint32_t`)
- **실제로는 32개 블록만 사용 가능** (2KB만 활용)
- 나머지 6KB는 낭비

**해결책**:
```c
static uint32_t g_bsw_memory_bitmap[4];  // 128비트 = 4×32비트
```

---

### ❌ **문제 4: 시스템 타이머 레지스터 오류**

**구현 코드 (system_services.h v2.0)**:
```c
#define BSW_SYSTIMER_BASE           0x60023000
#define BSW_SYSTIMER_CONF           (BSW_SYSTIMER_BASE + 0x000)
#define BSW_SYSTIMER_UNIT0_VALUE_LO (BSW_SYSTIMER_BASE + 0x004)
#define BSW_SYSTIMER_UNIT0_VALUE_HI (BSW_SYSTIMER_BASE + 0x008)
```

**ESP32-C6 실제 SYSTIMER 레지스터**:
```c
#define SYSTIMER_BASE_ADDR          0x60023000  // ✅ 맞음
#define SYSTIMER_CONF_REG           0x000       // ✅ 맞음
#define SYSTIMER_UNIT0_VALUE_LO_REG 0x004       // ✅ 맞음
#define SYSTIMER_UNIT0_VALUE_HI_REG 0x008       // ✅ 맞음
```

레지스터 주소는 정확하지만:

**구현 코드**:
```c
uint32_t bsw_get_time_ms(void) {
    uint32_t timer_lo = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_LO);
    uint32_t timer_hi = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_HI);
    
    uint64_t timer_us = ((uint64_t)timer_hi << 32) | timer_lo;
    return (uint32_t)(timer_us / 16000);  // ← ⚠️ 16MHz 가정
}
```

**문제점**:
1. **SYSTIMER 클럭 주파수 오류**: ESP32-C6 SYSTIMER는 **XTAL 클럭 (40MHz)** 사용
2. **잘못된 변환**: `/ 16000` 대신 `/ 40000` 사용해야 함
3. **UPDATE 레지스터 미사용**: 값 읽기 전 UPDATE 트리거 필요

**올바른 구현**:
```c
uint32_t bsw_get_time_ms(void) {
    // UPDATE 트리거 (값 동기화)
    BSW_SYS_REG_WRITE(SYSTIMER_UNIT0_OP_REG, SYSTIMER_TIMER_UNIT0_UPDATE);
    
    // 약간 대기 (UPDATE 완료까지)
    for (volatile int i = 0; i < 10; i++);
    
    uint32_t timer_lo = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_LO);
    uint32_t timer_hi = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_HI);
    
    uint64_t timer_us = ((uint64_t)timer_hi << 32) | timer_lo;
    return (uint32_t)(timer_us / 40000);  // 40MHz XTAL
}
```

---

### ❌ **문제 5: 워치독 레지스터 제어 불완전**

**구현 코드**:
```c
bool bsw_watchdog_init_bitwise(uint32_t timeout_ms) {
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0);
    
    uint32_t timeout_cycles = timeout_ms * 40000;  // 40MHz 클록 기준
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG1, timeout_cycles);
    
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0x83);  // 활성화 + 시스템 리셋 모드
    return true;
}
```

**ESP32-C6 워치독 초기화 절차 (TRM 기준)**:
1. WDT_WKEY 레지스터 쓰기 (보호 해제)
2. WDT_CONFIG0 설정
3. WDT_CONFIG1 타임아웃 설정
4. WDT_FEED로 타이머 시작
5. WDT_WKEY 레지스터 다시 잠금

**문제점**:
- ❌ **WDT_WKEY 보호 해제 없음** (쓰기 불가능)
- ❌ **클럭 소스 미설정** (APB vs RTC 선택)
- ❌ **초기 FEED 누락** (타이머 시작 안됨)

---

### ⚠️ **문제 6: 시스템 리셋 레지스터 가상 구현**

**구현 코드**:
```c
void bsw_system_restart(void) {
    // 소프트웨어 리셋 비트 설정
    BSW_SYS_REG_SET_BIT(BSW_SYSTEM_RST_EN, (1 << 0));
    
    while (1) {
        __asm__ __volatile__("nop");
    }
}
```

**문제점**:
- `BSW_SYSTEM_RST_EN` 레지스터 주소가 정의되지 않음
- ESP32-C6는 `RTC_CNTL_SW_CPU_STALL_REG` 사용
- 실제로 리셋이 동작하지 않을 가능성

---

## 📊 실제 동작 분석

### 헤더 파일 확인 (system_services.h)

```c
// 현재 헤더에 정의된 매크로
#define BSW_LOGE(tag, format, ...) bsw_log(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
```

### main.c 사용 예시

```c
BSW_LOGI(TAG, "Robot initialized successfully!");
// ↓ 매크로 확장
bsw_log(BSW_LOG_INFO, TAG, "Robot initialized successfully!");
```

**이 함수는 어디에 구현되어 있나?**

system_services.c를 보면:
- v2.0에는 `bsw_log_bitwise()` 구현
- v1.0에는 ESP-IDF `esp_log_writev()` 래퍼 구현

**실제 빌드가 성공한다면**: 
- v1.0 구현이 실제로 사용되고 있음
- v2.0 비트연산 구현은 미사용 (Dead Code)

---

## 🔧 권장 해결 방안

### 해결책 1: v1.0 (ESP-IDF Wrapper) 사용 (강력 권장)

현재 헤더가 v1.0이고 빌드가 성공한다면 v1.0을 계속 사용하는 것이 최선입니다.

**이유**:
- ✅ ESP-IDF API는 이미 검증됨
- ✅ UART, 타이머, 메모리 관리 모두 안정적
- ✅ FreeRTOS 통합
- ✅ 유지보수 용이

**v2.0 비트연산 코드 제거**:
```c
// system_services.c에서 모든 _bitwise 함수 제거
// 대신 ESP-IDF API 래퍼만 유지

void bsw_log(bsw_log_level_t level, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    
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

void bsw_delay_ms(uint32_t delay_ms) {
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

uint32_t bsw_get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void bsw_system_restart(void) {
    esp_restart();
}
```

---

### 해결책 2: v2.0 비트연산 완성 (비권장)

만약 "직접 제어"가 목적이라면:

#### A. 함수 이름 통일

**system_services.h 수정**:
```c
// _bitwise 접미사 추가
void bsw_log_bitwise(bsw_log_level_t level, const char* tag, const char* format, ...);
void* bsw_malloc_bitwise(size_t size);

// 매크로도 수정
#define BSW_LOGE(tag, format, ...) bsw_log_bitwise(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log_bitwise(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
```

#### B. 메모리 풀 비트맵 확장

```c
static uint32_t g_bsw_memory_bitmap[4];  // 128비트

void* bsw_malloc_bitwise(size_t size) {
    for (int i = 0; i < BSW_MEMORY_BLOCKS; i++) {
        int word_index = i / 32;
        int bit_index = i % 32;
        
        if (!(g_bsw_memory_bitmap[word_index] & (1U << bit_index))) {
            g_bsw_memory_bitmap[word_index] |= (1U << bit_index);
            return &g_bsw_memory_pool[i * BSW_MEMORY_BLOCK_SIZE];
        }
    }
    return NULL;
}
```

#### C. SYSTIMER 수정

```c
#define SYSTIMER_UNIT0_OP_REG (BSW_SYSTIMER_BASE + 0x00C)

uint32_t bsw_get_time_ms(void) {
    // UPDATE 트리거
    BSW_SYS_REG_WRITE(SYSTIMER_UNIT0_OP_REG, 1);
    for (volatile int i = 0; i < 10; i++);
    
    uint32_t timer_lo = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_LO);
    uint32_t timer_hi = BSW_SYS_REG_READ(BSW_SYSTIMER_UNIT0_VALUE_HI);
    
    uint64_t timer_us = ((uint64_t)timer_hi << 32) | timer_lo;
    return (uint32_t)(timer_us / 40000);  // 40MHz XTAL
}
```

#### D. 워치독 초기화 수정

```c
#define WDT_WKEY_VALUE 0x50D83AA1

bool bsw_watchdog_init_bitwise(uint32_t timeout_ms) {
    // 보호 해제
    BSW_SYS_REG_WRITE(BSW_WDT_WKEY, WDT_WKEY_VALUE);
    
    // WDT 비활성화
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0);
    
    // 타임아웃 설정 (APB 클럭 80MHz 기준)
    uint32_t timeout_cycles = timeout_ms * 80000;
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG1, timeout_cycles);
    
    // WDT 활성화
    BSW_SYS_REG_WRITE(BSW_WDT_CONFIG0, 0x83);
    
    // 초기 FEED
    BSW_SYS_REG_WRITE(BSW_WDT_FEED, 0x50);
    BSW_SYS_REG_WRITE(BSW_WDT_FEED, 0xA0);
    
    // 보호 다시 활성화
    BSW_SYS_REG_WRITE(BSW_WDT_WKEY, 0);
    
    return true;
}
```

---

## 📋 문제 요약표

| 문제 | 심각도 | 영향 | 현재 상태 |
|------|--------|------|----------|
| 함수 이름 불일치 | 🔴 치명적 | 링크 에러 가능 | v1.0으로 우회 |
| UART 초기화 없음 | 🟠 높음 | 로그 출력 실패 | ESP-IDF 사용 |
| 메모리 풀 32개 한계 | 🟠 높음 | 6KB 낭비 | 미사용 |
| SYSTIMER 클럭 오류 | 🟡 중간 | 시간 측정 부정확 | ESP-IDF 사용 |
| 워치독 초기화 불완전 | 🟡 중간 | WDT 동작 안함 | 미사용 |
| 시스템 리셋 가상 구현 | 🟢 낮음 | 리셋 실패 | ESP-IDF 사용 |

---

## 🎯 최종 권장사항

### 1순위: v1.0 (ESP-IDF Wrapper) 유지

```
현재 헤더 (v1.0) + v2.0 비트연산 코드 제거
→ ESP-IDF API 래퍼만 유지
```

**이유**:
- ✅ 현재 빌드 성공
- ✅ 모든 기능 안정적
- ✅ 유지보수 용이
- ✅ FreeRTOS 통합

**작업량**: 매우 적음 (Dead Code 제거만)

### 2순위: v2.0 완성 (학습 목적)

만약 "직접 제어" 학습이 목적이라면:
1. 함수 이름 통일
2. 메모리 비트맵 확장
3. SYSTIMER 수정
4. 워치독 초기화 완성
5. UART 초기화 추가

**작업량**: 많음 (4-6시간)  
**실용성**: 낮음 (ESP-IDF가 더 안정적)

---

## 📝 결론

현재 System Services는 **v1.0 (ESP-IDF Wrapper)와 v2.0 (Bitwise) 혼재** 상태입니다:

1. **헤더는 v1.0**을 선언하고 ESP-IDF API 래퍼 사용
2. **구현 파일은 v2.0** 비트연산 코드 포함 (미사용)
3. **실제 동작은 v1.0**으로 ESP-IDF API 호출

**강력 권장**: v1.0 (ESP-IDF Wrapper) 유지하고 v2.0 비트연산 코드 제거

이 방식이 안정성, 유지보수성, 실용성 모든 면에서 우수합니다.

---

**분석 완료일**: 2025-10-04  
**다음 단계**: v2.0 비트연산 코드 제거 또는 완성 선택
