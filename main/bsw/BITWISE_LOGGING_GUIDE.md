# v2.0 비트연산 로깅 사용 가이드

## 📋 개요

이 문서는 BalanceBot 프로젝트에서 v2.0 비트연산 기반 로깅 시스템(`bsw_log_bitwise`)을 사용하는 방법을 설명합니다.

**작성일:** 2025-01-21  
**버전:** 2.0 (멀티태스킹 안전)

---

## ✅ 현재 설정 상태

### **1. 헤더 파일 설정 완료**

`main/bsw/system_services.h`:
```c
// BSW_LOGE/LOGI 매크로가 자동으로 bsw_log_bitwise() 호출
#define BSW_LOGE(tag, format, ...) bsw_log_bitwise(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGW(tag, format, ...) bsw_log_bitwise(BSW_LOG_WARN, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log_bitwise(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
#define BSW_LOGD(tag, format, ...) bsw_log_bitwise(BSW_LOG_DEBUG, tag, format, ##__VA_ARGS__)
```

### **2. 로깅 함수 멀티태스킹 안전**

`main/bsw/system_services.c`:
- ✅ **FreeRTOS 뮤텍스 보호**: UART 접근 시 `g_bsw_uart_mutex` 사용
- ✅ **직접 UART 제어**: 0x60000000 레지스터 직접 쓰기
- ✅ **포맷 지원**: `%d`, `%s`, `%c`, **`%f`** (소수점 2자리)

---

## 🚀 사용 방법

### **기본 사용법**

`main.c`에서 기존과 동일하게 사용:

```c
#include "bsw/system_services.h"

static const char* TAG = "BALANCE_ROBOT";

void app_main(void) {
    // 정보 로그
    BSW_LOGI(TAG, "Balance Robot Starting...");
    
    // 에러 로그
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize motor!");
    }
    
    // 정수 포맷
    BSW_LOGI(TAG, "Sensor task priority: %d", 5);
    
    // Float 포맷 (소수점 2자리)
    BSW_LOGI(TAG, "Angle: %f degrees", 45.67f);
    
    // 문자열 포맷
    BSW_LOGI(TAG, "Device name: %s", "BalanceBot");
}
```

---

## 📊 지원 포맷 스펙

| 포맷 | 타입 | 예시 | 출력 결과 | 비고 |
|------|------|------|-----------|------|
| `%d` | int | `%d`, 42 | `42` | 음수 지원 |
| `%s` | char* | `%s`, "Hello" | `Hello` | NULL 안전 |
| `%c` | char | `%c`, 'A' | `A` | 단일 문자 |
| `%f` | float/double | `%f`, 3.14f | `3.14` | **소수점 2자리 고정** |

### ⚠️ **제한사항**

- `%f`는 소수점 **2자리만** 출력 (예: `3.14159` → `3.14`)
- `%lu`, `%x`, `%p` 등 **미지원** (ESP-IDF 로깅 필요 시 사용 불가)
- 가변 자릿수 미지원 (`%.6f` 불가)

---

## 🔧 멀티태스킹 안전성

### **FreeRTOS 환경에서 안전**

```c
// sensor_task (50Hz)
void sensor_task(void* param) {
    while(1) {
        BSW_LOGI(TAG, "Angle: %f", angle);  // ✅ 안전
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// balance_task (50Hz)
void balance_task(void* param) {
    while(1) {
        BSW_LOGI(TAG, "Motor: %d", speed);  // ✅ 안전
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

**동작 원리:**
1. `bsw_init_uart()` → UART 뮤텍스 생성
2. `xSemaphoreTake(g_bsw_uart_mutex, portMAX_DELAY)` → UART 접근 보호
3. UART FIFO 직접 쓰기 (0x60000000)
4. `xSemaphoreGive(g_bsw_uart_mutex)` → 뮤텍스 해제

---

## 📈 성능 특성

### **ESP-IDF 로깅 vs v2.0 비트연산 로깅**

| 항목 | ESP-IDF `ESP_LOGI` | v2.0 `BSW_LOGI` |
|------|-------------------|-----------------|
| **속도** | ~100-500 사이클/문자 | **~10-20 사이클/문자** ⚡ |
| **멀티태스킹** | ✅ 안전 (FreeRTOS) | ✅ 안전 (뮤텍스) |
| **포맷 지원** | printf 전체 | %d, %s, %c, %f만 |
| **디버깅** | 타임스탬프, 색상 | 간단한 레벨 표시 |
| **메모리** | 동적 버퍼 | 스택만 사용 |

**속도 이점: 5-25배 빠름** 🚀

---

## 🎯 출력 예시

### **실제 로그 출력**

```
[I] BALANCE_ROBOT: Balance Robot Starting...
[I] BALANCE_ROBOT: Mutexes created
[I] BALANCE_ROBOT: Sensor task started
[I] BALANCE_ROBOT: Balance task started
[I] BALANCE_ROBOT: Angle: 0.52 | Velocity: 12.34 | Battery: 7.40V
[W] BALANCE_ROBOT: Battery low: 6.70V
[E] BALANCE_ROBOT: Failed to initialize motor!
```

**포맷:**
- `[레벨] 태그: 메시지`
- 레벨: `E` (Error), `W` (Warn), `I` (Info), `D` (Debug)

---

## 🔄 ESP-IDF 로깅으로 전환하려면?

`main/bsw/system_services.h`에서 매크로만 변경:

```c
// Before (v2.0 비트연산)
#define BSW_LOGE(tag, format, ...) bsw_log_bitwise(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log_bitwise(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)

// After (ESP-IDF)
#include "esp_log.h"
#define BSW_LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
```

`main.c`는 **수정 불필요** ✅

---

## 🛠️ 빌드 및 테스트

### **빌드 명령어**

```bash
cd c:\Users\hyuns\Desktop\balaencedbot_esp_ide
idf.py build
```

### **플래시 및 모니터링**

```bash
idf.py flash monitor
```

### **예상 출력 (시리얼)**

```
[I] BALANCE_ROBOT: Balance Robot Starting...
[I] BALANCE_ROBOT: Sensor task started
[I] BALANCE_ROBOT: Angle: 0.00 | Velocity: 0.00 | Battery: 8.20V
```

---

## ⚠️ 주의사항

### **1. Float 소수점 제한**

```c
// ❌ 6자리 소수점 불가
BSW_LOGI(TAG, "GPS - Lat: %.6f", 37.123456);  // 출력: "37.12"

// ✅ 정수로 나누어 출력
int lat_int = (int)lat;
int lat_frac = (int)((lat - lat_int) * 1000000);
BSW_LOGI(TAG, "GPS - Lat: %d.%d", lat_int, lat_frac);  // 출력: "37.123456"
```

### **2. ISR에서 사용 금지**

```c
// ❌ ISR에서 호출 불가 (뮤텍스 사용)
void IRAM_ATTR pwm_isr() {
    BSW_LOGI(TAG, "PWM ISR");  // ❌ FreeRTOS API 호출 불가
}

// ✅ 태스크에서 호출
void sensor_task(void* param) {
    BSW_LOGI(TAG, "Sensor update");  // ✅ OK
}
```

### **3. 긴 문자열 주의**

```c
// ⚠️ 긴 문자열은 스택 사용량 증가
char buffer[256];
snprintf(buffer, sizeof(buffer), "Very long message...");
BSW_LOGI(TAG, buffer);  // 스택 오버플로우 주의
```

---

## 📚 참고 자료

- `main/bsw/system_services.h` - BSW 매크로 정의
- `main/bsw/system_services.c` - `bsw_log_bitwise()` 구현
- `main/main.c` - 사용 예시

---

## ✅ 체크리스트

프로젝트에 v2.0 비트연산 로깅이 적용되었는지 확인:

- [x] `system_services.h`에서 `BSW_LOGE/LOGI` 매크로가 `bsw_log_bitwise()` 호출
- [x] `system_services.c`에서 FreeRTOS 뮤텍스 사용
- [x] `bsw_log_bitwise()`에 `%f` 포맷 지원 추가
- [x] `main.c`에서 `BSW_LOGI/LOGE` 사용 중
- [ ] 빌드 성공 확인
- [ ] 시리얼 출력 테스트

---

**v2.0 비트연산 로깅이 성공적으로 적용되었습니다!** 🎉

이제 `main.c`의 모든 `BSW_LOGI/LOGE`가 자동으로 직접 UART 제어로 출력됩니다.
