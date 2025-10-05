# BSW (Basic Software) 종합 상태 리포트

**프로젝트**: ESP32-C6 BalanceBot  
**ESP-IDF**: v5.5  
**FreeRTOS**: v10+  
**작성일**: 2025-01-XX  
**작성자**: GitHub Copilot

---

## 📊 전체 드라이버 상태 요약

| 드라이버 | 버전 | FreeRTOS 안전성 | CPU Yielding | 비동기 지원 | 상태 | 비고 |
|---------|------|----------------|--------------|------------|------|------|
| **UART** | v4.0 | ✅ Mutex (per-port) | ✅ vTaskDelay() | ✅ RX polling task, Interrupt, DMA | ✅ **업그레이드 완료** | Ring buffer (512B), Priority 6 task |
| **SPI** | v2.0 | ✅ Mutex (per-port) | ✅ vTaskDelay() | ✅ DMA | ✅ **업그레이드 완료** | CS control with mutex |
| **I2C** | v6.0 | ✅ Mutex (per-port) | ⚠️ esp_rom_delay_us() | ⚠️ 없음 | ⚠️ **부분 안전** | Mutex는 있지만 CPU blocking 존재 |
| **ADC** | v1.0 | ✅ Mutex (single) | ⚠️ esp_rom_delay_us() | ❌ 없음 | ⚠️ **부분 안전** | Oneshot mode, Mutex는 있지만 CPU blocking |
| **GPIO** | v2.0 | ❌ 없음 | ❌ 없음 | ❌ 없음 | ❌ **미적용** | Direct register control, atomic 연산 필요 |
| **BLE** | v2.0 | ✅ ESP-IDF 내장 | N/A | ✅ NimBLE stack | ✅ **안전** | NimBLE 자체적으로 FreeRTOS 지원 |
| **PWM** | v4.0 | ✅ GPTIMER ISR | ✅ Direct GPIO writes | ✅ Hardware timer | ✅ **안전** | GPTIMER ISR 기반, 10μs period |
| **System Services** | v2.0 | ✅ Mutex | ✅ FreeRTOS APIs | ✅ Memory pool, UART log | ✅ **안전** | Core services with bitwise logging |

---

## ✅ 업그레이드 완료 드라이버 (2개)

### 1. UART Driver v4.0
**파일**: `main/bsw/uart_driver.c`  
**업그레이드 날짜**: 2025-01-XX  
**변경 사항**:
- ✅ Per-port mutex 추가: `uart_mutex[BSW_UART_PORT_MAX]`
- ✅ Ring buffer 구현: `uart_ring_buffer_t` (512 bytes)
- ✅ RX polling task: 5ms 주기, Priority 6, FIFO→Ring buffer 복사
- ✅ Interrupt-based RX: `uart_enable_rx_interrupt()`, `uart_isr_handler()`
- ✅ CPU yielding: `uart_delay_us()` → conditional vTaskDelay()
- ✅ DMA 지원: `uart_write_dma()`, `uart_wait_tx_done()`
- ✅ Resource cleanup: `uart_driver_deinit()` - mutex/task deletion

**주요 함수 변경**:
- `uart_driver_init_config()`: Mutex 생성 + RX polling task 시작
- `uart_read_data()`: FIFO → Ring buffer read
- `uart_write_data()`: Mutex-protected + CPU yielding in FIFO wait
- `uart_wait_trans_complete()`: `vTaskDelay(1)` 사용

**테스트 상태**: ⚠️ 빌드 대기 중

---

### 2. SPI Driver v2.0
**파일**: `main/bsw/spi_driver.c`  
**업그레이드 날짜**: 2025-01-XX  
**변경 사항**:
- ✅ Per-port mutex 추가: `spi_mutex[BSW_SPI_PORT_MAX]`
- ✅ CPU yielding: `esp_rom_delay_us(10)` → `vTaskDelay(1)`
- ✅ DMA 지원: `bsw_spi_transfer_dma()`, `bsw_spi_wait_dma_done()`
- ✅ CS control: `bsw_spi_cs_select()` acquires mutex, `bsw_spi_cs_deselect()` releases
- ✅ Resource cleanup: `bsw_spi_deinit()` - mutex deletion

**주요 함수 변경**:
- `bsw_spi_init()`: Mutex 생성 (line 118-123)
- `spi_wait_trans_complete()`: `TickType_t` + `xTaskGetTickCount()` + `vTaskDelay(1)`
- `bsw_spi_transfer_block()`: Full mutex wrapping (line 308-354)
- `bsw_spi_cs_select()`: Acquires mutex (line 374)
- `bsw_spi_cs_deselect()`: Releases mutex (line 391)

**테스트 상태**: ⚠️ 빌드 대기 중

---

## ⚠️ 부분 안전 드라이버 (2개 - CPU Blocking 문제)

### 3. I2C Driver v6.0 (Enhanced Production Ready)
**파일**: `main/bsw/i2c_driver.c` (870줄)  
**현재 상태**: Mutex는 있지만 CPU blocking 존재

**FreeRTOS 안전성**:
- ✅ Line 60: `static SemaphoreHandle_t i2c_mutex[BSW_I2C_PORT_MAX]`
- ✅ Line 325: `xSemaphoreCreateMutex()`
- ✅ Line 424, 504, 627: `xSemaphoreTake(i2c_mutex[port], ...)` - Mutex 사용 확인
- ✅ Line 476, 531, 652: `xSemaphoreGive(i2c_mutex[port])` - Mutex 해제 확인

**문제점**:
- ⚠️ Line 107-140: `i2c_wait_trans_complete()` - `esp_rom_delay_us(10)` CPU blocking
- ⚠️ Line 236, 242, 245: `i2c_bus_recovery()` - `esp_rom_delay_us(5)` CPU blocking

**개선 권장사항**:
1. `i2c_wait_trans_complete()`: `vTaskDelay(1)`로 변경 (UART/SPI 패턴)
2. `i2c_bus_recovery()`: Delay가 5μs로 매우 짧음 → ROM delay 유지 가능 (선택적)

**우선순위**: **중간** (Mutex는 있지만, IMU 센서 폴링 시 CPU blocking 발생 가능)

---

### 4. ADC Driver v1.0
**파일**: `main/bsw/adc_driver.c` (388줄)  
**현재 상태**: Mutex는 있지만 CPU blocking 존재

**FreeRTOS 안전성**:
- ✅ Line 37: `static SemaphoreHandle_t adc_mutex = NULL`
- ✅ Line 134: `xSemaphoreCreateMutex()`
- ✅ Line 141, 220, 282, 359: `xSemaphoreTake(adc_mutex, ...)` - Mutex 사용 확인
- ✅ Line 144, 160, 222, 284: `xSemaphoreGive(adc_mutex)` - Mutex 해제 확인

**문제점**:
- ⚠️ Line 71-95: `adc_wait_conversion_done()` - `esp_rom_delay_us(10)` CPU blocking
- ⚠️ Oneshot mode: 매번 변환 대기 시 CPU 점유

**개선 권장사항**:
1. `adc_wait_conversion_done()`: `vTaskDelay(1)`로 변경 (UART/SPI 패턴)
2. 선택적: Continuous mode + DMA 추가 (배터리 전압 모니터링용)

**우선순위**: **중간** (배터리 모니터링은 저빈도, 하지만 CPU blocking은 개선 권장)

---

## ❌ 미적용 드라이버 (1개 - 업그레이드 필요)

### 5. GPIO Driver v2.0
**파일**: `main/bsw/gpio_driver.c` (725줄)  
**현재 상태**: FreeRTOS 안전성 미적용

**분석 결과**:
- ❌ Mutex 없음: `grep` 검색 결과 0개
- ✅ FreeRTOS 헤더는 포함: Line 23-25 (freertos/FreeRTOS.h, task.h)
- ✅ Direct register control: HAL 사용으로 성능 최적화
- ⚠️ Flash pin protection: GPIO 26-30 예약 (ESP32-C6 Flash)

**위험성 평가**:
- **HIGH**: GPIO는 여러 태스크에서 동시 접근 가능 (motor, LED, sensor)
- Race condition 가능: Pin config, direction, level 설정 시 충돌
- Register read-modify-write 패턴: Atomic 연산 없이는 위험

**개선 필요 사항**:
1. ✅ Per-port mutex 추가: `gpio_mutex` (권장: 32개 GPIO용 단일 mutex)
2. ✅ Mutex wrapping for:
   - `bsw_gpio_config_pin()`
   - `bsw_gpio_set_level()`
   - `bsw_gpio_set_direction()`
3. ⚠️ `bsw_gpio_get_level()`: Read-only이므로 mutex 불필요 (선택적)

**우선순위**: **높음** (여러 태스크에서 공유되는 리소스, race condition 위험)

---

## ✅ 이미 안전한 드라이버 (3개)

### 6. BLE Driver v2.0 (NimBLE Stack)
**파일**: `main/bsw/ble_driver.c` (578줄)  
**안전성 근거**:
- ✅ NimBLE Stack: Apache Mynewt의 검증된 BLE 구현
- ✅ ESP-IDF v5.5: `nimble/nimble_port_freertos.h` 사용
- ✅ NimBLE 자체적으로 FreeRTOS 멀티태스킹 지원
- ✅ Event callback 메커니즘: Thread-safe 보장
- ✅ Host task: `ble_host_task()` - FreeRTOS task로 실행

**결론**: 추가 업그레이드 불필요

---

### 7. PWM Driver v4.0
**파일**: `main/bsw/pwm_driver.c`  
**안전성 근거**:
- ✅ GPTIMER ISR 기반: 10μs period, 8 channels
- ✅ Direct GPIO register writes: Atomic 연산
- ✅ Hardware timer: CPU에서 독립적으로 동작
- ✅ Motor control optimized

**결론**: 추가 업그레이드 불필요

---

### 8. System Services v2.0
**파일**: `main/system/system_services.c`  
**안전성 근거**:
- ✅ FreeRTOS mutex 사용
- ✅ Memory pool with protection
- ✅ UART logging with bitwise register control
- ✅ Watchdog timer management
- ✅ Core services for all drivers

**결론**: 추가 업그레이드 불필요

---

## 🎯 업그레이드 우선순위 및 작업량 추정

### 우선순위 1: GPIO Driver v2.0 → v3.0 (즉시)
**이유**: 
- 여러 태스크에서 공유 (motor, LED, sensor)
- Race condition 위험 **높음**
- Mutex 완전 미적용 상태

**예상 작업 시간**: 1-2시간  
**변경 범위**:
- Mutex 추가: `static SemaphoreHandle_t gpio_mutex = NULL`
- `bsw_gpio_init()`: Mutex 생성
- `bsw_gpio_config_pin()`: Mutex wrapping
- `bsw_gpio_set_level()`: Mutex wrapping
- `bsw_gpio_set_direction()`: Mutex wrapping
- `bsw_gpio_deinit()`: Mutex cleanup (추가)

**참고 가이드**: UART_V4_UPGRADE_GUIDE.md의 Mutex 패턴

---

### 우선순위 2: I2C Driver v6.0 → v6.1 (권장)
**이유**: 
- IMU 센서 폴링 (MPU6050 @ 400kHz)
- 50Hz 제어 루프에서 사용
- CPU blocking으로 인한 타이밍 지연 가능

**예상 작업 시간**: 30분  
**변경 범위**:
- `i2c_wait_trans_complete()`: `esp_rom_delay_us(10)` → `vTaskDelay(1)`
- (선택적) `i2c_bus_recovery()`: 5μs delay는 유지 가능 (너무 짧음)

**참고 가이드**: SPI_V2_UPGRADE_GUIDE.md의 vTaskDelay() 패턴

---

### 우선순위 3: ADC Driver v1.0 → v1.1 (선택적)
**이유**: 
- 배터리 전압 모니터링 (저빈도, ~1Hz)
- CPU blocking 영향은 낮지만 일관성 유지

**예상 작업 시간**: 30분  
**변경 범위**:
- `adc_wait_conversion_done()`: `esp_rom_delay_us(10)` → `vTaskDelay(1)`
- (선택적) Continuous mode + DMA 추가

**참고 가이드**: UART_V4_UPGRADE_GUIDE.md의 vTaskDelay() 패턴

---

## 📈 업그레이드 전후 비교

| 항목 | 업그레이드 전 | 업그레이드 후 |
|------|-------------|-------------|
| **FreeRTOS 안전 드라이버** | 5/8 (62.5%) | 8/8 (100%) |
| **CPU Blocking 제거** | 3개 드라이버 | 0개 |
| **Race Condition 위험** | GPIO (HIGH) | 없음 |
| **비동기 지원** | UART, BLE, PWM | UART, SPI, BLE, PWM, I2C (향상) |
| **Mutex 커버리지** | 6/8 | 8/8 |

---

## 🔨 통합 업그레이드 패턴 (UART v4.0 + SPI v2.0 기반)

### 패턴 1: Per-port Mutex 추가
```c
// 헤더에 FreeRTOS 포함
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Static mutex 선언
static SemaphoreHandle_t xxx_mutex[MAX_PORTS] = {NULL};

// Init 함수에서 mutex 생성
esp_err_t bsw_xxx_init(bsw_xxx_port_t port, ...) {
    if (xxx_mutex[port] == NULL) {
        xxx_mutex[port] = xSemaphoreCreateMutex();
        if (xxx_mutex[port] == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    // ... initialization ...
}

// 모든 read/write 함수에 mutex wrapping
esp_err_t bsw_xxx_operation(bsw_xxx_port_t port, ...) {
    if (xxx_mutex[port] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Acquire mutex with timeout
    if (xSemaphoreTake(xxx_mutex[port], pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // ... critical section ...
    
    // Release mutex
    xSemaphoreGive(xxx_mutex[port]);
    return ESP_OK;
}

// Deinit 함수에서 mutex 삭제
esp_err_t bsw_xxx_deinit(bsw_xxx_port_t port) {
    // ... cleanup ...
    
    if (xxx_mutex[port] != NULL) {
        vSemaphoreDelete(xxx_mutex[port]);
        xxx_mutex[port] = NULL;
    }
    return ESP_OK;
}
```

---

### 패턴 2: CPU Blocking 제거 (vTaskDelay)
```c
// BEFORE (CPU blocking)
static esp_err_t xxx_wait_complete(uint32_t timeout_ms) {
    uint32_t start = esp_timer_get_time() / 1000;
    while (1) {
        // Check status register
        if (HW_REG & STATUS_BIT) {
            return ESP_OK;
        }
        
        // Timeout check
        if ((esp_timer_get_time() / 1000 - start) >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        
        esp_rom_delay_us(10);  // ❌ CPU blocking!
    }
}

// AFTER (CPU yielding)
static esp_err_t xxx_wait_complete(uint32_t timeout_ms) {
    TickType_t start_tick = xTaskGetTickCount();
    
    while (1) {
        // Check status register
        if (HW_REG & STATUS_BIT) {
            return ESP_OK;
        }
        
        // Timeout check
        TickType_t elapsed = xTaskGetTickCount() - start_tick;
        if (pdTICKS_TO_MS(elapsed) >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        
        vTaskDelay(1);  // ✅ CPU yielding to other tasks!
    }
}
```

---

### 패턴 3: 조건부 Delay (Micro vs. Task)
```c
// UART v4.0 패턴: 1ms 미만은 ROM delay, 이상은 vTaskDelay
static void conditional_delay(uint32_t us) {
    if (us < 1000) {
        esp_rom_delay_us(us);  // < 1ms: ROM delay (정밀도 우선)
    } else {
        vTaskDelay(pdMS_TO_TICKS(us / 1000));  // >= 1ms: FreeRTOS delay (CPU 양보)
    }
}
```

---

## 📝 다음 단계

### 즉시 실행 (우선순위 1)
1. **GPIO Driver v2.0 → v3.0 업그레이드**
   - Mutex 추가 및 wrapping
   - 빌드 및 테스트
   - GPIO_V3_UPGRADE_GUIDE.md 작성

### 권장 실행 (우선순위 2-3)
2. **I2C Driver v6.0 → v6.1 업그레이드**
   - `vTaskDelay()` 적용
   - 빌드 및 테스트

3. **ADC Driver v1.0 → v1.1 업그레이드**
   - `vTaskDelay()` 적용
   - 빌드 및 테스트

### 통합 테스트
4. **전체 BSW 빌드 및 검증**
   - `idf.py build` 실행
   - 메모리 사용량 확인 (RAM/Flash)
   - 컴파일 에러 해결

5. **하드웨어 테스트**
   - ESP32-C6 DevKitC-1 플래싱
   - 50Hz 제어 루프 타이밍 검증
   - GPS/IMU/Motor 동시 동작 테스트

---

## 📚 참고 문서

- **UART_V4_UPGRADE_GUIDE.md**: UART v3.0 → v4.0 상세 가이드 (Ring buffer, RX polling task, DMA)
- **SPI_V2_UPGRADE_GUIDE.md**: SPI v1.0 → v2.0 상세 가이드 (Mutex, vTaskDelay, DMA)
- **ESP32-C6 Technical Reference Manual**: Hardware register 명세
- **ESP-IDF v5.5 FreeRTOS API Reference**: Task, Semaphore, Mutex API

---

## 🎓 교훈

1. **일관성 유지**: 모든 드라이버에 동일한 FreeRTOS 안전성 패턴 적용
2. **점진적 업그레이드**: UART → SPI → GPIO → I2C/ADC 순서로 진행
3. **테스트 우선**: 각 드라이버 업그레이드 후 즉시 빌드 검증
4. **문서화**: 업그레이드 가이드 작성으로 향후 유지보수 용이
5. **하드웨어 제약 고려**: ESP32-C6의 I2C1 부재, Flash pin 예약 등 명확히 문서화

---

**작성 완료**: 2025-01-XX  
**다음 업데이트**: GPIO v3.0 업그레이드 후
