# PWM 드라이버 상세 분석 보고서

## 📋 분석 개요

**분석 대상**: `main/bsw/pwm_driver.c` / `pwm_driver.h`  
**드라이버 버전**: v4.0 (Bitwise Direct Control PWM)  
**분석 일시**: 2025-10-04  
**분석자**: GitHub Copilot  

---

## 🎯 현재 구조 분석

### 1. 설계 의도
이 PWM 드라이버는 **소프트웨어 PWM** 구현을 목표로 하고 있습니다:
- ESP32-C6 하드웨어 타이머를 1μs 주기로 인터럽트
- ISR 핸들러에서 각 채널의 GPIO를 직접 토글
- 최대 8채널 동시 지원
- 1000단계 해상도 (0.1% 단위)

### 2. 코드 구조

```c
// 채널 구조체 (8채널)
static pwm_channel_config_t pwm_channels[PWM_CHANNEL_MAX];

// ISR 핸들러 (1μs마다 호출 예정)
static void IRAM_ATTR pwm_timer_isr_handler(void) {
    PWM_TIMER_FAST_CLEAR_INT(PWM_TIMER_GROUP, PWM_TIMER_NUM);
    
    for (int ch = 0; ch < PWM_CHANNEL_MAX; ch++) {
        if (!pwm_channels[ch].enabled) continue;
        
        uint32_t cycle_position = timer_counter % pwm_channels[ch].period_us;
        
        if (cycle_position < pwm_channels[ch].high_time_us) {
            PWM_FAST_GPIO_HIGH(pwm_channels[ch].gpio_num);
        } else {
            PWM_FAST_GPIO_LOW(pwm_channels[ch].gpio_num);
        }
    }
    
    timer_counter++;
}
```

---

## 🚨 발견된 심각한 문제들

### ❌ **문제 1: ISR이 실제로 등록되지 않음**

**위치**: `pwm_timer_raw_interrupt_enable()` (line 421-432)

```c
void pwm_timer_raw_interrupt_enable(uint32_t group, uint32_t timer, bool enable) {
    if (enable) {
        TIMER_WRITE_REG(group, timer, TIMER_INT_CLR_REG_OFFSET, (1 << timer));
        TIMER_WRITE_REG(group, timer, TIMER_INT_ENA_REG_OFFSET, (1 << timer));
        
        // ISR 등록 (여기서는 간소화) ← ⚠️ 주석만 있고 실제 구현 없음!
        BSW_LOGI(PWM_TAG, "Timer interrupt enabled");
    } else {
        TIMER_WRITE_REG(group, timer, TIMER_INT_ENA_REG_OFFSET, 0);
    }
}
```

**문제점**:
- 인터럽트 레지스터 활성화만 했을 뿐, **CPU 인터럽트 컨트롤러에 ISR 등록이 없음**
- `esp_intr_alloc()` 호출이 누락되어 `pwm_timer_isr_handler()`가 **절대 호출되지 않음**
- 타이머가 동작해도 ISR이 실행되지 않으므로 **PWM 출력이 전혀 발생하지 않음**

**예상 결과**: 
✅ 빌드 성공  
❌ **실제 PWM 동작 완전히 실패** (GPIO 출력 변화 없음)

---

### ❌ **문제 2: 1μs 인터럽트는 CPU 과부하 유발**

**설계 의도**: 1MHz 타이머 (1μs 주기) 인터럽트로 소프트웨어 PWM 구현

**문제점**:
```
1μs마다 ISR 호출 = 1,000,000 인터럽트/초
8채널 처리 시:
- 각 채널 조건문 검사
- 나눗셈 연산 (% 연산자)
- GPIO 레지스터 쓰기
- 루프 오버헤드
```

**성능 계산**:
- ESP32-C6 CPU: 160MHz RISC-V
- 1μs = 160 사이클
- ISR 오버헤드 (스택 저장, 복귀): ~30-50 사이클
- 8채널 처리: 조건문(3사이클) × 8 + 나눗셈(~40사이클) + GPIO(10사이클) = **최소 100사이클/ISR**
- **CPU 점유율 = (100/160) × 100 = 62.5%**

**실제 영향**:
- ⚠️ CPU 60% 이상이 PWM ISR에만 사용
- 🐌 메인 로직 (PID, 센서 처리) 성능 저하
- ⏱️ FreeRTOS 스케줄링 지연
- 🔥 배터리 소모 급증

---

### ❌ **문제 3: ESP32-C6에는 하드웨어 LEDC PWM이 이미 존재**

ESP32-C6는 **6채널 하드웨어 LEDC PWM**을 내장하고 있습니다:

| 기능 | 하드웨어 LEDC | 소프트웨어 PWM (현재) |
|------|---------------|----------------------|
| CPU 점유율 | 0% | 62.5% |
| 채널 수 | 6 | 8 |
| 해상도 | 최대 20비트 | 1000단계 (10비트) |
| 정확도 | 하드웨어 보장 | 인터럽트 지터 발생 |
| 주파수 범위 | 1Hz ~ 40MHz | 제한적 |
| 동작 방식 | 독립 하드웨어 | CPU 인터럽트 |

**이미 존재하는 ESP-IDF LEDC API**:
```c
ledc_timer_config()     // 타이머 설정
ledc_channel_config()   // 채널 설정
ledc_set_duty()         // 듀티 사이클 변경
ledc_update_duty()      // 하드웨어 적용
```

**현재 코드의 문제**:
- 🔄 하드웨어 LEDC를 사용하지 않고 소프트웨어로 재구현
- ⚡ 높은 CPU 오버헤드로 성능 낭비
- 🎯 정확도가 하드웨어보다 떨어짐

---

### ❌ **문제 4: 타이머 레지스터 제어 불완전**

**위치**: `pwm_timer_raw_init()` (line 367-388)

```c
esp_err_t pwm_timer_raw_init(uint32_t group, uint32_t timer, uint32_t resolution_hz) {
    TIMER_CLEAR_BITS(group, timer, TIMER_CONFIG_REG_OFFSET, TIMER_EN_BIT);
    
    uint32_t divider = 80000000 / resolution_hz;  // ← ⚠️ APB 클럭 하드코딩
    if (divider > TIMER_DIVIDER_MASK) {
        divider = TIMER_DIVIDER_MASK;
    }
    
    uint32_t config_val = TIMER_AUTORELOAD_BIT | TIMER_INCREASE_BIT | 
                         (divider << TIMER_DIVIDER_SHIFT);
    TIMER_WRITE_REG(group, timer, TIMER_CONFIG_REG_OFFSET, config_val);
    // ...
}
```

**문제점**:
1. **APB 클럭 주파수 하드코딩**: 80MHz로 가정하지만 ESP32-C6는 가변 클럭
2. **클럭 소스 미설정**: `TIMER_CLK_SRC` 레지스터 설정 누락
3. **타이머 그룹 클럭 게이팅 미확인**: TIMG 클럭이 활성화되지 않을 수 있음
4. **에러 검증 부족**: 레지스터 쓰기 성공 여부 미확인

---

### ❌ **문제 5: GPIO Fast 매크로가 실제로는 느림**

**위치**: `pwm_driver.h` (line 218-220)

```c
#define PWM_FAST_GPIO_HIGH(gpio) bsw_gpio_fast_set_high(gpio)
#define PWM_FAST_GPIO_LOW(gpio)  bsw_gpio_fast_set_low(gpio)
```

이 매크로들은 `gpio_driver.c`의 함수를 호출하는데:

```c
void bsw_gpio_fast_set_high(bsw_gpio_num_t gpio_num) {
    if (gpio_num >= GPIO_USABLE_PIN_COUNT) return;  // ← 조건문 검사
    
    volatile uint32_t* out_set_reg = (volatile uint32_t*)(GPIO_OUT_W1TS_REG);
    *out_set_reg = (1U << gpio_num);
}
```

**문제점**:
- 🐌 함수 호출 오버헤드 (CALL/RET 명령어)
- 🔍 조건문 검사 (`if` 분기)
- 📍 포인터 역참조

**실제 "빠른" GPIO 제어**는 인라인 어셈블리여야 합니다:
```c
#define PWM_FAST_GPIO_HIGH(gpio) \
    (*((volatile uint32_t*)GPIO_OUT_W1TS_REG) = (1U << (gpio)))
```

---

### ⚠️ **문제 6: 서보 모터 각도 변환 로직 오류**

**위치**: `pwm_servo_set_angle()` (line 273-305)

```c
esp_err_t pwm_servo_set_angle(pwm_channel_t channel, uint16_t angle) {
    // ...
    // 서보 각도를 듀티 사이클로 변환
    // 50Hz PWM에서: 1ms = 5% duty, 2ms = 10% duty
    // 0도 = 1ms (50 duty), 180도 = 2ms (100 duty)
    uint32_t duty = 50 + (angle * 50) / 180;  // 50 ~ 100 범위
    
    esp_err_t ret = pwm_set_duty(channel, duty);
    // ...
}
```

**문제점**:
- 🎯 **1000단계 해상도인데 50-100 범위만 사용** (5-10%)
- 📐 서보 PWM은 50Hz, 주기 20ms:
  - 1ms = 5% (1ms/20ms)
  - 2ms = 10% (2ms/20ms)
- 계산 오류: `duty` 값이 PWM_RESOLUTION(1000) 기준이 아님

**올바른 계산**:
```c
// 1000단계 해상도 기준
// 0도: 1ms/20ms = 5% = 50/1000
// 180도: 2ms/20ms = 10% = 100/1000
uint32_t duty = 50 + (angle * 50) / 180;  // 현재 코드
// duty는 50-100 범위이지만, pwm_set_duty()는 0-1000 범위를 기대함
```

---

### ⚠️ **문제 7: 나눗셈 연산의 ISR 내 사용**

**위치**: `pwm_timer_isr_handler()` (line 56-81)

```c
static void IRAM_ATTR pwm_timer_isr_handler(void) {
    // ...
    for (int ch = 0; ch < PWM_CHANNEL_MAX; ch++) {
        // ...
        uint32_t cycle_position = timer_counter % pwm_channels[ch].period_us;  // ← 나눗셈!
        
        if (cycle_position < pwm_channels[ch].high_time_us) {
            PWM_FAST_GPIO_HIGH(pwm_channels[ch].gpio_num);
        } else {
            PWM_FAST_GPIO_LOW(pwm_channels[ch].gpio_num);
        }
    }
    
    timer_counter++;
}
```

**문제점**:
- **% (모듈로) 연산자는 소프트웨어 나눗셈** (ESP32-C6에는 하드웨어 나눗셈 없음)
- RISC-V에서 나눗셈: **최소 30-40 사이클**
- ISR에서 8채널 × 40사이클 = **320 사이클 낭비**
- 1μs = 160 사이클인데 ISR이 320 사이클 소요 → **타이밍 붕괴**

**해결책**:
- 채널별 카운터 사용 (나눗셈 제거)
- 비트 마스크 활용 (2^n 주기만 허용)

---

### ⚠️ **문제 8: 채널 활성화 상태 관리 부재**

**위치**: `pwm_set_enable()` (line 219-235)

```c
esp_err_t pwm_set_enable(pwm_channel_t channel, bool enable) {
    if (channel >= PWM_CHANNEL_MAX) {
        BSW_LOGE(PWM_TAG, "Invalid PWM channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    pwm_channels[channel].enabled = enable;
    
    if (!enable) {
        PWM_FAST_GPIO_LOW(pwm_channels[channel].gpio_num);
    }
    
    BSW_LOGI(PWM_TAG, "PWM channel %d %s", channel, enable ? "enabled" : "disabled");
    return ESP_OK;
}
```

**문제점**:
- 채널 활성화해도 **GPIO 출력 모드 확인 없음**
- `pwm_channel_init_freq()` 호출 여부 미검증
- `gpio_num = 0`인 경우 (미초기화) 처리 없음

---

## 📊 실제 동작 예상 시나리오

### 시나리오 1: 모터 제어 사용 시

```c
// main.c에서 모터 초기화
motor_control_init(&motor, GPIO_1, GPIO_2, GPIO_3, PWM_CHANNEL_0);
motor_control_set_speed(&motor, 128);  // 50% 속도
```

**실제 동작**:
1. ✅ `pwm_driver_init()` 호출됨
2. ✅ `pwm_timer_raw_init()` 타이머 레지스터 설정
3. ✅ `pwm_timer_raw_interrupt_enable()` 인터럽트 레지스터 활성화
4. ❌ **CPU 인터럽트 컨트롤러에 ISR 미등록**
5. ❌ `pwm_timer_isr_handler()` **절대 호출되지 않음**
6. ❌ GPIO 3번 핀 **전혀 토글되지 않음**
7. ❌ 모터 **완전히 정지 상태**

### 시나리오 2: 만약 ISR이 정상 등록되었다면?

```
타이머: 1,000,000 인터럽트/초
ISR 처리 시간: 320 사이클 = 2μs (160MHz 기준)

CPU 점유율: (2μs / 1μs) × 100 = 200% ← 불가능!
```

**결과**: 
- 🔥 ISR이 끝나기 전에 다음 인터럽트 발생
- ⚠️ 인터럽트 중첩 (Nested Interrupt)
- 💥 시스템 불안정, 워치독 리셋

---

## 🔧 권장 해결 방안

### 해결책 1: 하드웨어 LEDC PWM 사용 (강력 권장)

ESP32-C6의 내장 LEDC를 활용하는 것이 최선입니다:

```c
esp_err_t pwm_driver_init(void) {
    // LEDC 타이머 설정 (모터용)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,  // 1024단계
        .freq_hz = 5000,                        // 5kHz
        .clk_cfg = LEDC_AUTO_CLK
    };
    return ledc_timer_config(&ledc_timer);
}

esp_err_t pwm_channel_init_freq(bsw_gpio_num_t gpio, pwm_channel_t channel, uint32_t frequency) {
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = gpio,
        .duty = 0,
        .hpoint = 0
    };
    return ledc_channel_config(&ledc_channel);
}

esp_err_t pwm_set_duty(pwm_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}
```

**장점**:
- ✅ CPU 점유율 0%
- ✅ 하드웨어 정밀도 보장
- ✅ ESP-IDF 공식 지원
- ✅ 6채널 독립 제어
- ✅ 전력 효율 최고

**단점**:
- 6채널 제한 (현재 8채널 설계)

---

### 해결책 2: 소프트웨어 PWM 개선 (비권장)

만약 8채널이 반드시 필요하다면:

#### A. ISR 등록 추가

```c
#include "driver/timer.h"

esp_err_t pwm_timer_raw_init(uint32_t group, uint32_t timer, uint32_t resolution_hz) {
    // 타이머 설정은 동일
    // ...
    
    // ISR 등록 추가
    esp_err_t ret = timer_isr_register(
        group, 
        timer, 
        pwm_timer_isr_handler,  // ISR 함수
        NULL,                   // 인자
        ESP_INTR_FLAG_IRAM,     // IRAM에 ISR 배치
        NULL                    // 핸들
    );
    
    if (ret != ESP_OK) {
        BSW_LOGE(PWM_TAG, "Failed to register timer ISR");
        return ret;
    }
    
    return ESP_OK;
}
```

#### B. 인터럽트 주기 증가 (10μs → 10kHz)

```c
#define PWM_TIMER_DIVIDER  800  // 80MHz / 800 = 100kHz (10μs)

// 알람 주기: 10μs
pwm_timer_raw_set_alarm(PWM_TIMER_GROUP, PWM_TIMER_NUM, 1000);
```

**효과**:
- CPU 점유율: 62.5% → 6.25%
- PWM 해상도: 100단계 (1% 단위)

#### C. 나눗셈 제거

```c
static volatile uint32_t channel_counters[PWM_CHANNEL_MAX];

static void IRAM_ATTR pwm_timer_isr_handler(void) {
    PWM_TIMER_FAST_CLEAR_INT(PWM_TIMER_GROUP, PWM_TIMER_NUM);
    
    for (int ch = 0; ch < PWM_CHANNEL_MAX; ch++) {
        if (!pwm_channels[ch].enabled) continue;
        
        // 나눗셈 대신 카운터 증가
        if (channel_counters[ch] < pwm_channels[ch].high_time_us) {
            PWM_FAST_GPIO_HIGH(pwm_channels[ch].gpio_num);
        } else {
            PWM_FAST_GPIO_LOW(pwm_channels[ch].gpio_num);
        }
        
        channel_counters[ch]++;
        if (channel_counters[ch] >= pwm_channels[ch].period_us) {
            channel_counters[ch] = 0;
        }
    }
}
```

---

### 해결책 3: 하이브리드 방식

- **모터 제어 (4채널)**: 하드웨어 LEDC 사용
- **서보 모터 (2채널)**: 하드웨어 LEDC 사용 (50Hz 타이머)
- **추가 2채널**: 소프트웨어 PWM (저주파수만)

```c
// 모터용 LEDC (타이머 0)
pwm_channel_init_freq(GPIO_1, PWM_CHANNEL_0, 5000);  // 5kHz
pwm_channel_init_freq(GPIO_2, PWM_CHANNEL_1, 5000);

// 서보용 LEDC (타이머 1)
pwm_servo_init(GPIO_3, PWM_CHANNEL_2);  // 50Hz

// 추가 소프트웨어 PWM (타이머 인터럽트)
pwm_channel_init_freq(GPIO_4, PWM_CHANNEL_6, 100);  // 100Hz만
```

---

## 📋 문제 요약표

| 문제 | 심각도 | 영향 | 현재 상태 |
|------|--------|------|----------|
| ISR 미등록 | 🔴 치명적 | PWM 완전 미동작 | 빌드 성공, 실행 실패 |
| CPU 과부하 | 🔴 치명적 | 62.5% 점유율 | 설계 단계 오류 |
| 하드웨어 LEDC 미사용 | 🟠 높음 | 성능 낭비 | 재설계 필요 |
| 타이머 설정 불완전 | 🟠 높음 | 동작 불안정 | 보완 필요 |
| GPIO 매크로 느림 | 🟡 중간 | ISR 성능 저하 | 최적화 필요 |
| 서보 각도 계산 오류 | 🟡 중간 | 서보 오동작 | 수정 필요 |
| 나눗셈 ISR 사용 | 🟡 중간 | 타이밍 지터 | 최적화 필요 |
| 채널 상태 관리 | 🟢 낮음 | 에러 처리 부족 | 개선 권장 |

---

## 🎯 최종 권장사항

### 1순위: 하드웨어 LEDC로 전환

```
현재 소프트웨어 PWM → ESP32-C6 내장 LEDC PWM
```

**이유**:
- ✅ CPU 점유율 62.5% → 0%
- ✅ 완벽한 하드웨어 정밀도
- ✅ 전력 효율 최고
- ✅ ESP-IDF 공식 지원
- ✅ 유지보수 용이

**구현 난이도**: 낮음 (ESP-IDF API 사용)  
**예상 작업 시간**: 1-2시간  
**성능 개선**: 극적

### 2순위: 소프트웨어 PWM 수정 (비권장)

만약 8채널이 필수라면:
1. ISR 등록 추가 (`timer_isr_register()`)
2. 인터럽트 주기 10μs로 증가
3. 나눗셈 제거 (채널별 카운터)
4. GPIO 매크로 인라인화

**구현 난이도**: 높음  
**예상 작업 시간**: 4-6시간  
**CPU 점유율**: 여전히 6% 이상

---

## 📝 결론

현재 PWM 드라이버는 **소프트웨어 PWM 구현 시도**이지만:

1. **ISR이 실제로 등록되지 않아 완전히 동작하지 않음** (치명적)
2. **설계 자체가 CPU 과부하를 유발** (1μs 인터럽트)
3. **ESP32-C6의 하드웨어 LEDC를 사용하지 않음** (비효율)

**강력 권장**: 하드웨어 LEDC PWM으로 전면 전환

이 방식이 성능, 전력 효율, 유지보수 모든 면에서 월등히 우수합니다.

---

**분석 완료일**: 2025-10-04  
**다음 단계**: 하드웨어 LEDC 기반 PWM 드라이버 재작성 권장
