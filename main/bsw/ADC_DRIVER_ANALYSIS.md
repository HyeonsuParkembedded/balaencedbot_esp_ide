# ADC 드라이버 분석 보고서

**분석 날짜**: 2025년 10월 4일  
**대상 파일**: `main/bsw/adc_driver.c`, `main/bsw/adc_driver.h`  
**ESP32-C6 버전**: v5.5  
**분석자**: GitHub Copilot

---

## 📋 개요

ESP32-C6 하드웨어 SAR ADC 컨트롤러의 레지스터를 직접 제어하는 드라이버입니다. 배터리 전압 측정 등 아날로그 센서 입력을 디지털 값으로 변환합니다.

---

## 🔍 주요 발견 사항

### ✅ 잘 구현된 부분

#### 1. **하드웨어 레지스터 직접 제어**
```c
// ADC 레지스터 매크로 정의 (명확하고 일관성 있음)
#define ADC_BASE_ADDR           0x60040000UL
#define ADC_READ_REG(offset)    (*(volatile uint32_t*)ADC_REG_ADDR(offset))
#define ADC_WRITE_REG(offset, val) (*(volatile uint32_t*)ADC_REG_ADDR(offset) = (val))
```
- ✅ ESP32-C6 TRM Chapter 30 기반의 정확한 레지스터 주소
- ✅ 일관된 매크로 네이밍 규칙
- ✅ 직접 레지스터 접근으로 오버헤드 최소화

#### 2. **ADC 채널-GPIO 매핑**
```c
static const uint8_t adc1_gpio_map[BSW_ADC_CHANNEL_MAX] = {
    0,  // ADC1_CH0 -> GPIO0
    1,  // ADC1_CH1 -> GPIO1
    2,  // ADC1_CH2 -> GPIO2
    3,  // ADC1_CH3 -> GPIO3
    4,  // ADC1_CH4 -> GPIO4
    5,  // ADC1_CH5 -> GPIO5
    6   // ADC1_CH6 -> GPIO6
};
```
- ✅ 명확한 채널-핀 매핑 테이블
- ✅ ESP32-C6 하드웨어 사양 반영

#### 3. **Oneshot 모드 구현**
```c
uint32_t onetime_reg = ADC_ONETIME_START_BIT | 
                       (channel << ADC_ONETIME_CHANNEL_SHIFT) |
                       (BSW_ADC_ATTEN_DB_11 << ADC_ONETIME_ATTEN_SHIFT);
ADC_WRITE_REG(APB_SARADC_ONETIME_SAMPLE_REG_OFFSET, onetime_reg);
```
- ✅ 필요 시에만 변환 수행 (CPU 부하 최소화)
- ✅ 타임아웃 메커니즘 구현 (100ms)

---

## ⚠️ 발견된 문제점

### 🔴 심각도 높음

#### 1. **Flash 핀 보호 누락**
**문제**: ADC1 채널 6은 GPIO 6을 사용하는데, **GPIO 6은 I2C SDA로 이미 사용 중**입니다!

```c
// config.h
#define CONFIG_MPU6050_SDA_PIN  GPIO_NUM_6   // I2C SDA

// adc_driver.c (문제 발생!)
static const uint8_t adc1_gpio_map[BSW_ADC_CHANNEL_MAX] = {
    6   // ADC1_CH6 -> GPIO6 (I2C와 충돌!)
};
```

**현재 사용 핀 현황**:
```
GPIO 0  : ADC1_CH0 (사용 가능)
GPIO 1  : Encoder A (Left) - 이미 사용 중
GPIO 2  : Encoder B (Left) - 이미 사용 중
GPIO 3  : Battery ADC (CONFIG_BATTERY_ADC_PIN) - 사용 중
GPIO 4  : ADC1_CH4 (사용 가능)
GPIO 5  : ADC1_CH5 (사용 가능)
GPIO 6  : I2C SDA (MPU6050) - 이미 사용 중! ⚠️
```

**영향**:
- ADC1_CH6 사용 시 I2C 통신 방해 가능
- MPU6050 IMU 센서 오작동 유발
- 배터리 센서는 GPIO 3 (ADC1_CH3) 사용 중

**해결 방법**:
```c
// ADC 채널 검증 추가
if (config->unit == BSW_ADC_UNIT_1 && config->channel == BSW_ADC_CHANNEL_6) {
    BSW_LOGE(ADC_TAG, "ADC1_CH6 (GPIO6) is reserved for I2C");
    return ESP_ERR_NOT_SUPPORTED;
}
```

#### 2. **GPIO 검증 누락**
**문제**: GPIO 26-30 (Flash 핀) 접근 차단이 없습니다.

```c
esp_err_t bsw_adc_config_channel(const adc_channel_config_t* config) {
    // ...
    bsw_gpio_num_t gpio_num = config->gpio_pin;
    
    // Flash 핀 검증 없음! ⚠️
    esp_err_t ret = bsw_gpio_config_pin(gpio_num, ...);
}
```

**해결 방법**:
```c
// GPIO 검증 (gpio_driver에서 이미 validate_gpio_num() 구현됨)
if (gpio_num >= GPIO_FLASH_PIN_START) {  // 26
    BSW_LOGE(ADC_TAG, "GPIO %d reserved for Flash", gpio_num);
    return ESP_ERR_NOT_SUPPORTED;
}
```

#### 3. **감쇠 레벨 하드코딩**
**문제**: `bsw_adc_get_raw()`에서 감쇠 레벨이 11dB로 고정되어 있습니다.

```c
esp_err_t bsw_adc_get_raw(bsw_adc_unit_t unit, bsw_adc_channel_t channel, uint32_t* raw_value) {
    // 감쇠 레벨이 11dB로 하드코딩됨 ⚠️
    uint32_t onetime_reg = ADC_ONETIME_START_BIT | 
                           (channel << ADC_ONETIME_CHANNEL_SHIFT) |
                           (BSW_ADC_ATTEN_DB_11 << ADC_ONETIME_ATTEN_SHIFT);
}
```

**문제점**:
- `bsw_adc_config_channel()`에서 설정한 `config->attenuation` 무시
- 사용자가 0dB, 2.5dB, 6dB 설정해도 항상 11dB로 동작
- 저전압 측정 시 해상도 낭비

**해결 방법**:
```c
// 채널별 감쇠 레벨 저장
static bsw_adc_atten_t channel_atten[BSW_ADC_UNIT_MAX][BSW_ADC_CHANNEL_MAX];

// config 시 저장
esp_err_t bsw_adc_config_channel(const adc_channel_config_t* config) {
    // ...
    channel_atten[config->unit][config->channel] = config->attenuation;
}

// get_raw에서 사용
esp_err_t bsw_adc_get_raw(...) {
    bsw_adc_atten_t atten = channel_atten[unit][channel];
    uint32_t onetime_reg = ADC_ONETIME_START_BIT | 
                           (channel << ADC_ONETIME_CHANNEL_SHIFT) |
                           (atten << ADC_ONETIME_ATTEN_SHIFT);
}
```

### 🟡 심각도 중간

#### 4. **ADC2 미구현**
**문제**: ADC2는 선언만 되어 있고 실제 구현이 없습니다.

```c
typedef enum {
    BSW_ADC_UNIT_1 = 0,
    BSW_ADC_UNIT_2,  // 선언만 있음
    BSW_ADC_UNIT_MAX
} bsw_adc_unit_t;

// bsw_adc_get_raw()에서 ADC2 읽기 있지만 레지스터 오프셋이 맞는지 미검증
```

**ESP32-C6 제약사항**: 
- ESP32-C6는 ADC1만 존재 (ADC2 없음)
- ESP32/S3와 달리 단일 ADC 유닛

**해결 방법**:
```c
// ESP32-C6 ADC 사양 명시
#define ESP32C6_HAS_ADC2 0  // ESP32-C6에는 ADC2 없음

#if !ESP32C6_HAS_ADC2
typedef enum {
    BSW_ADC_UNIT_1 = 0,
    // ADC_UNIT_2는 ESP32-C6에서 사용 불가
    BSW_ADC_UNIT_MAX
} bsw_adc_unit_t;
#endif
```

#### 5. **초기화 상태 경합 조건**
**문제**: `adc_initialized` 플래그가 멀티스레드 환경에서 보호되지 않습니다.

```c
static bool adc_initialized = false;

esp_err_t bsw_adc_init(void) {
    if (adc_initialized) {  // 경합 조건 발생 가능 ⚠️
        return ESP_OK;
    }
    // ...
    adc_initialized = true;
}
```

**해결 방법**:
```c
#include "freertos/semphr.h"

static SemaphoreHandle_t adc_mutex = NULL;

esp_err_t bsw_adc_init(void) {
    if (adc_mutex == NULL) {
        adc_mutex = xSemaphoreCreateMutex();
    }
    
    xSemaphoreTake(adc_mutex, portMAX_DELAY);
    if (adc_initialized) {
        xSemaphoreGive(adc_mutex);
        return ESP_OK;
    }
    // ... 초기화
    adc_initialized = true;
    xSemaphoreGive(adc_mutex);
}
```

#### 6. **전압 변환 선형화 문제**
**문제**: ADC 비선형성 보정이 없습니다.

```c
esp_err_t bsw_adc_raw_to_voltage(uint32_t raw_value, bsw_adc_atten_t attenuation, uint32_t* voltage_mv) {
    uint32_t voltage_range = atten_voltage_range[attenuation];
    *voltage_mv = (raw_value * voltage_range) / ADC_MAX_RAW_VALUE;  // 선형 변환만
}
```

**ESP32-C6 ADC 특성**:
- ADC는 비선형성이 있음 (특히 양 끝단)
- ESP-IDF는 eFuse에 교정 데이터 저장
- `adc_cali_scheme_curve_fitting` 필요

**현재 battery_sensor.c는 교정 사용 중**:
```c
// battery_sensor.c (올바른 구현)
adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .chan = channel,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
};
adc_cali_create_scheme_curve_fitting(&cali_config, &battery->adc_cali_handle);
```

**문제점**:
- BSW ADC 드라이버는 교정 없이 raw 변환만 제공
- 사용자가 직접 교정 구현해야 함 (중복 코드)

### 🟢 심각도 낮음

#### 7. **타임아웃 시간 고정**
```c
esp_err_t result = adc_wait_conversion_done(100);  // 100ms 고정
```

**개선 방법**:
```c
#define ADC_CONVERSION_TIMEOUT_MS 100
result = adc_wait_conversion_done(ADC_CONVERSION_TIMEOUT_MS);
```

#### 8. **에러 메시지 불충분**
```c
if (ret != ESP_OK) {
    BSW_LOGE(ADC_TAG, "Failed to configure GPIO %d for ADC", gpio_num);
    return ret;  // 왜 실패했는지 구체적 정보 없음
}
```

---

## 📊 현재 사용 현황

### Battery Sensor (정상 작동 중)
```c
// config.h
#define CONFIG_BATTERY_ADC_PIN      GPIO_NUM_3   // ADC1_CH3
#define CONFIG_BATTERY_ADC_CHANNEL  ADC_CHANNEL_3

// battery_sensor.c - ESP-IDF ADC API 사용 (BSW ADC 아님!)
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;
adc_oneshot_read(battery->adc_handle, battery->channel, &adc_raw);
```

**중요**: 배터리 센서는 **ESP-IDF의 ADC Oneshot API**를 사용하며, **BSW ADC 드라이버를 사용하지 않습니다**!

---

## 💡 개선 권장사항

### 우선순위 1 (즉시 수정 필요)

1. **ADC 채널 6 사용 금지** (I2C 충돌 방지)
```c
if (config->channel == BSW_ADC_CHANNEL_6) {
    BSW_LOGE(ADC_TAG, "ADC1_CH6 (GPIO6) reserved for I2C");
    return ESP_ERR_NOT_SUPPORTED;
}
```

2. **감쇠 레벨 동적 설정**
```c
static bsw_adc_atten_t channel_atten[BSW_ADC_UNIT_MAX][BSW_ADC_CHANNEL_MAX];
// config에서 저장, get_raw에서 사용
```

3. **Flash 핀 보호**
```c
if (gpio_num >= GPIO_FLASH_PIN_START) {
    return ESP_ERR_NOT_SUPPORTED;
}
```

### 우선순위 2 (성능 개선)

4. **ADC2 제거 (ESP32-C6는 ADC1만 존재)**
```c
typedef enum {
    BSW_ADC_UNIT_1 = 0,
    BSW_ADC_UNIT_MAX  // ADC2 제거
} bsw_adc_unit_t;
```

5. **멀티스레드 보호**
```c
static SemaphoreHandle_t adc_mutex = NULL;
// 모든 공개 API에서 Mutex 사용
```

6. **eFuse 기반 교정 추가**
```c
typedef struct {
    adc_cali_handle_t cali_handle;
    bsw_adc_atten_t attenuation;
    bool calibrated;
} adc_channel_state_t;

static adc_channel_state_t channel_state[BSW_ADC_CHANNEL_MAX];
```

### 우선순위 3 (코드 품질)

7. **타임아웃 상수화**
```c
#define ADC_CONVERSION_TIMEOUT_MS 100
#define ADC_STABILIZATION_DELAY_US 100
```

8. **에러 메시지 개선**
```c
BSW_LOGE(ADC_TAG, "GPIO config failed: %s (GPIO%d)", 
         esp_err_to_name(ret), gpio_num);
```

---

## 🎯 요약

| 항목 | 현재 상태 | 권장 사항 |
|------|----------|----------|
| **레지스터 제어** | ✅ 우수 | 유지 |
| **채널 매핑** | ✅ 명확 | GPIO 6 (I2C) 충돌 수정 |
| **Oneshot 모드** | ✅ 구현됨 | 감쇠 레벨 하드코딩 해제 |
| **ADC2 지원** | ⚠️ 미구현 | ESP32-C6는 ADC1만 지원 |
| **Flash 핀 보호** | ❌ 없음 | GPIO 26-30 접근 차단 |
| **멀티스레드** | ❌ 미보호 | Mutex 추가 |
| **교정** | ❌ 없음 | eFuse 교정 추가 |
| **실제 사용** | ⚠️ 미사용 | battery_sensor는 ESP-IDF API 사용 |

**결론**: 
- BSW ADC 드라이버는 구현되어 있지만 **실제로는 사용되지 않습니다**.
- 배터리 센서는 ESP-IDF의 `adc_oneshot` API를 직접 사용합니다.
- I2C 충돌, 감쇠 레벨 하드코딩, Flash 핀 보호 등 개선 필요.
- ESP32-C6 하드웨어 제약(ADC1만 존재)을 반영해야 합니다.

---

## 📝 구현 예시

### ADC 채널-감쇠 관리
```c
// 채널별 감쇠 레벨 저장
static bsw_adc_atten_t channel_atten[BSW_ADC_UNIT_MAX][BSW_ADC_CHANNEL_MAX] = {
    {BSW_ADC_ATTEN_DB_11, BSW_ADC_ATTEN_DB_11, ...}  // 기본값
};

esp_err_t bsw_adc_config_channel(const adc_channel_config_t* config) {
    // I2C 충돌 검사
    if (config->unit == BSW_ADC_UNIT_1 && config->channel == BSW_ADC_CHANNEL_6) {
        BSW_LOGE(ADC_TAG, "ADC1_CH6 (GPIO6) is reserved for I2C SDA");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Flash 핀 검사
    bsw_gpio_num_t gpio_num = adc1_gpio_map[config->channel];
    if (gpio_num >= GPIO_FLASH_PIN_START) {
        BSW_LOGE(ADC_TAG, "GPIO%d is reserved for Flash", gpio_num);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // 감쇠 레벨 저장
    channel_atten[config->unit][config->channel] = config->attenuation;
    
    // ... 기존 코드
}

esp_err_t bsw_adc_get_raw(bsw_adc_unit_t unit, bsw_adc_channel_t channel, uint32_t* raw_value) {
    // 저장된 감쇠 레벨 사용
    bsw_adc_atten_t atten = channel_atten[unit][channel];
    
    uint32_t onetime_reg = ADC_ONETIME_START_BIT | 
                           (channel << ADC_ONETIME_CHANNEL_SHIFT) |
                           (atten << ADC_ONETIME_ATTEN_SHIFT);
    // ...
}
```

**메모리 사용량**: 14 bytes (7 channels × 2 units × 1 byte)
