# ESP32-C6 I2C Hardware Constraints

## ⚠️ 중요: ESP32-C6 I2C 하드웨어 제약사항

### 🔴 I2C 컨트롤러 개수

ESP32-C6는 **I2C 컨트롤러가 1개만** 있습니다.

| ESP32 시리즈 | I2C0 | I2C1 | 총 개수 |
|--------------|------|------|---------|
| ESP32 | ✅ | ✅ | 2개 |
| ESP32-S2 | ✅ | ✅ | 2개 |
| ESP32-S3 | ✅ | ✅ | 2개 |
| ESP32-C3 | ✅ | ❌ | 1개 |
| **ESP32-C6** | **✅** | **❌** | **1개** |
| ESP32-H2 | ✅ | ❌ | 1개 |

### 📍 사용 가능한 I2C 컨트롤러

#### ✅ I2C0 (사용 가능)
```c
#define I2C0_BASE_ADDR    0x60013000UL
```
- **포트 번호**: `BSW_I2C_PORT_0` (I2C_NUM_0)
- **상태**: ✅ 완전 지원
- **GPIO 신호**:
  - `I2CEXT0_SDA_OUT_IDX` = 46
  - `I2CEXT0_SDA_IN_IDX` = 46
  - `I2CEXT0_SCL_OUT_IDX` = 45
  - `I2CEXT0_SCL_IN_IDX` = 45

#### ❌ I2C1 (사용 불가)
```c
// #define I2C1_BASE_ADDR 0x60014000UL  // ❌ NOT AVAILABLE on ESP32-C6
```
- **포트 번호**: ~~`BSW_I2C_PORT_1` (I2C_NUM_1)~~
- **상태**: ❌ **존재하지 않음**
- **시도 시 결과**: `ESP_ERR_NOT_SUPPORTED` 반환

### 🛡️ 코드에서의 보호 메커니즘

#### 1. 베이스 주소 배열
```c
static const uint32_t i2c_base_addrs[BSW_I2C_PORT_MAX] = {
    I2C0_BASE_ADDR,
    0  // I2C1 not available on ESP32-C6
};
```

#### 2. 초기화 시 검증
```c
if (port != BSW_I2C_PORT_0) {
    BSW_LOGE(I2C_TAG, "ESP32-C6 only supports I2C0. I2C%d is not available.", port);
    return ESP_ERR_NOT_SUPPORTED;
}
```

#### 3. GPIO Matrix 검증
```c
static esp_err_t i2c_gpio_matrix_config(bsw_i2c_port_t port, ...) {
    if (port != BSW_I2C_PORT_0) {
        BSW_LOGE(I2C_TAG, "ESP32-C6 only supports I2C port 0");
        return ESP_ERR_INVALID_ARG;
    }
    // ...
}
```

### 📝 올바른 사용 예제

#### ✅ 올바른 코드
```c
// I2C0 사용 (ESP32-C6에서 유일한 옵션)
i2c_driver_init(BSW_I2C_PORT_0, GPIO_NUM_6, GPIO_NUM_7);

// MPU6050 센서 통신
uint8_t data;
i2c_read_register(BSW_I2C_PORT_0, 0x68, 0x75, &data, 1);
```

#### ❌ 잘못된 코드
```c
// I2C1 사용 시도 - ESP32-C6에서 에러 발생!
i2c_driver_init(BSW_I2C_PORT_1, GPIO_NUM_8, GPIO_NUM_9);
// 결과: ESP_ERR_NOT_SUPPORTED 반환

// 에러 로그 출력:
// E (xxx) HW_I2C: ESP32-C6 only supports I2C0. I2C1 is not available.
```

### 🔧 다중 I2C 디바이스 연결 방법

ESP32-C6에서 여러 I2C 디바이스를 사용하려면:

#### 방법 1: I2C 버스 공유 (권장)
```c
// 동일한 I2C0 버스에 여러 디바이스 연결
// SDA: GPIO_6, SCL: GPIO_7 (공유)

// MPU6050 (주소: 0x68)
i2c_write_register(BSW_I2C_PORT_0, 0x68, reg, value);

// 다른 센서 (주소: 0x76)
i2c_write_register(BSW_I2C_PORT_0, 0x76, reg, value);
```

**장점**:
- ✅ 표준 I2C 방식
- ✅ 최대 127개 디바이스 연결 가능 (주소가 다르면)
- ✅ 하드웨어 I2C 컨트롤러 사용

**제약**:
- ⚠️ 디바이스별로 고유한 I2C 주소 필요
- ⚠️ 주소 충돌 시 I2C 멀티플렉서(TCA9548A 등) 필요

#### 방법 2: 소프트웨어 I2C (비권장)
```c
// 다른 GPIO 핀에 소프트웨어 I2C 구현
// 예: GPIO_10(SDA), GPIO_11(SCL)
```

**단점**:
- ❌ CPU 부하 높음
- ❌ 타이밍 정확도 낮음
- ❌ 별도 구현 필요

### 📚 참고 문서

1. **ESP32-C6 Technical Reference Manual**
   - Chapter 24: I2C Controller
   - Section 24.1: I2C Overview
   - ⚠️ 명시: "ESP32-C6 has one I2C controller"

2. **GPIO Signal Mapping**
   - 파일: `components/soc/esp32c6/include/soc/gpio_sig_map.h`
   - I2C0 신호만 정의됨 (I2C1 신호 없음)

3. **Clock Tree**
   - 파일: `components/soc/esp32c6/include/soc/clk_tree_defs.h`
   - I2C0 클럭만 관리됨

### 🔍 다른 ESP32 시리즈로 마이그레이션 시

만약 I2C 2개가 필요하다면:

```c
// ESP32, ESP32-S2, ESP32-S3에서는 가능
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    i2c_driver_init(BSW_I2C_PORT_0, GPIO_NUM_21, GPIO_NUM_22);  // I2C0
    i2c_driver_init(BSW_I2C_PORT_1, GPIO_NUM_25, GPIO_NUM_26);  // I2C1
#elif CONFIG_IDF_TARGET_ESP32C6
    i2c_driver_init(BSW_I2C_PORT_0, GPIO_NUM_6, GPIO_NUM_7);    // I2C0만
    // I2C1 사용 불가
#endif
```

### ✅ 현재 프로젝트 상태

**BalanceBot 프로젝트**:
- ✅ I2C0만 사용 (MPU6050 IMU 센서)
- ✅ GPIO_6 (SDA), GPIO_7 (SCL)
- ✅ ESP32-C6 하드웨어 제약 준수
- ✅ 추가 I2C 디바이스 필요 시 동일 버스 공유 가능

### 💡 요약

| 항목 | 내용 |
|------|------|
| **사용 가능 I2C** | I2C0만 (1개) |
| **베이스 주소** | 0x60013000 |
| **권장 사용법** | 버스 공유 (여러 디바이스, 다른 주소) |
| **I2C1 사용 시** | `ESP_ERR_NOT_SUPPORTED` 에러 |
| **다중 디바이스** | I2C 주소 구분 또는 멀티플렉서 사용 |

---

*ESP32-C6는 저전력에 최적화된 칩으로, I2C 컨트롤러 1개로도 충분한 I2C 통신을 지원합니다.* 🎯
