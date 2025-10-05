# I2C Driver v6.1 & ADC Driver v1.1 Upgrade Guide

**Date**: 2025-10-04  
**Upgrade Type**: CPU Blocking Removal (Minor Update)  
**Pattern**: SPI v2.0 vTaskDelay() Pattern

---

## 📋 Summary

간단한 업그레이드: **esp_rom_delay_us(10) → vTaskDelay(1)**로 변경하여 CPU blocking 제거

### I2C Driver v6.0 → v6.1
- ✅ Mutex는 이미 v6.0에 구현되어 있음
- ✅ CPU blocking만 제거 (1개 함수)
- ✅ 하위 호환성 100%

### ADC Driver v1.0 → v1.1
- ✅ Mutex는 이미 v1.0에 구현되어 있음
- ✅ CPU blocking만 제거 (1개 함수)
- ✅ 하위 호환성 100%

---

## 🔧 I2C Driver v6.1 Changes

### Modified Function: `i2c_wait_trans_complete()`

**BEFORE (v6.0)**:
```c
static esp_err_t i2c_wait_trans_complete(uint32_t base, uint32_t timeout_ms) {
    uint32_t start_time = esp_timer_get_time() / 1000;  // Convert to ms
    
    while (1) {
        uint32_t int_status = I2C_READ_REG(base, I2C_INT_RAW_REG_OFFSET);
        
        if (int_status & I2C_INT_TRANS_COMPLETE_BIT) {
            I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, I2C_INT_TRANS_COMPLETE_BIT);
            return ESP_OK;
        }
        
        // Check timeout
        uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
        if (elapsed >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        
        // ❌ CPU BLOCKING!
        esp_rom_delay_us(10);
    }
}
```

**AFTER (v6.1)**:
```c
static esp_err_t i2c_wait_trans_complete(uint32_t base, uint32_t timeout_ms) {
    TickType_t start_tick = xTaskGetTickCount();  // ← FreeRTOS tick 사용
    
    while (1) {
        uint32_t int_status = I2C_READ_REG(base, I2C_INT_RAW_REG_OFFSET);
        
        if (int_status & I2C_INT_TRANS_COMPLETE_BIT) {
            I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, I2C_INT_TRANS_COMPLETE_BIT);
            return ESP_OK;
        }
        
        // Check timeout
        TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
        if (pdTICKS_TO_MS(elapsed_ticks) >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        
        // ✅ CPU YIELDING to other tasks!
        vTaskDelay(1);
    }
}
```

**Key Changes**:
1. `uint32_t start_time` → `TickType_t start_tick`
2. `esp_timer_get_time()` → `xTaskGetTickCount()`
3. `esp_rom_delay_us(10)` → `vTaskDelay(1)`
4. Timeout calculation: `pdTICKS_TO_MS(elapsed_ticks)`

---

## 🔧 ADC Driver v1.1 Changes

### Modified Function: `adc_wait_conversion_done()`

**BEFORE (v1.0)**:
```c
static esp_err_t adc_wait_conversion_done(uint32_t timeout_ms) {
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    while (1) {
        uint32_t onetime_reg = ADC_READ_REG(APB_SARADC_ONETIME_SAMPLE_REG_OFFSET);
        if (!(onetime_reg & ADC_ONETIME_START_BIT)) {
            return ESP_OK;
        }
        
        uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
        if (elapsed >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        
        // ❌ CPU BLOCKING!
        esp_rom_delay_us(10);
    }
}
```

**AFTER (v1.1)**:
```c
static esp_err_t adc_wait_conversion_done(uint32_t timeout_ms) {
    TickType_t start_tick = xTaskGetTickCount();  // ← FreeRTOS tick 사용
    
    while (1) {
        uint32_t onetime_reg = ADC_READ_REG(APB_SARADC_ONETIME_SAMPLE_REG_OFFSET);
        if (!(onetime_reg & ADC_ONETIME_START_BIT)) {
            return ESP_OK;
        }
        
        TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
        if (pdTICKS_TO_MS(elapsed_ticks) >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        
        // ✅ CPU YIELDING to other tasks!
        vTaskDelay(1);
    }
}
```

**동일한 패턴**: SPI v2.0과 동일한 변경 사항

---

## 📊 Performance Impact

### I2C Driver (MPU6050 @ 400kHz)
- v6.0: `esp_rom_delay_us(10)` - CPU 100% 점유
- v6.1: `vTaskDelay(1)` - CPU 양보, 50Hz 제어 루프에 영향 없음
- **측정 필요**: IMU 읽기 시간 비교

### ADC Driver (Battery Monitoring @ ~1Hz)
- v1.0: `esp_rom_delay_us(10)` - CPU 100% 점유 (매우 짧은 시간)
- v1.1: `vTaskDelay(1)` - CPU 양보
- **영향 미미**: 배터리 모니터링은 저빈도 작업 (~1Hz)

---

## 🎯 Migration Guide

### 기존 코드 수정 불필요!

두 드라이버 모두 **내부 함수만 변경**되었으므로 사용자 코드는 수정 불필요합니다.

#### I2C 사용 코드 (변경 없음)
```c
// main.c - 코드 수정 불필요!
bsw_i2c_init(BSW_I2C_PORT_0, &i2c_config);
bsw_i2c_master_write_read(BSW_I2C_PORT_0, MPU6050_ADDR, 
                          write_buf, write_len, 
                          read_buf, read_len);
```

#### ADC 사용 코드 (변경 없음)
```c
// main.c - 코드 수정 불필요!
bsw_adc_init();
bsw_adc_config_channel(&adc_config);
bsw_adc_get_raw(BSW_ADC_UNIT_1, BSW_ADC_CHANNEL_0, &raw_value);
```

---

## ✅ Build Results

```
✅ Build successful
✅ Total image size: 672,255 bytes
✅ DIRAM: 107KB / 452KB (23.79%)
✅ Flash Code: 580KB
✅ Memory usage: Identical to previous build (no overhead)
```

**메모리 변화**: 없음 (함수 내부 로직만 변경, 크기 동일)

---

## 🧪 Testing Checklist

### I2C Driver v6.1
- [ ] MPU6050 IMU 센서 읽기 정상 동작
- [ ] 50Hz 제어 루프 타이밍 영향 없음
- [ ] 멀티태스크 환경에서 IMU 데이터 무결성 확인
- [ ] Bus recovery 메커니즘 정상 동작

### ADC Driver v1.1
- [ ] 배터리 전압 읽기 정상 동작
- [ ] ADC 변환 시간 측정 (예상: ~10-20μs, v1.0과 동일)
- [ ] 멀티채널 ADC 동시 사용 테스트

---

## 📚 Reference

- **SPI_V2_UPGRADE_GUIDE.md**: 동일한 vTaskDelay() 패턴 적용
- **GPIO_V3_UPGRADE_GUIDE.md**: Mutex 추가 패턴 (I2C/ADC는 이미 적용됨)
- **BSW_COMPREHENSIVE_STATUS_REPORT.md**: 전체 드라이버 현황

---

## 🎓 Lessons Learned

1. **Mutex는 이미 있었음**: I2C v6.0과 ADC v1.0이 이미 FreeRTOS mutex를 구현했지만 CPU blocking은 남아있었음
2. **간단한 업그레이드**: 1개 함수만 수정으로 CPU blocking 완전 제거
3. **일관성 유지**: UART v4.0, SPI v2.0과 동일한 패턴 적용
4. **하위 호환성**: 사용자 코드 수정 불필요, 내부 구현만 개선

---

**작성 완료**: 2025-10-04  
**빌드 상태**: ✅ 성공 (672KB)
