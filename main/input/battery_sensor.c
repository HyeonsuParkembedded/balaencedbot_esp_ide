/**
 * @file battery_sensor.c
 * @brief 배터리 전압 측정 센서 구현 파일
 * 
 * ADC를 사용하여 배터리 전압을 측정하고 모니터링하는 기능을 구현합니다.
 * 전압 분배 저항을 사용하여 실제 배터리 전압을 계산합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 1.0
 */

#include "battery_sensor.h"
#include "../bsw/system_services.h"
#include "../bsw/adc_driver.h"
#include "../config.h"
#include "esp_log.h"

static const char* TAG = "BATTERY_SENSOR";

/**
 * @brief 배터리 센서 초기화 (BSW ADC 드라이버 사용)
 */
esp_err_t battery_sensor_init(battery_sensor_t* battery, 
                             bsw_adc_unit_t adc_unit,
                             bsw_adc_channel_t adc_channel,
                             bsw_gpio_num_t gpio_pin,
                             float r1_kohm, 
                             float r2_kohm) {
    if (!battery) {
        BSW_LOGE(TAG, "Invalid battery sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }

    memset(battery, 0, sizeof(battery_sensor_t));
    
    // 전압 분배비 계산: Vout = Vin * R2/(R1+R2)
    // 따라서 Vin = Vout * (R1+R2)/R2
    battery->voltage_divider_ratio = (r1_kohm + r2_kohm) / r2_kohm;
    battery->adc_unit = adc_unit;
    battery->adc_channel = adc_channel;
    battery->attenuation = BSW_ADC_ATTEN_DB_11;  // 0~3.3V 범위 (배터리 전압 측정에 적합)
    
    // BSW ADC 초기화
    esp_err_t ret = bsw_adc_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // 이미 초기화된 경우는 OK
        BSW_LOGE(TAG, "Failed to initialize BSW ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // ADC 채널 설정
    adc_channel_config_t adc_config = {
        .unit = adc_unit,
        .channel = adc_channel,
        .attenuation = battery->attenuation,
        .gpio_pin = gpio_pin
    };
    
    ret = bsw_adc_config_channel(&adc_config);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    battery->initialized = true;
    battery->last_voltage = 0.0f;
    battery->battery_level = BATTERY_LEVEL_NORMAL;
    
    BSW_LOGI(TAG, "Battery sensor initialized with BSW ADC - Unit: %d, Channel: %d, GPIO: %d, Atten: 11dB", 
             adc_unit, adc_channel, gpio_pin);
    
    return ESP_OK;
}

/**
 * @brief 배터리 전압 측정 (BSW ADC 드라이버 사용)
 */
float battery_sensor_read_voltage(battery_sensor_t* battery) {
    if (!battery || !battery->initialized) {
        BSW_LOGE(TAG, "Battery sensor not initialized");
        return -1.0f;
    }

    // BSW ADC로 raw 값 읽기
    uint32_t adc_raw = 0;
    esp_err_t ret = bsw_adc_get_raw(battery->adc_unit, battery->adc_channel, &adc_raw);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "BSW ADC read failed: %s", esp_err_to_name(ret));
        return -1.0f;
    }

    // BSW ADC raw 값을 전압(mV)으로 변환
    uint32_t voltage_mv = 0;
    ret = bsw_adc_raw_to_voltage(adc_raw, battery->attenuation, &voltage_mv);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "BSW ADC voltage conversion failed: %s", esp_err_to_name(ret));
        return -1.0f;
    }

    // 전압 분배비를 적용하여 실제 배터리 전압 계산
    float actual_voltage = (voltage_mv / 1000.0f) * battery->voltage_divider_ratio;
    battery->last_voltage = actual_voltage;
    
    // 배터리 레벨 업데이트
    if (actual_voltage >= CONFIG_BATTERY_MAX_VOLTAGE * 0.9f) {
        battery->battery_level = BATTERY_LEVEL_FULL;
    } else if (actual_voltage >= CONFIG_BATTERY_MAX_VOLTAGE * 0.7f) {
        battery->battery_level = BATTERY_LEVEL_HIGH;
    } else if (actual_voltage >= CONFIG_BATTERY_LOW_THRESHOLD) {
        battery->battery_level = BATTERY_LEVEL_NORMAL;
    } else if (actual_voltage >= CONFIG_BATTERY_CRITICAL_THRESHOLD) {
        battery->battery_level = BATTERY_LEVEL_LOW;
    } else {
        battery->battery_level = BATTERY_LEVEL_CRITICAL;
    }

    return actual_voltage;
}

/**
 * @brief 배터리 레벨 확인
 */
battery_level_t battery_sensor_get_level(battery_sensor_t* battery) {
    if (!battery || !battery->initialized) {
        return BATTERY_LEVEL_CRITICAL;
    }
    return battery->battery_level;
}

/**
 * @brief 배터리 퍼센트 계산
 */
int battery_sensor_get_percentage(battery_sensor_t* battery) {
    if (!battery || !battery->initialized) {
        return -1;
    }

    float voltage = battery->last_voltage;
    if (voltage <= 0) {
        voltage = battery_sensor_read_voltage(battery);
        if (voltage <= 0) {
            return -1;
        }
    }

    // 선형 보간을 사용한 배터리 퍼센트 계산
    if (voltage >= CONFIG_BATTERY_MAX_VOLTAGE) {
        return 100;
    } else if (voltage <= CONFIG_BATTERY_MIN_VOLTAGE) {
        return 0;
    } else {
        float range = CONFIG_BATTERY_MAX_VOLTAGE - CONFIG_BATTERY_MIN_VOLTAGE;
        float normalized = (voltage - CONFIG_BATTERY_MIN_VOLTAGE) / range;
        return (int)(normalized * 100.0f);
    }
}

/**
 * @brief 저전압 알림 확인
 */
bool battery_sensor_is_low(battery_sensor_t* battery) {
    if (!battery || !battery->initialized) {
        return true; // 안전을 위해 true 반환
    }
    return battery->last_voltage < CONFIG_BATTERY_LOW_THRESHOLD;
}

/**
 * @brief 위험 전압 알림 확인
 */
bool battery_sensor_is_critical(battery_sensor_t* battery) {
    if (!battery || !battery->initialized) {
        return true; // 안전을 위해 true 반환
    }
    return battery->last_voltage < CONFIG_BATTERY_CRITICAL_THRESHOLD;
}

/**
 * @brief 배터리 센서 해제 (BSW ADC는 전역 리소스이므로 개별 해제 불필요)
 */
esp_err_t battery_sensor_deinit(battery_sensor_t* battery) {
    if (!battery) {
        return ESP_ERR_INVALID_ARG;
    }

    battery->initialized = false;
    battery->last_voltage = 0.0f;
    battery->battery_level = BATTERY_LEVEL_CRITICAL;
    
    BSW_LOGI(TAG, "Battery sensor deinitialized (BSW ADC remains active)");
    
    return ESP_OK;
}