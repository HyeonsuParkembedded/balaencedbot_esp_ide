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
#include "../config.h"
#include "esp_log.h"

static const char* TAG = "BATTERY_SENSOR";

/**
 * @brief 배터리 센서 초기화
 */
esp_err_t battery_sensor_init(battery_sensor_t* battery, adc_channel_t channel, 
                             float r1_kohm, float r2_kohm) {
    if (!battery) {
        BSW_LOGE(TAG, "Invalid battery sensor pointer");
        return ESP_ERR_INVALID_ARG;
    }

    memset(battery, 0, sizeof(battery_sensor_t));
    battery->channel = channel;
    
    // 전압 분배비 계산: Vout = Vin * R2/(R1+R2)
    // 따라서 Vin = Vout * (R1+R2)/R2
    battery->voltage_divider_ratio = (r1_kohm + r2_kohm) / r2_kohm;
    
    // ADC 설정
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &battery->adc_handle);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // ADC 채널 설정
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,  // 0~3.1V 범위
    };
    
    ret = adc_oneshot_config_channel(battery->adc_handle, channel, &config);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(battery->adc_handle);
        return ret;
    }

    // ADC 교정 핸들 생성
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &battery->adc_cali_handle);
    if (ret != ESP_OK) {
        BSW_LOGW(TAG, "ADC calibration failed, using raw values");
        battery->adc_cali_handle = NULL;
    }

    battery->initialized = true;
    battery->last_voltage = 0.0f;
    battery->battery_level = BATTERY_LEVEL_NORMAL;
    
    BSW_LOGI(TAG, "Battery sensor initialized - Channel: %d, Voltage divider ratio: %.2f", 
             channel, battery->voltage_divider_ratio);
    
    return ESP_OK;
}

/**
 * @brief 배터리 전압 측정
 */
float battery_sensor_read_voltage(battery_sensor_t* battery) {
    if (!battery || !battery->initialized) {
        BSW_LOGE(TAG, "Battery sensor not initialized");
        return -1.0f;
    }

    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(battery->adc_handle, battery->channel, &adc_raw);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return -1.0f;
    }

    int voltage_mv = 0;
    if (battery->adc_cali_handle != NULL) {
        // 교정된 값 사용
        ret = adc_cali_raw_to_voltage(battery->adc_cali_handle, adc_raw, &voltage_mv);
        if (ret != ESP_OK) {
            BSW_LOGE(TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
            return -1.0f;
        }
    } else {
        // 교정되지 않은 raw 값 사용 (대략적인 변환)
        // ADC_ATTEN_DB_11에서 대략 3.1V/4095 = 0.757mV per LSB
        voltage_mv = (adc_raw * 3100) / 4095;
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
 * @brief 배터리 센서 해제
 */
esp_err_t battery_sensor_deinit(battery_sensor_t* battery) {
    if (!battery) {
        return ESP_ERR_INVALID_ARG;
    }

    if (battery->adc_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(battery->adc_cali_handle);
        battery->adc_cali_handle = NULL;
    }

    if (battery->adc_handle) {
        adc_oneshot_del_unit(battery->adc_handle);
        battery->adc_handle = NULL;
    }

    battery->initialized = false;
    BSW_LOGI(TAG, "Battery sensor deinitialized");
    
    return ESP_OK;
}