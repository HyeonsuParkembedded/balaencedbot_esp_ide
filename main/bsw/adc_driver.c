/**
 * @file adc_driver.c
 * @brief ESP32-C6 Hardware SAR ADC Direct Register Control Implementation
 * 
 * ESP32-C6 SAR ADC 하드웨어 컨트롤러의 레지스터를 직접 제어하는 드라이버 구현입니다.
 * 아날로그 센서 입력을 디지털 값으로 변환하는 데 최적화되어 있습니다.
 * 
 * 구현 특징:
 * - ESP32-C6 SAR ADC 컨트롤러 레지스터 직접 제어
 * - 하드웨어 기반 변환으로 CPU 부하 최소화
 * - Oneshot 모드 지원 (필요 시 변환)
 * - 12비트 해상도 (0-4095)
 * - 다양한 감쇠 레벨 지원
 * - 하드웨어 클럭 관리
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 1.0 (Hardware SAR ADC Controller)
 */

#include "adc_driver.h"
#include "system_services.h"
#include "gpio_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* ADC_TAG = "HW_ADC";

// ADC initialization state
static bool adc_initialized = false;

// ADC channel to GPIO mapping for ESP32-C6
static const uint8_t adc1_gpio_map[BSW_ADC_CHANNEL_MAX] = {
    0,  // ADC1_CH0 -> GPIO0
    1,  // ADC1_CH1 -> GPIO1
    2,  // ADC1_CH2 -> GPIO2
    3,  // ADC1_CH3 -> GPIO3
    4,  // ADC1_CH4 -> GPIO4
    5,  // ADC1_CH5 -> GPIO5
    6   // ADC1_CH6 -> GPIO6
};

// Voltage ranges for different attenuation levels (in mV)
static const uint32_t atten_voltage_range[BSW_ADC_ATTEN_MAX] = {
    800,    // 0dB
    1100,   // 2.5dB
    1350,   // 6dB
    3300    // 11dB (full range)
};

/**
 * @brief Wait for ADC conversion complete with timeout
 * 
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK or ESP_ERR_TIMEOUT
 */
static esp_err_t adc_wait_conversion_done(uint32_t timeout_ms) {
    uint32_t start_time = esp_timer_get_time() / 1000;  // Convert to ms
    
    while (1) {
        // Check if conversion is done (START bit cleared)
        uint32_t onetime_reg = ADC_READ_REG(APB_SARADC_ONETIME_SAMPLE_REG_OFFSET);
        if (!(onetime_reg & ADC_ONETIME_START_BIT)) {
            return ESP_OK;
        }
        
        // Check timeout
        uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
        if (elapsed >= timeout_ms) {
            BSW_LOGE(ADC_TAG, "ADC conversion timeout after %lu ms", elapsed);
            return ESP_ERR_TIMEOUT;
        }
        
        // Small delay to avoid busy-wait
        esp_rom_delay_us(10);
    }
}

/**
 * @brief Configure ADC power and clock
 */
static void adc_configure_power_clock(void) {
    // Power up SAR ADC
    ADC_SET_BITS(APB_SARADC_CTRL_REG_OFFSET, ADC_CTRL_XPD_SAR_FORCE_BIT);
    
    // Enable SAR clock
    ADC_SET_BITS(APB_SARADC_CTRL_REG_OFFSET, ADC_CTRL_SAR_CLK_GATED_BIT);
    
    // Configure ADC clock divider (APB_CLK / divider)
    // Default: use APB clock with divider = 2 for stable operation
    uint32_t clkm_conf = ADC_READ_REG(APB_SARADC_CLKM_CONF_REG_OFFSET);
    clkm_conf &= ~(0xFF);  // Clear divider bits
    clkm_conf |= 2;        // Set divider to 2
    ADC_WRITE_REG(APB_SARADC_CLKM_CONF_REG_OFFSET, clkm_conf);
}

/**
 * @brief Reset ADC controller
 */
static void adc_hw_reset(void) {
    // Clear all interrupts
    ADC_WRITE_REG(APB_SARADC_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    // Disable all interrupts
    ADC_WRITE_REG(APB_SARADC_INT_ENA_REG_OFFSET, 0);
    
    // Reset control registers
    ADC_WRITE_REG(APB_SARADC_CTRL_REG_OFFSET, 0);
    ADC_WRITE_REG(APB_SARADC_CTRL2_REG_OFFSET, 0);
}

/**
 * @brief ADC Hardware Controller Initialization
 * 
 * @return esp_err_t Initialization result
 */
esp_err_t bsw_adc_init(void) {
    if (adc_initialized) {
        BSW_LOGW(ADC_TAG, "ADC already initialized");
        return ESP_OK;
    }
    
    // Reset ADC hardware
    adc_hw_reset();
    
    // Configure power and clock
    adc_configure_power_clock();
    
    // Wait for ADC to stabilize
    esp_rom_delay_us(100);
    
    adc_initialized = true;
    
    BSW_LOGI(ADC_TAG, "ADC HW controller initialized");
    
    return ESP_OK;
}

/**
 * @brief ADC Channel Configuration
 * 
 * @param config ADC channel configuration
 * @return esp_err_t Configuration result
 */
esp_err_t bsw_adc_config_channel(const adc_channel_config_t* config) {
    if (!adc_initialized) {
        BSW_LOGE(ADC_TAG, "ADC not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!config) {
        BSW_LOGE(ADC_TAG, "Invalid config pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->unit >= BSW_ADC_UNIT_MAX) {
        BSW_LOGE(ADC_TAG, "Invalid ADC unit: %d", config->unit);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->channel >= BSW_ADC_CHANNEL_MAX) {
        BSW_LOGE(ADC_TAG, "Invalid ADC channel: %d", config->channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->attenuation >= BSW_ADC_ATTEN_MAX) {
        BSW_LOGE(ADC_TAG, "Invalid attenuation: %d", config->attenuation);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Configure GPIO as analog input (disable digital functions)
    // For ADC1, use the channel-to-GPIO mapping
    bsw_gpio_num_t gpio_num = config->gpio_pin;
    
    if (config->unit == BSW_ADC_UNIT_1) {
        if (config->channel < BSW_ADC_CHANNEL_MAX) {
            gpio_num = adc1_gpio_map[config->channel];
        }
    }
    
    // Configure GPIO for ADC (analog input)
    esp_err_t ret = bsw_gpio_config_pin(gpio_num, BSW_GPIO_MODE_INPUT, 
                                       BSW_GPIO_PULLUP_DISABLE, 
                                       BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(ADC_TAG, "Failed to configure GPIO %d for ADC", gpio_num);
        return ret;
    }
    
    BSW_LOGI(ADC_TAG, "ADC%d CH%d configured: GPIO=%d, atten=%d", 
             config->unit + 1, config->channel, gpio_num, config->attenuation);
    
    return ESP_OK;
}

/**
 * @brief ADC Raw Value Read (Oneshot Mode)
 * 
 * @param unit ADC unit number
 * @param channel ADC channel number
 * @param raw_value Pointer to store raw value
 * @return esp_err_t Read result
 */
esp_err_t bsw_adc_get_raw(bsw_adc_unit_t unit, bsw_adc_channel_t channel, uint32_t* raw_value) {
    if (!adc_initialized) {
        BSW_LOGE(ADC_TAG, "ADC not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!raw_value) {
        BSW_LOGE(ADC_TAG, "Invalid raw_value pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (unit >= BSW_ADC_UNIT_MAX) {
        BSW_LOGE(ADC_TAG, "Invalid ADC unit: %d", unit);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (channel >= BSW_ADC_CHANNEL_MAX) {
        BSW_LOGE(ADC_TAG, "Invalid ADC channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Configure oneshot sample register
    // Bits [31]: START
    // Bits [28:24]: Channel selection
    // Bits [17:16]: Attenuation (use 11dB for full range)
    uint32_t onetime_reg = ADC_ONETIME_START_BIT | 
                           (channel << ADC_ONETIME_CHANNEL_SHIFT) |
                           (BSW_ADC_ATTEN_DB_11 << ADC_ONETIME_ATTEN_SHIFT);
    
    // Start conversion
    ADC_WRITE_REG(APB_SARADC_ONETIME_SAMPLE_REG_OFFSET, onetime_reg);
    
    // Wait for conversion to complete
    esp_err_t result = adc_wait_conversion_done(100);  // 100ms timeout
    if (result != ESP_OK) {
        BSW_LOGE(ADC_TAG, "ADC conversion timeout");
        return result;
    }
    
    // Read conversion result
    if (unit == BSW_ADC_UNIT_1) {
        uint32_t status_reg = ADC_READ_REG(APB_SARADC_SAR1_DATA_STATUS_REG_OFFSET);
        *raw_value = status_reg & 0xFFF;  // 12-bit result
    } else {
        uint32_t status_reg = ADC_READ_REG(APB_SARADC_SAR2_DATA_STATUS_REG_OFFSET);
        *raw_value = status_reg & 0xFFF;  // 12-bit result
    }
    
    return ESP_OK;
}

/**
 * @brief ADC Raw Value to Voltage Conversion
 * 
 * @param raw_value ADC raw value (0-4095)
 * @param attenuation Attenuation level
 * @param voltage_mv Pointer to store voltage in mV
 * @return esp_err_t Conversion result
 */
esp_err_t bsw_adc_raw_to_voltage(uint32_t raw_value, bsw_adc_atten_t attenuation, uint32_t* voltage_mv) {
    if (!voltage_mv) {
        BSW_LOGE(ADC_TAG, "Invalid voltage_mv pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (attenuation >= BSW_ADC_ATTEN_MAX) {
        BSW_LOGE(ADC_TAG, "Invalid attenuation: %d", attenuation);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (raw_value > ADC_MAX_RAW_VALUE) {
        BSW_LOGE(ADC_TAG, "Invalid raw value: %lu", raw_value);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Linear conversion based on attenuation
    // voltage = (raw_value * voltage_range) / ADC_MAX_RAW_VALUE
    uint32_t voltage_range = atten_voltage_range[attenuation];
    *voltage_mv = (raw_value * voltage_range) / ADC_MAX_RAW_VALUE;
    
    return ESP_OK;
}

/**
 * @brief ADC Driver Deinitialization
 * 
 * @return esp_err_t Deinitialization result
 */
esp_err_t bsw_adc_deinit(void) {
    if (!adc_initialized) {
        BSW_LOGW(ADC_TAG, "ADC not initialized");
        return ESP_OK;
    }
    
    // Reset ADC hardware
    adc_hw_reset();
    
    // Power down SAR ADC
    ADC_CLEAR_BITS(APB_SARADC_CTRL_REG_OFFSET, ADC_CTRL_XPD_SAR_FORCE_BIT);
    
    // Disable SAR clock
    ADC_CLEAR_BITS(APB_SARADC_CTRL_REG_OFFSET, ADC_CTRL_SAR_CLK_GATED_BIT);
    
    adc_initialized = false;
    
    BSW_LOGI(ADC_TAG, "ADC HW controller deinitialized");
    
    return ESP_OK;
}
