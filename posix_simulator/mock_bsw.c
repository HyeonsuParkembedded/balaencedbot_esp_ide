/**
 * @file mock_bsw.c
 * @brief BSW Mock Implementation for POSIX Simulator
 * 
 * BSW 레이어의 Mock 구현체입니다. 실제 하드웨어 동작을 시뮬레이션합니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#include "mock_bsw.h"
#include <stdarg.h>
#include <pthread.h>

// ============================================================================
// Mock State
// ============================================================================
static pthread_mutex_t bsw_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool gpio_initialized = false;
static bool i2c_initialized[BSW_I2C_NUM_MAX] = {false};
static bool pwm_initialized = false;
static bool adc_initialized = false;

// PWM state for motor simulation
static struct {
    bool active;
    uint32_t duty;
    uint32_t frequency;
    bsw_gpio_num_t pin;
} pwm_channels[BSW_PWM_CHANNEL_MAX] = {0};

// ADC simulation (battery voltage)
static uint32_t adc_values[2][BSW_ADC_CHANNEL_MAX] = {
    {2048, 2048, 2048, 2048},  // Unit 1 - simulate 3.3V/2 = ~2048 for 12-bit ADC
    {2048, 2048, 2048, 2048}   // Unit 2
};

// ============================================================================
// BSW GPIO Mock Implementation
// ============================================================================
bsw_err_t bsw_gpio_init(void) {
    pthread_mutex_lock(&bsw_mutex);
    gpio_initialized = true;
    pthread_mutex_unlock(&bsw_mutex);
    BSW_LOGI("MOCK_GPIO", "GPIO subsystem initialized");
    return BSW_OK;
}

bsw_err_t bsw_gpio_set_mode(bsw_gpio_num_t pin, bsw_gpio_mode_t mode) {
    if (!gpio_initialized) return BSW_ERR_INVALID_STATE;
    
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = (gpio_mode_t)mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    
    esp_err_t ret = gpio_config(&config);
    return (ret == ESP_OK) ? BSW_OK : BSW_FAIL;
}

bsw_err_t bsw_gpio_set_level(bsw_gpio_num_t pin, uint32_t level) {
    if (!gpio_initialized) return BSW_ERR_INVALID_STATE;
    
    esp_err_t ret = gpio_set_level((gpio_num_t)pin, level);
    return (ret == ESP_OK) ? BSW_OK : BSW_FAIL;
}

uint32_t bsw_gpio_get_level(bsw_gpio_num_t pin) {
    if (!gpio_initialized) return 0;
    return gpio_get_level((gpio_num_t)pin);
}

// ============================================================================
// BSW I2C Mock Implementation
// ============================================================================
bsw_err_t bsw_i2c_init(bsw_i2c_num_t i2c_num, const bsw_i2c_config_t* config) {
    if (i2c_num >= BSW_I2C_NUM_MAX) return BSW_ERR_INVALID_ARG;
    
    // Configure I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master_clk_speed = config->clk_speed
    };
    
    esp_err_t ret = i2c_param_config((i2c_port_t)i2c_num, &i2c_conf);
    if (ret != ESP_OK) return BSW_FAIL;
    
    ret = i2c_driver_install((i2c_port_t)i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) return BSW_FAIL;
    
    pthread_mutex_lock(&bsw_mutex);
    i2c_initialized[i2c_num] = true;
    pthread_mutex_unlock(&bsw_mutex);
    
    BSW_LOGI("MOCK_I2C", "I2C%d initialized (SDA=%d, SCL=%d, Speed=%d)", 
             i2c_num, config->sda_pin, config->scl_pin, config->clk_speed);
    return BSW_OK;
}

bsw_err_t bsw_i2c_write(bsw_i2c_num_t i2c_num, uint8_t addr, const uint8_t* data, size_t size) {
    if (i2c_num >= BSW_I2C_NUM_MAX || !i2c_initialized[i2c_num]) return BSW_ERR_INVALID_STATE;
    
    esp_err_t ret = i2c_master_write_to_device((i2c_port_t)i2c_num, addr, data, size, 1000);
    return (ret == ESP_OK) ? BSW_OK : BSW_FAIL;
}

bsw_err_t bsw_i2c_read(bsw_i2c_num_t i2c_num, uint8_t addr, uint8_t* data, size_t size) {
    if (i2c_num >= BSW_I2C_NUM_MAX || !i2c_initialized[i2c_num]) return BSW_ERR_INVALID_STATE;
    
    esp_err_t ret = i2c_master_read_from_device((i2c_port_t)i2c_num, addr, data, size, 1000);
    return (ret == ESP_OK) ? BSW_OK : BSW_FAIL;
}

bsw_err_t bsw_i2c_write_read(bsw_i2c_num_t i2c_num, uint8_t addr,
                            const uint8_t* write_data, size_t write_size,
                            uint8_t* read_data, size_t read_size) {
    if (i2c_num >= BSW_I2C_NUM_MAX || !i2c_initialized[i2c_num]) return BSW_ERR_INVALID_STATE;
    
    // Write register address
    esp_err_t ret = i2c_master_write_to_device((i2c_port_t)i2c_num, addr, write_data, write_size, 1000);
    if (ret != ESP_OK) return BSW_FAIL;
    
    // Read data
    ret = i2c_master_read_from_device((i2c_port_t)i2c_num, addr, read_data, read_size, 1000);
    return (ret == ESP_OK) ? BSW_OK : BSW_FAIL;
}

// ============================================================================
// BSW PWM Mock Implementation
// ============================================================================
bsw_err_t bsw_pwm_init(void) {
    pthread_mutex_lock(&bsw_mutex);
    pwm_initialized = true;
    // Initialize all channels as inactive
    for (int i = 0; i < BSW_PWM_CHANNEL_MAX; i++) {
        pwm_channels[i].active = false;
        pwm_channels[i].duty = 0;
        pwm_channels[i].frequency = 1000; // Default 1kHz
        pwm_channels[i].pin = 0;
    }
    pthread_mutex_unlock(&bsw_mutex);
    
    BSW_LOGI("MOCK_PWM", "PWM subsystem initialized");
    return BSW_OK;
}

bsw_err_t bsw_pwm_config_channel(bsw_pwm_channel_t channel, const bsw_pwm_config_t* config) {
    if (channel >= BSW_PWM_CHANNEL_MAX || !pwm_initialized) return BSW_ERR_INVALID_ARG;
    
    pthread_mutex_lock(&bsw_mutex);
    pwm_channels[channel].pin = config->pin;
    pwm_channels[channel].frequency = config->freq_hz;
    pthread_mutex_unlock(&bsw_mutex);
    
    BSW_LOGI("MOCK_PWM", "PWM Channel %d configured: Pin=%d, Freq=%dHz", 
             channel, config->pin, config->freq_hz);
    return BSW_OK;
}

bsw_err_t bsw_pwm_set_duty(bsw_pwm_channel_t channel, uint32_t duty) {
    if (channel >= BSW_PWM_CHANNEL_MAX || !pwm_initialized) return BSW_ERR_INVALID_ARG;
    
    pthread_mutex_lock(&bsw_mutex);
    pwm_channels[channel].duty = duty;
    pthread_mutex_unlock(&bsw_mutex);
    
    // Simulate motor control output
    if (pwm_channels[channel].active) {
        float duty_percent = (float)duty / 255.0f * 100.0f;
        BSW_LOGD("MOCK_PWM", "Motor Channel %d: %.1f%% duty", channel, duty_percent);
    }
    
    return BSW_OK;
}

bsw_err_t bsw_pwm_start(bsw_pwm_channel_t channel) {
    if (channel >= BSW_PWM_CHANNEL_MAX || !pwm_initialized) return BSW_ERR_INVALID_ARG;
    
    pthread_mutex_lock(&bsw_mutex);
    pwm_channels[channel].active = true;
    pthread_mutex_unlock(&bsw_mutex);
    
    BSW_LOGI("MOCK_PWM", "PWM Channel %d started", channel);
    return BSW_OK;
}

bsw_err_t bsw_pwm_stop(bsw_pwm_channel_t channel) {
    if (channel >= BSW_PWM_CHANNEL_MAX || !pwm_initialized) return BSW_ERR_INVALID_ARG;
    
    pthread_mutex_lock(&bsw_mutex);
    pwm_channels[channel].active = false;
    pwm_channels[channel].duty = 0;
    pthread_mutex_unlock(&bsw_mutex);
    
    BSW_LOGI("MOCK_PWM", "PWM Channel %d stopped", channel);
    return BSW_OK;
}

// ============================================================================
// BSW ADC Mock Implementation  
// ============================================================================
bsw_err_t bsw_adc_init(void) {
    pthread_mutex_lock(&bsw_mutex);
    adc_initialized = true;
    // Simulate realistic battery voltage readings (~3.7V for LiPo)
    // 3.7V / 3.3V * 4096 = ~4648 (but we use voltage divider, so ~2324)
    for (int unit = 0; unit < 2; unit++) {
        for (int ch = 0; ch < BSW_ADC_CHANNEL_MAX; ch++) {
            adc_values[unit][ch] = 2324; // Simulate ~3.7V battery through voltage divider
        }
    }
    pthread_mutex_unlock(&bsw_mutex);
    
    BSW_LOGI("MOCK_ADC", "ADC subsystem initialized");
    return BSW_OK;
}

bsw_err_t bsw_adc_config_channel(bsw_adc_unit_t unit, bsw_adc_channel_t channel) {
    if (unit >= 2 || channel >= BSW_ADC_CHANNEL_MAX || !adc_initialized) {
        return BSW_ERR_INVALID_ARG;
    }
    
    BSW_LOGI("MOCK_ADC", "ADC Unit%d Channel%d configured", unit, channel);
    return BSW_OK;
}

uint32_t bsw_adc_read(bsw_adc_unit_t unit, bsw_adc_channel_t channel) {
    if (unit >= 2 || channel >= BSW_ADC_CHANNEL_MAX || !adc_initialized) {
        return 0;
    }
    
    pthread_mutex_lock(&bsw_mutex);
    uint32_t value = adc_values[unit][channel];
    
    // Add some noise to simulate real ADC readings
    int noise = (rand() % 20) - 10; // ±10 ADC counts noise
    value = (uint32_t)((int)value + noise);
    if (value > 4095) value = 4095;  // 12-bit ADC max
    
    pthread_mutex_unlock(&bsw_mutex);
    
    return value;
}

// ============================================================================
// BSW UART Mock Implementation
// ============================================================================
bsw_err_t bsw_uart_init(bsw_uart_num_t uart_num, const bsw_uart_config_t* config) {
    BSW_LOGI("MOCK_UART", "UART%d initialized: %d baud, TX=%d, RX=%d", 
             uart_num, config->baudrate, config->tx_pin, config->rx_pin);
    return BSW_OK;
}

bsw_err_t bsw_uart_write(bsw_uart_num_t uart_num, const uint8_t* data, size_t size) {
    printf("[MOCK_UART%d_TX] ", uart_num);
    for (size_t i = 0; i < size; i++) {
        printf("%c", data[i]);
    }
    printf("\n");
    return BSW_OK;
}

size_t bsw_uart_read(bsw_uart_num_t uart_num, uint8_t* buffer, size_t size, uint32_t timeout_ms) {
    // Simulate GPS NMEA data for GPS sensor
    if (uart_num == BSW_UART_NUM_1) {
        static int gps_counter = 0;
        gps_counter++;
        
        // Every 10th call, return some mock NMEA data
        if (gps_counter % 10 == 0) {
            const char* nmea = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
            size_t nmea_len = strlen(nmea);
            size_t copy_len = (size < nmea_len) ? size : nmea_len;
            memcpy(buffer, nmea, copy_len);
            BSW_LOGD("MOCK_GPS", "Generated NMEA data");
            return copy_len;
        }
    }
    
    // Default: no data available
    return 0;
}

// ============================================================================
// BSW System Services Mock Implementation
// ============================================================================
void bsw_log_print(bsw_log_level_t level, const char* tag, const char* format, ...) {
    const char* level_str[] = {"ERROR", "WARN", "INFO", "DEBUG"};
    
    printf("[%s][%s] ", level_str[level], tag);
    
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    
    printf("\n");
}

uint32_t bsw_get_tick_ms(void) {
    return mock_get_time_ms();
}

uint32_t bsw_get_tick_us(void) {
    return mock_get_time_us();
}

void bsw_delay_ms(uint32_t ms) {
    usleep(ms * 1000);
}

void bsw_delay_us(uint32_t us) {
    usleep(us);
}