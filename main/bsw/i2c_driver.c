/**
 * @file i2c_driver.c
 * @brief ESP32-C6 Hardware I2C Controller Direct Register Control Implementation
 * 
 * ESP32-C6 I2C 하드웨어 컨트롤러의 레지스터를 직접 제어하는 드라이버 구현입니다.
 * MPU6050 IMU 센서와의 통신에 최적화되어 있으며, CPU 부하를 최소화합니다.
 * 
 * 구현 특징:
 * - ESP32-C6 I2C 컨트롤러 레지스터 직접 제어
 * - GPIO Matrix를 통한 유연한 핀 매핑 (하드웨어 I2C 신호 연결)
 * - 하드웨어 FIFO 기반 데이터 송수신 (32바이트 초과 자동 분할)
 * - COMMAND 레지스터 기반 트랜잭션 제어
 * - FreeRTOS Mutex를 통한 멀티태스크 동기화
 * - 자동 버스 복구 메커니즘 (SCL 클럭 펄스 생성)
 * - 동적 APB 클럭 주파수 감지
 * - 하드웨어 클럭 생성 (정확한 타이밍)
 * - CPU 부하 최소화 (하드웨어가 자동 처리)
 * - 고속 통신 지원 (100kHz, 400kHz, 1MHz)
 * 
 * 개선 사항 (v6.0):
 * ✅ GPIO Matrix를 통한 I2C 신호 매핑 구현
 * ✅ Mutex 기반 멀티태스크 동시성 제어
 * ✅ FIFO 32바이트 초과 데이터 자동 분할 처리
 * ✅ I2C 버스 hang 시 자동 복구 메커니즘
 * ✅ 동적 APB 클럭 주파수 감지
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 6.1 (CPU Blocking Removed)
 */

#include "i2c_driver.h"
#include "system_services.h"
#include "gpio_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "hal/gpio_hal.h"
#include "esp_clk_tree.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char* I2C_TAG = "HW_I2C";

// I2C Hardware Controller Base Addresses
// ESP32-C6 only has I2C0, I2C1 does not exist
static const uint32_t i2c_base_addrs[BSW_I2C_PORT_MAX] = {
    I2C0_BASE_ADDR,
    0  // I2C1 not available on ESP32-C6
};

// I2C Port Configuration Storage
static i2c_hw_config_t i2c_configs[BSW_I2C_PORT_MAX];
static bool i2c_initialized[BSW_I2C_PORT_MAX] = {false};

// Mutex for thread-safe I2C access
static SemaphoreHandle_t i2c_mutex[BSW_I2C_PORT_MAX] = {NULL};

// APB Clock frequency cache
static uint32_t apb_freq_hz = 0;

// Forward declarations of internal functions
static esp_err_t i2c_read_register_internal(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);
static esp_err_t i2c_write_raw_internal(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len);
static esp_err_t i2c_read_raw_internal(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len);

/**
 * @brief Get APB clock frequency dynamically
 * 
 * @return uint32_t APB clock frequency in Hz
 */
static uint32_t i2c_get_apb_freq(void) {
    if (apb_freq_hz == 0) {
        // ESP32-C6 uses XTAL (40MHz) as APB clock source
        esp_err_t ret = esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_XTAL, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &apb_freq_hz);
        if (ret != ESP_OK || apb_freq_hz == 0) {
            // Fallback to default 40MHz if unable to read
            apb_freq_hz = 40000000;
            BSW_LOGW(I2C_TAG, "Failed to get APB freq, using default 40MHz");
        } else {
            BSW_LOGI(I2C_TAG, "APB Clock Frequency: %lu Hz", (uint32_t)apb_freq_hz);
        }
    }
    return apb_freq_hz;
}

/**
 * @brief Get I2C hardware controller base address
 * 
 * @param port I2C port number
 * @return uint32_t Base address or 0 if invalid
 */
static inline uint32_t i2c_get_base_addr(bsw_i2c_port_t port) {
    if (port >= BSW_I2C_PORT_MAX) return 0;
    return i2c_base_addrs[port];
}

/**
 * @brief Wait for I2C transaction complete with timeout
 * 
 * @param base I2C controller base address
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK or ESP_ERR_TIMEOUT
 */
static esp_err_t i2c_wait_trans_complete(uint32_t base, uint32_t timeout_ms) {
    TickType_t start_tick = xTaskGetTickCount();
    
    while (1) {
        uint32_t int_status = I2C_READ_REG(base, I2C_INT_RAW_REG_OFFSET);
        
        // Check for transaction complete
        if (int_status & I2C_INT_TRANS_COMPLETE_BIT) {
            // Clear interrupt
            I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, I2C_INT_TRANS_COMPLETE_BIT);
            return ESP_OK;
        }
        
        // Check for errors
        if (int_status & (I2C_INT_NACK_BIT | I2C_INT_TIME_OUT_BIT | I2C_INT_ARBITRATION_LOST_BIT)) {
            // Clear all interrupts
            I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
            BSW_LOGE(I2C_TAG, "I2C error: int_status=0x%lx", int_status);
            return ESP_FAIL;
        }
        
        // Check timeout
        TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
        if (pdTICKS_TO_MS(elapsed_ticks) >= timeout_ms) {
            BSW_LOGE(I2C_TAG, "I2C transaction timeout after %lu ms", pdTICKS_TO_MS(elapsed_ticks));
            return ESP_ERR_TIMEOUT;
        }
        
        // Yield CPU to other tasks (FreeRTOS-safe)
        vTaskDelay(1);
    }
}

/**
 * @brief Calculate I2C timing parameters based on clock speed
 * 
 * @param clock_speed Clock speed in Hz
 * @param scl_low_period Output: SCL low period
 * @param scl_high_period Output: SCL high period
 * @param sda_hold Output: SDA hold time
 * @param sda_sample Output: SDA sample time
 */
static void i2c_calc_timing_params(uint32_t clock_speed, 
                                    uint32_t* scl_low_period,
                                    uint32_t* scl_high_period,
                                    uint32_t* sda_hold,
                                    uint32_t* sda_sample) {
    // I2C source clock: Get APB_CLK dynamically
    const uint32_t source_clk = i2c_get_apb_freq();
    
    // Calculate period in source clock cycles
    uint32_t period = source_clk / clock_speed;
    
    // Standard timing ratios (based on I2C specification)
    // SCL low:high ratio is approximately 1:1 for standard/fast mode
    *scl_low_period = period / 2;
    *scl_high_period = period / 2;
    
    // SDA hold time: typically 1/4 of period
    *sda_hold = period / 4;
    
    // SDA sample time: typically 1/2 of SCL high period
    *sda_sample = *scl_high_period / 2;
}

/**
 * @brief Map GPIO pins to I2C controller function via GPIO Matrix
 * 
 * @param port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @return esp_err_t Success or failure
 */
static esp_err_t i2c_gpio_matrix_config(bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin) {
    // ESP32-C6 has only I2C0 controller
    // I2C signal indices for GPIO matrix
    if (port != BSW_I2C_PORT_0) {
        BSW_LOGE(I2C_TAG, "ESP32-C6 only supports I2C port 0");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Connect I2C SDA output signal to GPIO
    esp_rom_gpio_connect_out_signal((uint32_t)sda_pin, I2CEXT0_SDA_OUT_IDX, false, false);
    // Connect GPIO to I2C SDA input signal
    esp_rom_gpio_connect_in_signal((uint32_t)sda_pin, I2CEXT0_SDA_IN_IDX, false);
    
    // Connect I2C SCL output signal to GPIO
    esp_rom_gpio_connect_out_signal((uint32_t)scl_pin, I2CEXT0_SCL_OUT_IDX, false, false);
    // Connect GPIO to I2C SCL input signal
    esp_rom_gpio_connect_in_signal((uint32_t)scl_pin, I2CEXT0_SCL_IN_IDX, false);
    
    BSW_LOGI(I2C_TAG, "GPIO Matrix configured: SDA=%d, SCL=%d for I2C%d", sda_pin, scl_pin, port);
    return ESP_OK;
}

/**
 * @brief I2C Bus Recovery - Generate SCL clock pulses to release stuck bus
 * 
 * @param port I2C port number
 * @return esp_err_t Recovery result
 */
esp_err_t i2c_bus_recovery(bsw_i2c_port_t port) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port]) {
        return ESP_ERR_INVALID_STATE;
    }
    
    bsw_gpio_num_t sda_pin = i2c_configs[port].sda_pin;
    bsw_gpio_num_t scl_pin = i2c_configs[port].scl_pin;
    
    BSW_LOGW(I2C_TAG, "Starting I2C bus recovery on port %d", port);
    
    // Temporarily configure pins as GPIO outputs
    bsw_gpio_set_direction(scl_pin, BSW_GPIO_MODE_OUTPUT);
    bsw_gpio_set_direction(sda_pin, BSW_GPIO_MODE_INPUT);
    
    // Generate 9 clock pulses to release any stuck slave
    for (int i = 0; i < 9; i++) {
        bsw_gpio_set_level(scl_pin, 0);
        esp_rom_delay_us(5);
        bsw_gpio_set_level(scl_pin, 1);
        esp_rom_delay_us(5);
        
        // Check if SDA is released (high)
        if (bsw_gpio_get_level(sda_pin) == 1) {
            BSW_LOGI(I2C_TAG, "Bus recovered after %d clock pulses", i + 1);
            break;
        }
    }
    
    // Generate STOP condition
    bsw_gpio_set_direction(sda_pin, BSW_GPIO_MODE_OUTPUT);
    bsw_gpio_set_level(sda_pin, 0);
    esp_rom_delay_us(5);
    bsw_gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(5);
    bsw_gpio_set_level(sda_pin, 1);
    esp_rom_delay_us(5);
    
    // Reconfigure pins for I2C function
    bsw_gpio_config_pin(sda_pin, BSW_GPIO_MODE_INPUT_OUTPUT, 
                       i2c_configs[port].use_pullup ? BSW_GPIO_PULLUP_ENABLE : BSW_GPIO_PULLUP_DISABLE, 
                       BSW_GPIO_PULLDOWN_DISABLE);
    bsw_gpio_config_pin(scl_pin, BSW_GPIO_MODE_INPUT_OUTPUT, 
                       i2c_configs[port].use_pullup ? BSW_GPIO_PULLUP_ENABLE : BSW_GPIO_PULLUP_DISABLE, 
                       BSW_GPIO_PULLDOWN_DISABLE);
    
    i2c_gpio_matrix_config(port, sda_pin, scl_pin);
    
    BSW_LOGI(I2C_TAG, "Bus recovery completed");
    return ESP_OK;
}

/**
 * @brief Reset I2C controller hardware
 * 
 * @param base I2C controller base address
 */
static void i2c_hw_reset(uint32_t base) {
    // Reset FIFO
    I2C_SET_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));  // TX/RX FIFO reset
    esp_rom_delay_us(10);
    I2C_CLEAR_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
    
    // Clear all interrupts
    I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    // Reset control register (disable master mode temporarily)
    I2C_CLEAR_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_MS_MODE_BIT);
}

/**
 * @brief I2C Hardware Controller Initialization (Default 100kHz)
 * 
 * @param port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @return esp_err_t Initialization result
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin) {
    i2c_hw_config_t config = {
        .sda_pin = sda_pin,
        .scl_pin = scl_pin,
        .clock_speed = I2C_DEFAULT_CLOCK_SPEED,
        .use_pullup = true,
        .timeout_ms = I2C_DEFAULT_TIMEOUT_MS
    };
    
    return i2c_driver_init_config(port, &config);
}

/**
 * @brief I2C Hardware Controller Initialization (Custom Config)
 * 
 * @param port I2C port number
 * @param config I2C hardware configuration
 * @return esp_err_t Initialization result
 */
esp_err_t i2c_driver_init_config(bsw_i2c_port_t port, const i2c_hw_config_t* config) {
    if (port >= BSW_I2C_PORT_MAX || !config) {
        BSW_LOGE(I2C_TAG, "Invalid I2C port %d or config is NULL", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    // ESP32-C6 only supports I2C0
    if (port != BSW_I2C_PORT_0) {
        BSW_LOGE(I2C_TAG, "ESP32-C6 only supports I2C0. I2C%d is not available.", port);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    uint32_t base = i2c_get_base_addr(port);
    if (base == 0) {
        BSW_LOGE(I2C_TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create mutex for thread-safe access if not exists
    if (i2c_mutex[port] == NULL) {
        i2c_mutex[port] = xSemaphoreCreateMutex();
        if (i2c_mutex[port] == NULL) {
            BSW_LOGE(I2C_TAG, "Failed to create mutex for I2C port %d", port);
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Save configuration
    i2c_configs[port] = *config;
    
    // Reset I2C controller
    i2c_hw_reset(base);
    
    // Configure GPIO pins for I2C function (open-drain with pullup)
    esp_err_t ret = bsw_gpio_config_pin(config->sda_pin, BSW_GPIO_MODE_INPUT_OUTPUT, 
                                       config->use_pullup ? BSW_GPIO_PULLUP_ENABLE : BSW_GPIO_PULLUP_DISABLE, 
                                       BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Failed to configure SDA pin %d", config->sda_pin);
        return ret;
    }
    
    ret = bsw_gpio_config_pin(config->scl_pin, BSW_GPIO_MODE_INPUT_OUTPUT, 
                             config->use_pullup ? BSW_GPIO_PULLUP_ENABLE : BSW_GPIO_PULLUP_DISABLE, 
                             BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Failed to configure SCL pin %d", config->scl_pin);
        return ret;
    }
    
    // Map GPIO pins to I2C controller function via GPIO Matrix
    ret = i2c_gpio_matrix_config(port, config->sda_pin, config->scl_pin);
    if (ret != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Failed to configure GPIO matrix for I2C port %d", port);
        return ret;
    }
    
    // Calculate timing parameters
    uint32_t scl_low, scl_high, sda_hold, sda_sample;
    i2c_calc_timing_params(config->clock_speed, &scl_low, &scl_high, &sda_hold, &sda_sample);
    
    // Configure I2C timing registers
    I2C_WRITE_REG(base, I2C_SCL_LOW_PERIOD_REG_OFFSET, scl_low);
    I2C_WRITE_REG(base, I2C_SCL_HIGH_PERIOD_REG_OFFSET, scl_high);
    I2C_WRITE_REG(base, I2C_SDA_HOLD_REG_OFFSET, sda_hold);
    I2C_WRITE_REG(base, I2C_SDA_SAMPLE_REG_OFFSET, sda_sample);
    
    // Configure START/STOP timing
    I2C_WRITE_REG(base, I2C_SCL_START_HOLD_REG_OFFSET, scl_high / 2);
    I2C_WRITE_REG(base, I2C_SCL_RSTART_SETUP_REG_OFFSET, scl_high / 2);
    I2C_WRITE_REG(base, I2C_SCL_STOP_HOLD_REG_OFFSET, scl_high / 2);
    I2C_WRITE_REG(base, I2C_SCL_STOP_SETUP_REG_OFFSET, scl_high / 2);
    
    // Configure FIFO (TX/RX thresholds)
    uint32_t fifo_conf = (1 << 2) |  // TX FIFO empty threshold = 1
                         (15 << 0);   // RX FIFO full threshold = 15
    I2C_WRITE_REG(base, I2C_FIFO_CONF_REG_OFFSET, fifo_conf);
    
    // Configure timeout (use dynamic APB clock)
    uint32_t apb_freq = i2c_get_apb_freq();
    uint32_t timeout_val = (config->timeout_ms * (apb_freq / 1000)) / 16;  // Convert ms to I2C clock cycles
    I2C_WRITE_REG(base, I2C_TO_REG_OFFSET, timeout_val);
    
    // Enable master mode and set MSB first
    uint32_t ctr_val = I2C_CTR_MS_MODE_BIT;  // Master mode
    I2C_WRITE_REG(base, I2C_CTR_REG_OFFSET, ctr_val);
    
    // Clear all interrupts
    I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    i2c_initialized[port] = true;
    
    BSW_LOGI(I2C_TAG, "I2C HW port %d initialized: SDA=%d, SCL=%d, freq=%luHz", 
             port, config->sda_pin, config->scl_pin, (uint32_t)config->clock_speed);
    
    return ESP_OK;
}

/**
 * @brief I2C Device Register Write (Hardware Controller)
 * 
 * Uses I2C hardware controller COMMAND registers and FIFO to write data.
 * 
 * Protocol: START -> ADDR+W -> REG -> DATA -> STOP
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address
 * @param value Data value to write
 * @return esp_err_t Transmission result
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port]) {
        BSW_LOGE(I2C_TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Acquire mutex for thread-safe access
    if (i2c_mutex[port] != NULL) {
        if (xSemaphoreTake(i2c_mutex[port], pdMS_TO_TICKS(i2c_configs[port].timeout_ms)) != pdTRUE) {
            BSW_LOGE(I2C_TAG, "Failed to acquire I2C mutex for port %d", port);
            return ESP_ERR_TIMEOUT;
        }
    }

    uint32_t base = i2c_get_base_addr(port);
    
    // Reset FIFO
    I2C_SET_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0));  // TX FIFO reset
    esp_rom_delay_us(1);
    I2C_CLEAR_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0));
    
    // Write data to TX FIFO: device address + W, register address, data value
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, (device_addr << 1));  // Address + Write bit (0)
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, reg_addr);
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, value);
    
    // Configure command sequence
    // Command 0: RSTART (Restart/Start condition)
    I2C_WRITE_REG(base, I2C_COMD0_REG_OFFSET, (I2C_CMD_RSTART << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 1: WRITE 3 bytes (device addr + reg addr + data)
    I2C_WRITE_REG(base, I2C_COMD1_REG_OFFSET, 
                  (I2C_CMD_WRITE << I2C_COMMAND_OPCODE_SHIFT) | 3);
    
    // Command 2: STOP
    I2C_WRITE_REG(base, I2C_COMD2_REG_OFFSET, (I2C_CMD_STOP << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 3: END
    I2C_WRITE_REG(base, I2C_COMD3_REG_OFFSET, (I2C_CMD_END << I2C_COMMAND_OPCODE_SHIFT));
    
    // Clear interrupts before starting
    I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    // Start transaction
    I2C_SET_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    // Wait for completion
    esp_err_t result = i2c_wait_trans_complete(base, i2c_configs[port].timeout_ms);
    
    // Clear transaction start bit
    I2C_CLEAR_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    if (result != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Write register failed: dev=0x%02X, reg=0x%02X", device_addr, reg_addr);
        // Attempt bus recovery on failure
        i2c_bus_recovery(port);
    }
    
    // Release mutex
    if (i2c_mutex[port] != NULL) {
        xSemaphoreGive(i2c_mutex[port]);
    }
    
    return result;
}

/**
 * @brief I2C Device Register Read (Hardware Controller)
 * 
 * Uses I2C hardware controller COMMAND registers and FIFO to read data.
 * 
 * Protocol: START -> ADDR+W -> REG -> RESTART -> ADDR+R -> DATA... -> STOP
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return esp_err_t Transmission result
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port] || !data || len == 0) {
        BSW_LOGE(I2C_TAG, "Invalid parameters: port=%d, data=%p, len=%zu", port, data, len);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Acquire mutex for thread-safe access
    if (i2c_mutex[port] != NULL) {
        if (xSemaphoreTake(i2c_mutex[port], pdMS_TO_TICKS(i2c_configs[port].timeout_ms)) != pdTRUE) {
            BSW_LOGE(I2C_TAG, "Failed to acquire I2C mutex for port %d", port);
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // Handle large reads by splitting into multiple transactions
    const size_t max_read_size = 28;  // Leave some margin from 32-byte FIFO
    size_t bytes_read = 0;
    esp_err_t final_result = ESP_OK;
    
    while (bytes_read < len) {
        size_t chunk_size = (len - bytes_read) > max_read_size ? max_read_size : (len - bytes_read);
        uint8_t chunk_reg_addr = reg_addr + bytes_read;
        
        esp_err_t chunk_result = i2c_read_register_internal(port, device_addr, chunk_reg_addr, 
                                                             &data[bytes_read], chunk_size);
        if (chunk_result != ESP_OK) {
            final_result = chunk_result;
            break;
        }
        
        bytes_read += chunk_size;
    }
    
    // Release mutex
    if (i2c_mutex[port] != NULL) {
        xSemaphoreGive(i2c_mutex[port]);
    }
    
    return final_result;
}

/**
 * @brief Internal I2C register read (without mutex, for chunked reads)
 */
static esp_err_t i2c_read_register_internal(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    uint32_t base = i2c_get_base_addr(port);
    
    // Reset FIFO
    I2C_SET_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));  // TX/RX FIFO reset
    esp_rom_delay_us(1);
    I2C_CLEAR_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
    
    // Phase 1: Write register address
    // Write device address + W and register address to TX FIFO
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, (device_addr << 1));  // Address + Write bit (0)
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, reg_addr);
    
    // Command 0: START
    I2C_WRITE_REG(base, I2C_COMD0_REG_OFFSET, (I2C_CMD_RSTART << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 1: WRITE 2 bytes (device addr + reg addr)
    I2C_WRITE_REG(base, I2C_COMD1_REG_OFFSET, 
                  (I2C_CMD_WRITE << I2C_COMMAND_OPCODE_SHIFT) | 2);
    
    // Phase 2: Read data
    // Write device address + R to TX FIFO
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, (device_addr << 1) | 0x01);  // Address + Read bit
    
    // Command 2: RESTART
    I2C_WRITE_REG(base, I2C_COMD2_REG_OFFSET, (I2C_CMD_RSTART << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 3: WRITE 1 byte (device addr + R)
    I2C_WRITE_REG(base, I2C_COMD3_REG_OFFSET, 
                  (I2C_CMD_WRITE << I2C_COMMAND_OPCODE_SHIFT) | 1);
    
    // Command 4: READ len bytes
    I2C_WRITE_REG(base, I2C_COMD4_REG_OFFSET, 
                  (I2C_CMD_READ << I2C_COMMAND_OPCODE_SHIFT) | (len & 0xFF));
    
    // Command 5: STOP
    I2C_WRITE_REG(base, I2C_COMD5_REG_OFFSET, (I2C_CMD_STOP << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 6: END
    I2C_WRITE_REG(base, I2C_COMD6_REG_OFFSET, (I2C_CMD_END << I2C_COMMAND_OPCODE_SHIFT));
    
    // Clear interrupts before starting
    I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    // Start transaction
    I2C_SET_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    // Wait for completion
    esp_err_t result = i2c_wait_trans_complete(base, i2c_configs[port].timeout_ms);
    
    // Clear transaction start bit
    I2C_CLEAR_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    if (result != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Read register failed: dev=0x%02X, reg=0x%02X", device_addr, reg_addr);
        // Attempt bus recovery on failure
        i2c_bus_recovery(port);
        return result;
    }
    
    // Read data from RX FIFO
    for (size_t i = 0; i < len; i++) {
        data[i] = (uint8_t)I2C_READ_REG(base, I2C_DATA_REG_OFFSET);
    }
    
    return ESP_OK;
}

/**
 * @brief I2C Raw Data Write (Hardware Controller)
 * 
 * Uses I2C hardware controller to write raw data bytes.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param data Data buffer to write
 * @param len Number of bytes to write
 * @return esp_err_t Transmission result
 */
esp_err_t i2c_write_raw(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port] || !data || len == 0) {
        BSW_LOGE(I2C_TAG, "Invalid parameters for raw write");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Acquire mutex for thread-safe access
    if (i2c_mutex[port] != NULL) {
        if (xSemaphoreTake(i2c_mutex[port], pdMS_TO_TICKS(i2c_configs[port].timeout_ms)) != pdTRUE) {
            BSW_LOGE(I2C_TAG, "Failed to acquire I2C mutex for port %d", port);
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // Handle large writes by splitting into chunks
    const size_t max_write_size = 30;  // 32-byte FIFO - 1 for address - 1 margin
    size_t bytes_written = 0;
    esp_err_t final_result = ESP_OK;
    
    while (bytes_written < len) {
        size_t chunk_size = (len - bytes_written) > max_write_size ? max_write_size : (len - bytes_written);
        
        esp_err_t chunk_result = i2c_write_raw_internal(port, device_addr, &data[bytes_written], chunk_size);
        if (chunk_result != ESP_OK) {
            final_result = chunk_result;
            break;
        }
        
        bytes_written += chunk_size;
    }
    
    // Release mutex
    if (i2c_mutex[port] != NULL) {
        xSemaphoreGive(i2c_mutex[port]);
    }
    
    return final_result;
}

/**
 * @brief Internal I2C raw write (without mutex, for chunked writes)
 */
static esp_err_t i2c_write_raw_internal(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len) {
    uint32_t base = i2c_get_base_addr(port);
    
    // Reset FIFO
    I2C_SET_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
    esp_rom_delay_us(1);
    I2C_CLEAR_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
    
    // Write device address + W to FIFO
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, (device_addr << 1));  // Write bit is 0
    
    // Write data bytes to FIFO (up to 31 bytes, 1 for address)
    size_t fifo_capacity = 31;  // 32-byte FIFO - 1 for address
    size_t bytes_to_write = (len < fifo_capacity) ? len : fifo_capacity;
    
    for (size_t i = 0; i < bytes_to_write; i++) {
        I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, data[i]);
    }
    
    // Command 0: START
    I2C_WRITE_REG(base, I2C_COMD0_REG_OFFSET, (I2C_CMD_RSTART << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 1: WRITE (1 + len) bytes
    I2C_WRITE_REG(base, I2C_COMD1_REG_OFFSET, 
                  (I2C_CMD_WRITE << I2C_COMMAND_OPCODE_SHIFT) | ((1 + bytes_to_write) & 0xFF));
    
    // Command 2: STOP
    I2C_WRITE_REG(base, I2C_COMD2_REG_OFFSET, (I2C_CMD_STOP << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 3: END
    I2C_WRITE_REG(base, I2C_COMD3_REG_OFFSET, (I2C_CMD_END << I2C_COMMAND_OPCODE_SHIFT));
    
    // Clear interrupts
    I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    // Start transaction
    I2C_SET_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    // Wait for completion
    esp_err_t result = i2c_wait_trans_complete(base, i2c_configs[port].timeout_ms);
    
    // Clear transaction start bit
    I2C_CLEAR_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    if (result != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Raw write failed: dev=0x%02X, len=%zu", device_addr, len);
        // Attempt bus recovery on failure
        i2c_bus_recovery(port);
        return result;
    }
    
    return ESP_OK;
}

/**
 * @brief I2C Raw Data Read (Hardware Controller)
 * 
 * Uses I2C hardware controller to read raw data bytes.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return esp_err_t Transmission result
 */
esp_err_t i2c_read_raw(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port] || !data || len == 0) {
        BSW_LOGE(I2C_TAG, "Invalid parameters for raw read");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Acquire mutex for thread-safe access
    if (i2c_mutex[port] != NULL) {
        if (xSemaphoreTake(i2c_mutex[port], pdMS_TO_TICKS(i2c_configs[port].timeout_ms)) != pdTRUE) {
            BSW_LOGE(I2C_TAG, "Failed to acquire I2C mutex for port %d", port);
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // Handle large reads by splitting into chunks
    const size_t max_read_size = 31;  // 32-byte FIFO - 1 for address
    size_t bytes_read = 0;
    esp_err_t final_result = ESP_OK;
    
    while (bytes_read < len) {
        size_t chunk_size = (len - bytes_read) > max_read_size ? max_read_size : (len - bytes_read);
        
        esp_err_t chunk_result = i2c_read_raw_internal(port, device_addr, &data[bytes_read], chunk_size);
        if (chunk_result != ESP_OK) {
            final_result = chunk_result;
            break;
        }
        
        bytes_read += chunk_size;
    }
    
    // Release mutex
    if (i2c_mutex[port] != NULL) {
        xSemaphoreGive(i2c_mutex[port]);
    }
    
    return final_result;
}

/**
 * @brief Internal I2C raw read (without mutex, for chunked reads)
 */
static esp_err_t i2c_read_raw_internal(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len) {
    uint32_t base = i2c_get_base_addr(port);
    
    // Reset FIFO
    I2C_SET_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
    esp_rom_delay_us(1);
    I2C_CLEAR_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
    
    // Write device address + R to FIFO
    I2C_WRITE_REG(base, I2C_DATA_REG_OFFSET, (device_addr << 1) | 0x01);
    
    // Limit read length to FIFO capacity
    size_t bytes_to_read = (len < 32) ? len : 32;
    
    // Command 0: START
    I2C_WRITE_REG(base, I2C_COMD0_REG_OFFSET, (I2C_CMD_RSTART << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 1: WRITE 1 byte (device address + R)
    I2C_WRITE_REG(base, I2C_COMD1_REG_OFFSET, 
                  (I2C_CMD_WRITE << I2C_COMMAND_OPCODE_SHIFT) | 1);
    
    // Command 2: READ len bytes
    I2C_WRITE_REG(base, I2C_COMD2_REG_OFFSET, 
                  (I2C_CMD_READ << I2C_COMMAND_OPCODE_SHIFT) | (bytes_to_read & 0xFF));
    
    // Command 3: STOP
    I2C_WRITE_REG(base, I2C_COMD3_REG_OFFSET, (I2C_CMD_STOP << I2C_COMMAND_OPCODE_SHIFT));
    
    // Command 4: END
    I2C_WRITE_REG(base, I2C_COMD4_REG_OFFSET, (I2C_CMD_END << I2C_COMMAND_OPCODE_SHIFT));
    
    // Clear interrupts
    I2C_WRITE_REG(base, I2C_INT_CLR_REG_OFFSET, 0xFFFFFFFF);
    
    // Start transaction
    I2C_SET_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    // Wait for completion
    esp_err_t result = i2c_wait_trans_complete(base, i2c_configs[port].timeout_ms);
    
    // Clear transaction start bit
    I2C_CLEAR_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_TRANS_START_BIT);
    
    if (result != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Raw read failed: dev=0x%02X, len=%zu", device_addr, len);
        // Attempt bus recovery on failure
        i2c_bus_recovery(port);
        return result;
    }
    
    // Read data from RX FIFO
    for (size_t i = 0; i < bytes_to_read; i++) {
        data[i] = (uint8_t)I2C_READ_REG(base, I2C_DATA_REG_OFFSET);
    }
    
    return ESP_OK;
}

/**
 * @brief I2C Driver Deinitialization (Hardware Controller)
 * 
 * Disables I2C hardware controller and restores GPIO pins.
 * 
 * @param port I2C port number
 * @return esp_err_t Deinitialization result
 */
esp_err_t i2c_driver_deinit(bsw_i2c_port_t port) {
    if (port >= BSW_I2C_PORT_MAX) {
        BSW_LOGE(I2C_TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (i2c_initialized[port]) {
        uint32_t base = i2c_get_base_addr(port);
        
        // Disable master mode
        I2C_CLEAR_BITS(base, I2C_CTR_REG_OFFSET, I2C_CTR_MS_MODE_BIT);
        
        // Reset FIFO
        I2C_SET_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
        esp_rom_delay_us(1);
        I2C_CLEAR_BITS(base, I2C_FIFO_CONF_REG_OFFSET, (1U << 0) | (1U << 1));
        
        // Disable all interrupts
        I2C_WRITE_REG(base, I2C_INT_ENA_REG_OFFSET, 0);
        
        // Restore GPIO pins to input mode (pulled high by external resistors)
        bsw_gpio_set_direction(i2c_configs[port].sda_pin, BSW_GPIO_MODE_INPUT);
        bsw_gpio_set_direction(i2c_configs[port].scl_pin, BSW_GPIO_MODE_INPUT);
        
        // Delete mutex
        if (i2c_mutex[port] != NULL) {
            vSemaphoreDelete(i2c_mutex[port]);
            i2c_mutex[port] = NULL;
        }
        
        i2c_initialized[port] = false;
        
        BSW_LOGI(I2C_TAG, "I2C port %d deinitialized (hardware controller disabled)", port);
    }
    
    return ESP_OK;
}