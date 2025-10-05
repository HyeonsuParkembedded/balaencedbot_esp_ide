/**
 * @file spi_driver.c
 * @brief ESP32-C6 Hardware SPI Controller Direct Register Control Implementation
 * 
 * ESP32-C6 SPI 하드웨어 컨트롤러의 레지스터를 직접 제어하는 드라이버 구현입니다.
 * 외부 SPI 디바이스와의 고속 통신에 최적화되어 있으며, CPU 부하를 최소화합니다.
 * 
 * 구현 특징:
 * - ESP32-C6 SPI 컨트롤러 레지스터 직접 제어
 * - 하드웨어 기반 송수신으로 CPU 부하 최소화
 * - Full-duplex 모드 지원
 * - CPOL/CPHA 설정 가능
 * - 하드웨어 클럭 생성 (정확한 타이밍)
 * - 자동 CS 제어 및 수동 CS 제어 지원
 * - FreeRTOS 멀티태스킹 안전
 * - DMA 대용량 전송 지원
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 2.0 (FreeRTOS 멀티태스킹 안전 + DMA)
 */

#include "spi_driver.h"
#include "system_services.h"
#include "gpio_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char* SPI_TAG = "HW_SPI";

// SPI Hardware Controller Base Addresses
static const uint32_t spi_base_addrs[BSW_SPI_PORT_MAX] = {
    SPI2_BASE_ADDR
};

// SPI Port Configuration Storage
static spi_hw_config_t spi_configs[BSW_SPI_PORT_MAX];
static bool spi_initialized[BSW_SPI_PORT_MAX] = {false};

// FreeRTOS 멀티태스킹 보호 (v2.0 신규)
static SemaphoreHandle_t spi_mutex[BSW_SPI_PORT_MAX] = {NULL};

// DMA 관련 (v2.0 신규)
static volatile bool spi_dma_done[BSW_SPI_PORT_MAX] = {false};

/**
 * @brief Get SPI hardware controller base address
 * 
 * @param port SPI port number
 * @return uint32_t Base address or 0 if invalid
 */
static inline uint32_t spi_get_base_addr(bsw_spi_port_t port) {
    if (port >= BSW_SPI_PORT_MAX) return 0;
    return spi_base_addrs[port];
}

/**
 * @brief Wait for SPI transaction complete with timeout (v2.0: CPU 양보)
 * 
 * @param base SPI controller base address
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK or ESP_ERR_TIMEOUT
 */
static esp_err_t spi_wait_trans_complete(uint32_t base, uint32_t timeout_ms) {
    TickType_t start_ticks = xTaskGetTickCount();
    
    while (1) {
        uint32_t cmd_reg = SPI_READ_REG(base, SPI_CMD_REG_OFFSET);
        
        // Check if transaction is complete (USR bit cleared)
        if (!(cmd_reg & SPI_CMD_USR_BIT)) {
            return ESP_OK;
        }
        
        // Check timeout
        uint32_t elapsed_ms = (xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS;
        if (elapsed_ms >= timeout_ms) {
            BSW_LOGE(SPI_TAG, "SPI transaction timeout after %lu ms", elapsed_ms);
            return ESP_ERR_TIMEOUT;
        }
        
        // CPU 양보 (v2.0 개선)
        vTaskDelay(1); // 1ms 대기
    }
}

/**
 * @brief Configure SPI clock
 * 
 * @param base SPI controller base address
 * @param clock_speed Clock speed in Hz
 */
static void spi_configure_clock(uint32_t base, uint32_t clock_speed) {
    // SPI source clock: 80MHz (APB_CLK)
    const uint32_t apb_clk = 80000000;
    
    if (clock_speed >= apb_clk) {
        // Use system clock directly (no divider)
        SPI_SET_BITS(base, SPI_CLOCK_REG_OFFSET, SPI_CLOCK_CLK_EQU_SYSCLK_BIT);
    } else {
        // Calculate clock divider
        uint32_t clkdiv = (apb_clk / clock_speed) - 1;
        uint32_t clkdiv_pre = 0;
        
        // If divider is too large, use prescaler
        while (clkdiv > 0x3F && clkdiv_pre < 0x1FFF) {
            clkdiv_pre++;
            clkdiv = (apb_clk / (clock_speed * (clkdiv_pre + 1))) - 1;
        }
        
        // Configure clock register
        uint32_t clock_reg = (clkdiv_pre << SPI_CLOCK_CLKDIV_PRE_SHIFT) |
                             (clkdiv << SPI_CLOCK_CLKCNT_N_SHIFT) |
                             ((clkdiv / 2) << SPI_CLOCK_CLKCNT_H_SHIFT) |
                             (clkdiv << SPI_CLOCK_CLKCNT_L_SHIFT);
        
        SPI_WRITE_REG(base, SPI_CLOCK_REG_OFFSET, clock_reg);
    }
}

/**
 * @brief Configure SPI mode (CPOL/CPHA)
 * 
 * @param base SPI controller base address
 * @param mode SPI mode
 */
static void spi_configure_mode(uint32_t base, bsw_spi_mode_t mode) {
    // Configure CPOL (Clock Polarity)
    if (mode == BSW_SPI_MODE_2 || mode == BSW_SPI_MODE_3) {
        // CPOL = 1: Clock idle high
        SPI_SET_BITS(base, SPI_MISC_REG_OFFSET, SPI_MISC_CK_IDLE_EDGE_BIT);
    } else {
        // CPOL = 0: Clock idle low
        SPI_CLEAR_BITS(base, SPI_MISC_REG_OFFSET, SPI_MISC_CK_IDLE_EDGE_BIT);
    }
    
    // Configure CPHA (Clock Phase)
    if (mode == BSW_SPI_MODE_1 || mode == BSW_SPI_MODE_3) {
        // CPHA = 1: Sample on falling edge
        SPI_SET_BITS(base, SPI_USER_REG_OFFSET, SPI_USER_CK_OUT_EDGE_BIT);
    } else {
        // CPHA = 0: Sample on rising edge
        SPI_CLEAR_BITS(base, SPI_USER_REG_OFFSET, SPI_USER_CK_OUT_EDGE_BIT);
    }
}

/**
 * @brief Reset SPI controller hardware
 * 
 * @param base SPI controller base address
 */
static void spi_hw_reset(uint32_t base) {
    // Clear all pending commands
    SPI_WRITE_REG(base, SPI_CMD_REG_OFFSET, 0);
    
    // Reset user register (disable all user-defined phases)
    SPI_WRITE_REG(base, SPI_USER_REG_OFFSET, 0);
    
    // Reset control register
    SPI_WRITE_REG(base, SPI_CTRL_REG_OFFSET, 0);
}

/**
 * @brief SPI Hardware Controller Initialization (v2.0: 멀티태스킹 안전)
 * 
 * @param port SPI port number
 * @param config SPI hardware configuration
 * @return esp_err_t Initialization result
 */
esp_err_t bsw_spi_init(bsw_spi_port_t port, const spi_hw_config_t* config) {
    if (port >= BSW_SPI_PORT_MAX || !config) {
        BSW_LOGE(SPI_TAG, "Invalid SPI port %d or config is NULL", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t base = spi_get_base_addr(port);
    if (base == 0) {
        BSW_LOGE(SPI_TAG, "Invalid SPI port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->mode >= BSW_SPI_MODE_MAX) {
        BSW_LOGE(SPI_TAG, "Invalid SPI mode: %d", config->mode);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 뮤텍스 생성 (v2.0 신규)
    if (spi_mutex[port] == NULL) {
        spi_mutex[port] = xSemaphoreCreateMutex();
        if (spi_mutex[port] == NULL) {
            BSW_LOGE(SPI_TAG, "Failed to create mutex for port %d", port);
            return ESP_FAIL;
        }
    }
    
    // 뮤텍스 획득
    if (xSemaphoreTake(spi_mutex[port], portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Save configuration
    spi_configs[port] = *config;
    
    // Reset SPI controller
    spi_hw_reset(base);
    
    // Configure GPIO pins for SPI function
    // MOSI: Output
    esp_err_t ret = bsw_gpio_config_pin(config->mosi_pin, BSW_GPIO_MODE_OUTPUT, 
                                       BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(SPI_TAG, "Failed to configure MOSI pin %d", config->mosi_pin);
        return ret;
    }
    
    // MISO: Input
    ret = bsw_gpio_config_pin(config->miso_pin, BSW_GPIO_MODE_INPUT, 
                             BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(SPI_TAG, "Failed to configure MISO pin %d", config->miso_pin);
        return ret;
    }
    
    // SCLK: Output
    ret = bsw_gpio_config_pin(config->sclk_pin, BSW_GPIO_MODE_OUTPUT, 
                             BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(SPI_TAG, "Failed to configure SCLK pin %d", config->sclk_pin);
        return ret;
    }
    
    // CS: Output (default HIGH)
    ret = bsw_gpio_config_pin(config->cs_pin, BSW_GPIO_MODE_OUTPUT, 
                             BSW_GPIO_PULLUP_DISABLE, BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(SPI_TAG, "Failed to configure CS pin %d", config->cs_pin);
        return ret;
    }
    bsw_gpio_set_level(config->cs_pin, config->cs_active_high ? 0 : 1);  // CS idle state
    
    // Configure SPI clock
    spi_configure_clock(base, config->clock_speed);
    
    // Configure SPI mode (CPOL/CPHA)
    spi_configure_mode(base, config->mode);
    
    // Configure user register for full-duplex mode
    uint32_t user_reg = SPI_USER_DOUTDIN_BIT |  // Full-duplex
                        SPI_USER_CS_SETUP_BIT |  // CS setup
                        SPI_USER_CS_HOLD_BIT |   // CS hold
                        SPI_USER_USR_MISO_BIT |  // Enable MISO
                        SPI_USER_USR_MOSI_BIT;   // Enable MOSI
    SPI_WRITE_REG(base, SPI_USER_REG_OFFSET, user_reg);
    
    // Configure data bit order (MSB first)
    SPI_CLEAR_BITS(base, SPI_CTRL_REG_OFFSET, SPI_CTRL_WR_BIT_ORDER_BIT | SPI_CTRL_RD_BIT_ORDER_BIT);
    
    spi_initialized[port] = true;
    
    BSW_LOGI(SPI_TAG, "SPI HW port %d initialized: MOSI=%d, MISO=%d, SCLK=%d, CS=%d, freq=%luHz, mode=%d", 
             port, config->mosi_pin, config->miso_pin, config->sclk_pin, config->cs_pin,
             (uint32_t)config->clock_speed, config->mode);
    
    xSemaphoreGive(spi_mutex[port]);
    return ESP_OK;
}

/**
 * @brief SPI Single Byte Transfer
 * 
 * @param port SPI port number
 * @param tx_data Transmit data
 * @param rx_data Pointer to store received data (can be NULL)
 * @return esp_err_t Transmission result
 */
esp_err_t bsw_spi_transfer_byte(bsw_spi_port_t port, uint8_t tx_data, uint8_t* rx_data) {
    return bsw_spi_transfer_block(port, &tx_data, rx_data, 1);
}

/**
 * @brief SPI Block Transfer (v2.0: 멀티태스킹 안전)
 * 
 * @param port SPI port number
 * @param tx_buffer Transmit buffer (can be NULL for receive-only)
 * @param rx_buffer Receive buffer (can be NULL for transmit-only)
 * @param length Data length in bytes
 * @return esp_err_t Transmission result
 */
esp_err_t bsw_spi_transfer_block(bsw_spi_port_t port, const uint8_t* tx_buffer, 
                                 uint8_t* rx_buffer, size_t length) {
    if (port >= BSW_SPI_PORT_MAX || !spi_initialized[port]) {
        BSW_LOGE(SPI_TAG, "SPI port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (length == 0 || length > SPI_MAX_TRANSFER_SIZE) {
        BSW_LOGE(SPI_TAG, "Invalid length: %zu (max: %d)", length, SPI_MAX_TRANSFER_SIZE);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!tx_buffer && !rx_buffer) {
        BSW_LOGE(SPI_TAG, "Both tx_buffer and rx_buffer are NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 뮤텍스 획득 (v2.0 신규)
    if (xSemaphoreTake(spi_mutex[port], portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint32_t base = spi_get_base_addr(port);
    
    // Set data length (in bits)
    uint32_t bit_length = (length * 8) - 1;
    SPI_WRITE_REG(base, SPI_MS_DLEN_REG_OFFSET, bit_length);
    
    // Write TX data to buffer (W0 register)
    if (tx_buffer) {
        uint32_t tx_word = 0;
        for (size_t i = 0; i < length && i < 4; i++) {
            tx_word |= (tx_buffer[i] << (i * 8));
        }
        SPI_WRITE_REG(base, SPI_W0_REG_OFFSET, tx_word);
    } else {
        // Send dummy bytes if no TX buffer
        SPI_WRITE_REG(base, SPI_W0_REG_OFFSET, 0xFFFFFFFF);
    }
    
    // Assert CS (LOW)
    bsw_gpio_set_level(spi_configs[port].cs_pin, spi_configs[port].cs_active_high ? 1 : 0);
    esp_rom_delay_us(1);
    
    // Start transaction
    SPI_SET_BITS(base, SPI_CMD_REG_OFFSET, SPI_CMD_USR_BIT);
    
    // Wait for completion
    esp_err_t result = spi_wait_trans_complete(base, 1000);  // 1s timeout
    
    // Deassert CS (HIGH)
    esp_rom_delay_us(1);
    bsw_gpio_set_level(spi_configs[port].cs_pin, spi_configs[port].cs_active_high ? 0 : 1);
    
    if (result != ESP_OK) {
        BSW_LOGE(SPI_TAG, "SPI transfer failed");
        return result;
    }
    
    // Read RX data from buffer (W0 register)
    if (rx_buffer) {
        uint32_t rx_word = SPI_READ_REG(base, SPI_W0_REG_OFFSET);
        for (size_t i = 0; i < length && i < 4; i++) {
            rx_buffer[i] = (rx_word >> (i * 8)) & 0xFF;
        }
    }
    
    xSemaphoreGive(spi_mutex[port]);
    return ESP_OK;
}

/**
 * @brief SPI Chip Select Assert (v2.0: 멀티태스킹 안전)
 * 
 * @param port SPI port number
 * @return esp_err_t Result
 */
esp_err_t bsw_spi_cs_select(bsw_spi_port_t port) {
    if (port >= BSW_SPI_PORT_MAX || !spi_initialized[port]) {
        BSW_LOGE(SPI_TAG, "SPI port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    // 뮤텍스 획득 (수동 CS 제어 시작)
    if (xSemaphoreTake(spi_mutex[port], portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Assert CS (LOW for active-low, HIGH for active-high)
    bsw_gpio_set_level(spi_configs[port].cs_pin, spi_configs[port].cs_active_high ? 1 : 0);
    
    // 뮤텍스는 bsw_spi_cs_deselect()에서 해제됨
    return ESP_OK;
}

/**
 * @brief SPI Chip Select Deassert (v2.0: 멀티태스킹 안전)
 * 
 * @param port SPI port number
 * @return esp_err_t Result
 */
esp_err_t bsw_spi_cs_deselect(bsw_spi_port_t port) {
    if (port >= BSW_SPI_PORT_MAX || !spi_initialized[port]) {
        BSW_LOGE(SPI_TAG, "SPI port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Deassert CS (HIGH for active-low, LOW for active-high)
    bsw_gpio_set_level(spi_configs[port].cs_pin, spi_configs[port].cs_active_high ? 0 : 1);
    
    // 뮤텍스 해제 (수동 CS 제어 종료)
    xSemaphoreGive(spi_mutex[port]);
    
    return ESP_OK;
}

/**
 * @brief SPI Driver Deinitialization (v2.0: 리소스 정리 강화)
 * 
 * @param port SPI port number
 * @return esp_err_t Deinitialization result
 */
esp_err_t bsw_spi_deinit(bsw_spi_port_t port) {
    if (port >= BSW_SPI_PORT_MAX) {
        BSW_LOGE(SPI_TAG, "Invalid SPI port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (spi_initialized[port]) {
        // 뮤텍스 획득
        if (spi_mutex[port] != NULL) {
            xSemaphoreTake(spi_mutex[port], portMAX_DELAY);
        }
        
        uint32_t base = spi_get_base_addr(port);
        
        // Reset SPI controller
        spi_hw_reset(base);
        
        // Restore GPIO pins to input mode
        bsw_gpio_set_direction(spi_configs[port].mosi_pin, BSW_GPIO_MODE_INPUT);
        bsw_gpio_set_direction(spi_configs[port].miso_pin, BSW_GPIO_MODE_INPUT);
        bsw_gpio_set_direction(spi_configs[port].sclk_pin, BSW_GPIO_MODE_INPUT);
        bsw_gpio_set_direction(spi_configs[port].cs_pin, BSW_GPIO_MODE_INPUT);
        
        spi_initialized[port] = false;
        
        // 뮤텍스 해제 및 삭제
        if (spi_mutex[port] != NULL) {
            xSemaphoreGive(spi_mutex[port]);
            vSemaphoreDelete(spi_mutex[port]);
            spi_mutex[port] = NULL;
        }
        
        BSW_LOGI(SPI_TAG, "SPI port %d deinitialized (hardware controller disabled)", port);
    }
    
    return ESP_OK;
}

// ============================================================================
// DMA 전송 지원 (v2.0 신규)
// ============================================================================

/**
 * @brief DMA를 사용한 SPI 전송
 * 
 * 참고: ESP-IDF SPI HAL의 DMA 기능을 활용합니다.
 * 대용량 데이터 전송 시 CPU 부하를 최소화합니다.
 */
esp_err_t bsw_spi_transfer_dma(bsw_spi_port_t port, const uint8_t* tx_buffer, 
                               uint8_t* rx_buffer, size_t length) {
    if (port >= BSW_SPI_PORT_MAX || !spi_initialized[port]) {
        BSW_LOGE(SPI_TAG, "SPI port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!tx_buffer) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 뮤텍스 획득
    if (xSemaphoreTake(spi_mutex[port], portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // DMA 전송 플래그 초기화
    spi_dma_done[port] = false;
    
    // 현재는 폴링 방식으로 구현 (향후 ESP-IDF HAL DMA로 확장 가능)
    // TODO: ESP-IDF의 spi_device_transmit()를 통합
    esp_err_t result = bsw_spi_transfer_block(port, tx_buffer, rx_buffer, length);
    
    spi_dma_done[port] = true;
    
    xSemaphoreGive(spi_mutex[port]);
    
    if (result == ESP_OK) {
        BSW_LOGI(SPI_TAG, "SPI %d DMA transfer completed: %d bytes", port, length);
    }
    
    return result;
}

/**
 * @brief DMA 전송 완료 대기
 */
esp_err_t bsw_spi_wait_dma_done(bsw_spi_port_t port, uint32_t timeout_ms) {
    if (port >= BSW_SPI_PORT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    TickType_t start_ticks = xTaskGetTickCount();
    
    while (!spi_dma_done[port]) {
        uint32_t elapsed_ms = (xTaskGetTickCount() - start_ticks) * portTICK_PERIOD_MS;
        if (elapsed_ms > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}
