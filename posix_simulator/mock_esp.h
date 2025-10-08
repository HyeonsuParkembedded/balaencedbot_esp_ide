/**
 * @file mock_esp.h
 * @brief ESP32 API Mock for POSIX Simulator
 * 
 * ESP32-IDF의 핵심 API들을 POSIX 환경에서 시뮬레이션하기 위한 Mock 헤더입니다.
 * BalanceBot의 제어 로직을 하드웨어 없이 테스트할 수 있게 해줍니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#ifndef MOCK_ESP_H
#define MOCK_ESP_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// ESP Error Codes Mock
// ============================================================================
typedef int esp_err_t;
#define ESP_OK          0       ///< esp_err_t value indicating success
#define ESP_FAIL        -1      ///< Generic esp_err_t code indicating failure
#define ESP_ERR_NO_MEM  0x101   ///< Out of memory
#define ESP_ERR_INVALID_ARG 0x102 ///< Invalid argument
#define ESP_ERR_TIMEOUT 0x107   ///< Operation or resource request timed out

// ============================================================================
// ESP System Mock
// ============================================================================
#define esp_restart()       do { printf("[MOCK] ESP Restart called\n"); exit(0); } while(0)
#define esp_get_free_heap_size()  (1024*1024)  // Mock: 1MB free heap

// ============================================================================
// ESP Logging Mock
// ============================================================================
typedef enum {
    ESP_LOG_NONE,       ///< No log output
    ESP_LOG_ERROR,      ///< Critical errors, software module can not recover on its own
    ESP_LOG_WARN,       ///< Error conditions from which recovery measures have been taken
    ESP_LOG_INFO,       ///< Information messages which describe normal flow of events
    ESP_LOG_DEBUG,      ///< Extra information which is not necessary for normal use (values, pointers, sizes, etc).
    ESP_LOG_VERBOSE     ///< Bigger chunks of debugging information, or frequent messages which can potentially flood the output.
} esp_log_level_t;

// Mock ESP_LOG functions
#define ESP_LOGE(tag, format, ...) printf("[ERROR][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("[WARN ][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) printf("[INFO ][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) printf("[DEBUG][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) printf("[VERB ][%s] " format "\n", tag, ##__VA_ARGS__)

// ============================================================================
// NVS (Non-Volatile Storage) Mock
// ============================================================================
typedef void* nvs_handle_t;

esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);
esp_err_t nvs_open(const char* name, int open_mode, nvs_handle_t *out_handle);
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length);
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);
esp_err_t nvs_commit(nvs_handle_t handle);
void nvs_close(nvs_handle_t handle);

#define NVS_READWRITE 1

// ============================================================================
// GPIO Mock
// ============================================================================
typedef int gpio_num_t;
#define GPIO_NUM_NC (-1)

typedef enum {
    GPIO_MODE_DISABLE = 0,
    GPIO_MODE_INPUT = 1,
    GPIO_MODE_OUTPUT = 2,
    GPIO_MODE_OUTPUT_OD = 3,
    GPIO_MODE_INPUT_OUTPUT_OD = 4,
    GPIO_MODE_INPUT_OUTPUT = 5
} gpio_mode_t;

typedef enum {
    GPIO_PULLUP_DISABLE = 0,
    GPIO_PULLUP_ENABLE = 1
} gpio_pullup_t;

typedef enum {
    GPIO_PULLDOWN_DISABLE = 0,
    GPIO_PULLDOWN_ENABLE = 1
} gpio_pulldown_t;

typedef struct {
    uint64_t pin_bit_mask;
    gpio_mode_t mode;
    gpio_pullup_t pull_up_en;
    gpio_pulldown_t pull_down_en;
    // Add other fields as needed
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *pGPIOConfig);
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
int gpio_get_level(gpio_num_t gpio_num);

// ============================================================================
// I2C Mock
// ============================================================================
typedef int i2c_port_t;
#define I2C_NUM_0 0

typedef enum {
    I2C_MODE_SLAVE = 0,
    I2C_MODE_MASTER = 1
} i2c_mode_t;

typedef struct {
    i2c_mode_t mode;
    int sda_io_num;
    int scl_io_num;
    bool sda_pullup_en;
    bool scl_pullup_en;
    uint32_t master_clk_speed;
} i2c_config_t;

esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t* conf);
esp_err_t i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags);
esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address, const uint8_t* write_buffer, size_t write_size, int ticks_to_wait);
esp_err_t i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address, uint8_t* read_buffer, size_t read_size, int ticks_to_wait);

// ============================================================================
// Timer Mock
// ============================================================================
uint32_t esp_timer_get_time(void);  // Returns microseconds since boot

// ============================================================================
// Math utilities
// ============================================================================
static inline uint32_t mock_get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static inline uint32_t mock_get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

#ifdef __cplusplus
}
#endif

#endif // MOCK_ESP_H