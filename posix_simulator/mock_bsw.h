/**
 * @file mock_bsw.h
 * @brief BSW (Basic Software) Mock Layer for POSIX Simulator
 * 
 * BalanceBot의 BSW 레이어를 POSIX 환경에서 시뮬레이션하기 위한 Mock 인터페이스입니다.
 * 실제 BSW API와 동일한 인터페이스를 제공하면서 하드웨어 동작을 시뮬레이션합니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#ifndef MOCK_BSW_H
#define MOCK_BSW_H

#include "mock_esp.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// BSW Error Codes (same as original)
// ============================================================================
typedef enum {
    BSW_OK = 0,                  ///< 성공
    BSW_FAIL = -1,              ///< 일반적인 실패
    BSW_ERR_INVALID_ARG = -2,   ///< 잘못된 인수
    BSW_ERR_NO_MEM = -3,        ///< 메모리 부족
    BSW_ERR_INVALID_STATE = -4, ///< 잘못된 상태
    BSW_ERR_TIMEOUT = -5        ///< 타임아웃
} bsw_err_t;

// ============================================================================
// BSW GPIO Mock
// ============================================================================
typedef uint8_t bsw_gpio_num_t;

typedef enum {
    BSW_GPIO_MODE_DISABLE = 0,
    BSW_GPIO_MODE_INPUT = 1,
    BSW_GPIO_MODE_OUTPUT = 2,
    BSW_GPIO_MODE_OUTPUT_OD = 3,
    BSW_GPIO_MODE_INPUT_OUTPUT_OD = 4,
    BSW_GPIO_MODE_INPUT_OUTPUT = 5
} bsw_gpio_mode_t;

bsw_err_t bsw_gpio_init(void);
bsw_err_t bsw_gpio_set_mode(bsw_gpio_num_t pin, bsw_gpio_mode_t mode);
bsw_err_t bsw_gpio_set_level(bsw_gpio_num_t pin, uint32_t level);
uint32_t bsw_gpio_get_level(bsw_gpio_num_t pin);

// ============================================================================
// BSW I2C Mock  
// ============================================================================
typedef enum {
    BSW_I2C_NUM_0 = 0,
    BSW_I2C_NUM_MAX
} bsw_i2c_num_t;

typedef struct {
    bsw_gpio_num_t sda_pin;
    bsw_gpio_num_t scl_pin;
    uint32_t clk_speed;
} bsw_i2c_config_t;

bsw_err_t bsw_i2c_init(bsw_i2c_num_t i2c_num, const bsw_i2c_config_t* config);
bsw_err_t bsw_i2c_write(bsw_i2c_num_t i2c_num, uint8_t addr, const uint8_t* data, size_t size);
bsw_err_t bsw_i2c_read(bsw_i2c_num_t i2c_num, uint8_t addr, uint8_t* data, size_t size);
bsw_err_t bsw_i2c_write_read(bsw_i2c_num_t i2c_num, uint8_t addr, 
                             const uint8_t* write_data, size_t write_size,
                             uint8_t* read_data, size_t read_size);

// ============================================================================
// BSW PWM Mock
// ============================================================================
typedef enum {
    BSW_PWM_CHANNEL_0 = 0,
    BSW_PWM_CHANNEL_1,
    BSW_PWM_CHANNEL_2,
    BSW_PWM_CHANNEL_3,
    BSW_PWM_CHANNEL_MAX
} bsw_pwm_channel_t;

typedef struct {
    bsw_gpio_num_t pin;
    uint32_t freq_hz;
    uint8_t duty_resolution;
} bsw_pwm_config_t;

bsw_err_t bsw_pwm_init(void);
bsw_err_t bsw_pwm_config_channel(bsw_pwm_channel_t channel, const bsw_pwm_config_t* config);
bsw_err_t bsw_pwm_set_duty(bsw_pwm_channel_t channel, uint32_t duty);
bsw_err_t bsw_pwm_start(bsw_pwm_channel_t channel);
bsw_err_t bsw_pwm_stop(bsw_pwm_channel_t channel);

// ============================================================================
// BSW ADC Mock
// ============================================================================
typedef enum {
    BSW_ADC_UNIT_1 = 0,
    BSW_ADC_UNIT_2
} bsw_adc_unit_t;

typedef enum {
    BSW_ADC_CHANNEL_0 = 0,
    BSW_ADC_CHANNEL_1,
    BSW_ADC_CHANNEL_2,
    BSW_ADC_CHANNEL_3,
    BSW_ADC_CHANNEL_MAX
} bsw_adc_channel_t;

bsw_err_t bsw_adc_init(void);
bsw_err_t bsw_adc_config_channel(bsw_adc_unit_t unit, bsw_adc_channel_t channel);
uint32_t bsw_adc_read(bsw_adc_unit_t unit, bsw_adc_channel_t channel);

// ============================================================================
// BSW UART Mock
// ============================================================================
typedef enum {
    BSW_UART_NUM_0 = 0,
    BSW_UART_NUM_1,
    BSW_UART_NUM_MAX
} bsw_uart_num_t;

typedef struct {
    int baudrate;
    bsw_gpio_num_t tx_pin;
    bsw_gpio_num_t rx_pin;
} bsw_uart_config_t;

bsw_err_t bsw_uart_init(bsw_uart_num_t uart_num, const bsw_uart_config_t* config);
bsw_err_t bsw_uart_write(bsw_uart_num_t uart_num, const uint8_t* data, size_t size);
size_t bsw_uart_read(bsw_uart_num_t uart_num, uint8_t* buffer, size_t size, uint32_t timeout_ms);

// ============================================================================
// BSW System Services Mock
// ============================================================================
typedef enum {
    BSW_LOG_ERROR = 0,
    BSW_LOG_WARN,
    BSW_LOG_INFO,
    BSW_LOG_DEBUG
} bsw_log_level_t;

void bsw_log_print(bsw_log_level_t level, const char* tag, const char* format, ...);
uint32_t bsw_get_tick_ms(void);
uint32_t bsw_get_tick_us(void);
void bsw_delay_ms(uint32_t ms);
void bsw_delay_us(uint32_t us);

// BSW Logging Macros (same interface as original)
#define BSW_LOGE(tag, format, ...) bsw_log_print(BSW_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define BSW_LOGW(tag, format, ...) bsw_log_print(BSW_LOG_WARN, tag, format, ##__VA_ARGS__)
#define BSW_LOGI(tag, format, ...) bsw_log_print(BSW_LOG_INFO, tag, format, ##__VA_ARGS__)
#define BSW_LOGD(tag, format, ...) bsw_log_print(BSW_LOG_DEBUG, tag, format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // MOCK_BSW_H