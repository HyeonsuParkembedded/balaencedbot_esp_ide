/**
 * @file i2c_driver.h
 * @brief ESP32-C6 Hardware I2C Controller Driver (Wrapper for ESP-IDF Driver)
 * 
 * ESP32-C6 I2C driver wrapping the standard ESP-IDF driver/i2c_master.h.
 * Used for communication with MPU6050 IMU sensor.
 * 
 * @author Hyeonsu Park, Suyong Kim (Modified by Copilot)
 * @date 2025-10-08
 * @version 7.0 (Standard ESP-IDF Driver)
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "gpio_driver.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief BSW I2C Port Type Definition
 * 
 * @warning ESP32-C6 ONLY supports I2C0!
 */
typedef enum {
    BSW_I2C_PORT_0 = 0,         ///< I2C Port 0 - ✅ Available on ESP32-C6
    BSW_I2C_PORT_1,             ///< I2C Port 1 - ❌ NOT available on ESP32-C6
    BSW_I2C_PORT_MAX
} bsw_i2c_port_t;

/**
 * @brief I2C Clock Speed Options
 */
typedef enum {
    BSW_I2C_FREQ_100K = 100000,     ///< Standard Mode: 100kHz
    BSW_I2C_FREQ_400K = 400000,     ///< Fast Mode: 400kHz
    BSW_I2C_FREQ_1M   = 1000000     ///< Fast Mode Plus: 1MHz
} bsw_i2c_clock_speed_t;

/**
 * @brief I2C Hardware Configuration Structure
 */
typedef struct {
    bsw_gpio_num_t sda_pin;             ///< SDA Pin Number
    bsw_gpio_num_t scl_pin;             ///< SCL Pin Number
    bsw_i2c_clock_speed_t clock_speed;  ///< Clock Speed
    bool use_pullup;                    ///< Internal Pullup Enable
    uint32_t timeout_ms;                ///< Transaction Timeout (ms)
} i2c_hw_config_t;

/**
 * @brief I2C Default Configuration Constants
 */
#define I2C_DEFAULT_CLOCK_SPEED     BSW_I2C_FREQ_100K  ///< Default: 100kHz
#define I2C_FAST_CLOCK_SPEED        BSW_I2C_FREQ_400K  ///< Fast: 400kHz
#define I2C_DEFAULT_TIMEOUT_MS      1000               ///< Default Timeout: 1s

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup I2C_DRIVER ESP32-C6 Hardware I2C Controller Driver API
 * @{
 */

/**
 * @brief I2C Hardware Controller Initialization
 * 
 * Initializes the I2C master bus using ESP-IDF driver.
 * 
 * @param port I2C port number (0 only)
 * @param sda_pin SDA GPIO pin number
 * @param scl_pin SCL GPIO pin number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin);

/**
 * @brief I2C Hardware Controller Initialization (Custom Configuration)
 * 
 * @param port I2C port number
 * @param config I2C hardware configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_init_config(bsw_i2c_port_t port, const i2c_hw_config_t* config);

/**
 * @brief I2C Device Register Write
 * 
 * Writes a single byte to a device register.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address
 * @param value Data value to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

/**
 * @brief I2C Device Register Read
 * 
 * Reads multiple bytes from a device register.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);

/**
 * @brief I2C Raw Data Write
 * 
 * Writes raw data to the I2C bus.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param data Data buffer to write
 * @param len Data length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_write_raw(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len);

/**
 * @brief I2C Raw Data Read
 * 
 * Reads raw data from the I2C bus.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit)
 * @param data Buffer to store read data
 * @param len Data length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_read_raw(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len);

/**
 * @brief I2C Driver Deinitialization
 * 
 * Frees I2C resources.
 * 
 * @param port I2C port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_deinit(bsw_i2c_port_t port);

/**
 * @brief I2C Bus Recovery Function
 * 
 * @param port I2C port number
 * @return esp_err_t ESP_OK
 */
esp_err_t i2c_bus_recovery(bsw_i2c_port_t port);

/** @} */ // I2C_DRIVER

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H
