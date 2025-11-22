
import os

file_path = r"c:\Users\hyuns\Desktop\balaencedbot_esp_ide\main\bsw\i2c_driver.c"
new_content = r"""/**
 * @file i2c_driver.c
 * @brief ESP32-C6 Hardware I2C Controller Implementation using ESP-IDF Driver
 * 
 * Replaced custom register manipulation with standard ESP-IDF driver/i2c_master.h
 * to resolve timeout issues and ensure compatibility.
 * 
 * @author Hyeonsu Park, Suyong Kim (Modified by Copilot)
 * @date 2025-10-08
 * @version 7.0 (Standard ESP-IDF Driver)
 */

#include "i2c_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char* I2C_TAG = "HW_I2C";

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;
static bool i2c_initialized = false;

// Keep track of current device address to support multiple devices if needed
// For now, we assume single device or re-add device if address changes
static uint8_t current_device_addr = 0;

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

esp_err_t i2c_driver_init_config(bsw_i2c_port_t port, const i2c_hw_config_t* config) {
    if (port != BSW_I2C_PORT_0) {
        ESP_LOGE(I2C_TAG, "Only I2C0 is supported on ESP32-C6");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (i2c_initialized) {
        ESP_LOGW(I2C_TAG, "I2C already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1, // Auto-detect
        .scl_io_num = config->scl_pin,
        .sda_io_num = config->sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = config->use_pullup,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    
    i2c_initialized = true;
    ESP_LOGI(I2C_TAG, "I2C initialized successfully");
    return ESP_OK;
}

static esp_err_t ensure_device_handle(uint8_t device_addr) {
    if (!i2c_initialized || bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (dev_handle != NULL && current_device_addr == device_addr) {
        return ESP_OK;
    }

    if (dev_handle != NULL) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = 100000, // Default to 100kHz, can be parameterized if needed
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    current_device_addr = device_addr;
    return ESP_OK;
}

esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
    if (port != BSW_I2C_PORT_0) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = ensure_device_handle(device_addr);
    if (ret != ESP_OK) return ret;

    uint8_t data[2] = {reg_addr, value};
    return i2c_master_transmit(dev_handle, data, sizeof(data), -1);
}

esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (port != BSW_I2C_PORT_0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ensure_device_handle(device_addr);
    if (ret != ESP_OK) return ret;

    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, -1);
}

esp_err_t i2c_write_raw(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len) {
    if (port != BSW_I2C_PORT_0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ensure_device_handle(device_addr);
    if (ret != ESP_OK) return ret;

    return i2c_master_transmit(dev_handle, data, len, -1);
}

esp_err_t i2c_read_raw(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len) {
    if (port != BSW_I2C_PORT_0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ensure_device_handle(device_addr);
    if (ret != ESP_OK) return ret;

    return i2c_master_receive(dev_handle, data, len, -1);
}

esp_err_t i2c_driver_deinit(bsw_i2c_port_t port) {
    if (port != BSW_I2C_PORT_0) return ESP_ERR_INVALID_ARG;

    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }
    if (bus_handle) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
    }
    i2c_initialized = false;
    return ESP_OK;
}

esp_err_t i2c_bus_recovery(bsw_i2c_port_t port) {
    // Not implemented for standard driver yet, or use i2c_master_bus_reset if available
    return ESP_OK;
}
"""

with open(file_path, "w", encoding="utf-8") as f:
    f.write(new_content)

print("Successfully updated i2c_driver.c")
