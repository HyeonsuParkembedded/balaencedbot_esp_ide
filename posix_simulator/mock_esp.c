/**
 * @file mock_esp.c
 * @brief ESP32 API Mock Implementation for POSIX Simulator
 * 
 * ESP32-IDF API들의 Mock 구현입니다. 실제 하드웨어 동작을 시뮬레이션합니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#include "mock_esp.h"
#include <inttypes.h>
#include <pthread.h>

// ============================================================================
// Global Mock State
// ============================================================================
static pthread_mutex_t mock_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool nvs_initialized = false;
static uint32_t gpio_levels[64] = {0}; // Support GPIO 0-63
static bool i2c_initialized[2] = {false}; // I2C_NUM_0, I2C_NUM_1

// Mock IMU data for simulation
static struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    bool connected;
} mock_imu = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false};

// ============================================================================
// NVS Mock Implementation
// ============================================================================
esp_err_t nvs_flash_init(void) {
    pthread_mutex_lock(&mock_mutex);
    nvs_initialized = true;
    pthread_mutex_unlock(&mock_mutex);
    printf("[MOCK_NVS] Flash initialized\n");
    return ESP_OK;
}

esp_err_t nvs_flash_erase(void) {
    printf("[MOCK_NVS] Flash erased\n");
    return ESP_OK;
}

esp_err_t nvs_open(const char* name, int open_mode, nvs_handle_t *out_handle) {
    if (!nvs_initialized) return ESP_FAIL;
    *out_handle = (nvs_handle_t)0x12345678;  // Mock handle
    printf("[MOCK_NVS] Opened namespace: %s\n", name);
    return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length) {
    printf("[MOCK_NVS] Get blob: %s (returning default values)\n", key);
    // Return default/simulated values
    return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length) {
    printf("[MOCK_NVS] Set blob: %s (%zu bytes)\n", key, length);
    return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t handle) {
    printf("[MOCK_NVS] Commit\n");
    return ESP_OK;
}

void nvs_close(nvs_handle_t handle) {
    printf("[MOCK_NVS] Close\n");
}

// ============================================================================
// GPIO Mock Implementation  
// ============================================================================
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig) {
    printf("[MOCK_GPIO] Config pins: 0x%" PRIx64 ", mode: %d\n",
           pGPIOConfig->pin_bit_mask, pGPIOConfig->mode);
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    if (gpio_num >= 0 && gpio_num < 64) {
        pthread_mutex_lock(&mock_mutex);
        gpio_levels[gpio_num] = level;
        pthread_mutex_unlock(&mock_mutex);
        printf("[MOCK_GPIO] Set GPIO%d = %d\n", gpio_num, level);
    }
    return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio_num) {
    if (gpio_num >= 0 && gpio_num < 64) {
        pthread_mutex_lock(&mock_mutex);
        uint32_t level = gpio_levels[gpio_num];
        pthread_mutex_unlock(&mock_mutex);
        return level;
    }
    return 0;
}

// ============================================================================
// I2C Mock Implementation
// ============================================================================
esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t* conf) {
    printf("[MOCK_I2C] Config I2C%d: SDA=%d, SCL=%d, Speed=%d\n", 
           i2c_num, conf->sda_io_num, conf->scl_io_num, conf->master_clk_speed);
    return ESP_OK;
}

esp_err_t i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags) {
    if (i2c_num >= 0 && i2c_num < 2) {
        pthread_mutex_lock(&mock_mutex);
        i2c_initialized[i2c_num] = true;
        pthread_mutex_unlock(&mock_mutex);
        printf("[MOCK_I2C] Install I2C%d driver\n", i2c_num);
    }
    return ESP_OK;
}

esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address, 
                                     const uint8_t* write_buffer, size_t write_size, int ticks_to_wait) {
    printf("[MOCK_I2C] Write to device 0x%02X: %zu bytes\n", device_address, write_size);
    
    // Special handling for MPU6050 (typical address 0x68)
    if (device_address == 0x68) {
        if (write_size >= 1) {
            uint8_t reg = write_buffer[0];
            printf("[MOCK_MPU6050] Write to register 0x%02X\n", reg);
            if (reg == 0x6B && write_size >= 2) {  // PWR_MGMT_1 register
                printf("[MOCK_MPU6050] Power management configured\n");
                mock_imu.connected = true;
            }
        }
    }
    
    return ESP_OK;
}

esp_err_t i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address, 
                                     uint8_t* read_buffer, size_t read_size, int ticks_to_wait) {
    printf("[MOCK_I2C] Read from device 0x%02X: %zu bytes\n", device_address, read_size);
    
    // Special handling for MPU6050 IMU simulation
    if (device_address == 0x68 && mock_imu.connected) {
        // Simulate IMU data with some basic physics
        static uint32_t last_time = 0;
        uint32_t current_time = mock_get_time_ms();
        
        if (current_time - last_time > 20) {  // Update at 50Hz
            // Simple pendulum simulation for balance bot
            static float angle = 0.1f;  // Initial small angle (radians)
            static float angular_velocity = 0.0f;
            float dt = 0.02f; // 50Hz
            
            // Simple pendulum physics: theta'' = -(g/L) * sin(theta)
            float angular_acceleration = -9.81f * sin(angle);
            angular_velocity += angular_acceleration * dt;
            angle += angular_velocity * dt;
            
            // Add some damping
            angular_velocity *= 0.99f;
            
            // Convert to accelerometer and gyroscope readings
            mock_imu.accel_x = sin(angle) * 9.81f;           // Tilt acceleration
            mock_imu.accel_y = 0.0f;                         // No Y-axis tilt
            mock_imu.accel_z = cos(angle) * 9.81f;           // Gravity component
            mock_imu.gyro_x = angular_velocity * 180.0f / M_PI; // Convert to deg/s
            mock_imu.gyro_y = 0.0f;
            mock_imu.gyro_z = 0.0f;
            
            last_time = current_time;
        }
        
        // Fill buffer with mock IMU data (MPU6050 format)
        if (read_size >= 14) {  // Accel (6 bytes) + Temp (2 bytes) + Gyro (6 bytes)
            int16_t* data = (int16_t*)read_buffer;
            
            // Accelerometer data (16384 LSB/g for ±2g range)
            data[0] = __builtin_bswap16((int16_t)(mock_imu.accel_x * 16384.0f / 9.81f));
            data[1] = __builtin_bswap16((int16_t)(mock_imu.accel_y * 16384.0f / 9.81f));
            data[2] = __builtin_bswap16((int16_t)(mock_imu.accel_z * 16384.0f / 9.81f));
            
            // Temperature (not used, set to room temperature ~25°C)
            data[3] = __builtin_bswap16((int16_t)(25.0f * 340.0f + 12412.0f));
            
            // Gyroscope data (131 LSB/°/s for ±250°/s range)  
            data[4] = __builtin_bswap16((int16_t)(mock_imu.gyro_x * 131.0f));
            data[5] = __builtin_bswap16((int16_t)(mock_imu.gyro_y * 131.0f));
            data[6] = __builtin_bswap16((int16_t)(mock_imu.gyro_z * 131.0f));
            
            printf("[MOCK_MPU6050] IMU Data - Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)\n",
                   mock_imu.accel_x, mock_imu.accel_y, mock_imu.accel_z,
                   mock_imu.gyro_x, mock_imu.gyro_y, mock_imu.gyro_z);
        }
    } else {
        // Default: fill with zeros
        memset(read_buffer, 0, read_size);
    }
    
    return ESP_OK;
}

// ============================================================================
// Timer Mock Implementation
// ============================================================================
uint32_t esp_timer_get_time(void) {
    return mock_get_time_us();
}