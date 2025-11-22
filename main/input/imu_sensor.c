/**
 * @file imu_sensor.c
 * @brief MPU6050/MPU6500 IMU Sensor Driver Implementation
 * 
 * Implements low-level control and data processing for MPU6050/MPU6500 6-axis IMU.
 * Reads accelerometer/gyroscope data via I2C and calculates pitch/roll angles.
 * 
 * @author Hyeonsu Park, Suyong Kim (Modified by Copilot)
 * @date 2025-10-08
 * @version 1.1 (Added MPU6500 Support)
 */

#include "imu_sensor.h"
#include "../bsw/i2c_driver.h"
#include "../bsw/system_services.h"
#include <math.h>

static const char* IMU_TAG = "IMU_SENSOR"; ///< Logging Tag

/**
 * @defgroup MPU6050_REGISTERS MPU6050/MPU6500 Register Addresses
 * @brief Internal register addresses for MPU6050/MPU6500 sensors
 * @{
 */
#define MPU6050_ADDR            0x68  ///< I2C Device Address (AD0 Low)
#define MPU6050_WHO_AM_I        0x75  ///< Device ID Register
#define MPU6050_PWR_MGMT_1      0x6B  ///< Power Management 1
#define MPU6050_GYRO_CONFIG     0x1B  ///< Gyroscope Configuration
#define MPU6050_ACCEL_CONFIG    0x1C  ///< Accelerometer Configuration
#define MPU6050_ACCEL_XOUT_H    0x3B  ///< Accelerometer X-Axis High Byte
#define MPU6050_GYRO_XOUT_H     0x43  ///< Gyroscope X-Axis High Byte
#define MPU6050_CONFIG_REG      0x1A  ///< Configuration Register (DLPF)
#define MPU6500_ACCEL_CONFIG_2  0x1D  ///< MPU6500 Specific: Accel DLPF Config
/** @} */

/**
 * @brief Initialize MPU6050/MPU6500 IMU sensor and I2C communication
 * 
 * Sets up I2C communication and initializes the sensor.
 * Verifies sensor connection by checking WHO_AM_I register (Supports 0x68 and 0x70).
 * Configures power management, gyroscope, and accelerometer settings.
 * 
 * Initialization Steps:
 * 1. Initialize I2C driver
 * 2. Check WHO_AM_I register (0x68 for MPU6050, 0x70 for MPU6500)
 * 3. Wake up sensor (PWR_MGMT_1)
 * 4. Configure DLPF (44Hz bandwidth)
 * 5. Configure Gyroscope range (±500°/s)
 * 6. Configure Accelerometer range (±2g)
 * 
 * @param sensor IMU sensor structure pointer
 * @param port I2C port number
 * @param sda_pin I2C SDA pin number
 * @param scl_pin I2C SCL pin number
 * @return ESP_OK on success, ESP_FAIL on connection failure or I2C error
 */
esp_err_t imu_sensor_init(imu_sensor_t* sensor, bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin) {
    sensor->i2c_port = port;
    sensor->data.accel_x = sensor->data.accel_y = sensor->data.accel_z = 0.0f;
    sensor->data.gyro_x = sensor->data.gyro_y = sensor->data.gyro_z = 0.0f;
    sensor->data.pitch = sensor->data.roll = 0.0f;
    sensor->data.pitch_offset = 0.0f;
    sensor->data.initialized = false;

    // Initialize I2C driver
    esp_err_t ret = i2c_driver_init(port, sda_pin, scl_pin);
    if (ret != ESP_OK) {
        return ret;
    }

    // Check WHO_AM_I register
    uint8_t who_am_i;
    ret = i2c_read_register(port, MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        BSW_LOGE(IMU_TAG, "Failed to read WHO_AM_I register (Error: %d). Check wiring/pull-ups.", ret);
        return ret;
    }

    // Support both MPU6050 (0x68) and MPU6500 (0x70) IDs
    if (who_am_i != 0x68 && who_am_i != 0x70) {
        BSW_LOGE(IMU_TAG, "Unknown IMU ID: 0x%02X (Expected 0x68[MPU6050] or 0x70[MPU6500])", who_am_i);
        return ESP_FAIL;
    }
    
    bool is_mpu6500 = (who_am_i == 0x70);
    BSW_LOGI(IMU_TAG, "IMU Detected. ID: 0x%02X (%s)", who_am_i, is_mpu6500 ? "MPU6500" : "MPU6050");

    // Wake up MPU6050/6500
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // DLPF Config (44Hz bandwidth for Gyro/Temp)
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_CONFIG_REG, 0x03);
    if (ret != ESP_OK) {
        return ret;
    }

    // MPU6500 Specific: Configure Accel DLPF (41Hz)
    if (is_mpu6500) {
        // ACCEL_CONFIG_2 (0x1D): accel_fchoice_b=0 (bit 3), A_DLPFCFG=3 (bits 2:0) -> 0x03
        ret = i2c_write_register(port, MPU6050_ADDR, MPU6500_ACCEL_CONFIG_2, 0x03);
        if (ret != ESP_OK) {
            BSW_LOGW(IMU_TAG, "Failed to set MPU6500 Accel DLPF");
        } else {
            BSW_LOGI(IMU_TAG, "MPU6500 Accel DLPF configured (41Hz)");
        }
    }

    // Configure gyroscope (±500 degrees/s)
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x08);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure accelerometer (±2g)
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    sensor->data.initialized = true;

    BSW_LOGI(IMU_TAG, "IMU sensor initialized successfully");

    return ESP_OK;
}

/**
 * @brief Update IMU sensor data
 * 
 * Reads 14 bytes of continuous data from MPU6050/6500.
 * Extracts accelerometer and gyroscope values.
 * Converts raw data to physical units and calculates pitch/roll angles.
 * 
 * Data Processing:
 * 1. Read 14 bytes via I2C (Accel 6B + Temp 2B + Gyro 6B)
 * 2. Convert 16-bit Big-Endian data to integers
 * 3. Apply scaling factors (Accel: /16384, Gyro: /65.5)
 * 4. Calculate Pitch/Roll from accelerometer data (using atan2)
 * 
 * @param sensor IMU sensor structure pointer
 * @return ESP_OK on success, ESP_FAIL if not initialized or I2C error
 */
esp_err_t imu_sensor_update(imu_sensor_t* sensor) {
    if (!sensor->data.initialized) {
        return ESP_FAIL;
    }

    uint8_t raw_data[14];
    esp_err_t ret = i2c_read_register(sensor->i2c_port, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse accelerometer data
    int16_t accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // Parse gyroscope data
    int16_t gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    // Convert to physical units
    sensor->data.accel_x = accel_x / 16384.0f;  // ±2g range
    sensor->data.accel_y = accel_y / 16384.0f;
    sensor->data.accel_z = accel_z / 16384.0f;

    sensor->data.gyro_x = gyro_x / 65.5f;      // ±500°/s range
    sensor->data.gyro_y = gyro_y / 65.5f;
    sensor->data.gyro_z = gyro_z / 65.5f;

    // Calculate pitch and roll from accelerometer
    sensor->data.pitch = atan2(-sensor->data.accel_x, sqrt(sensor->data.accel_y * sensor->data.accel_y + sensor->data.accel_z * sensor->data.accel_z)) * 180.0f / M_PI;
    sensor->data.pitch -= sensor->data.pitch_offset; // Apply offset
    sensor->data.roll = atan2(sensor->data.accel_y, sensor->data.accel_z) * 180.0f / M_PI;

    return ESP_OK;
}

/**
 * @brief Get current Pitch angle
 * @param sensor IMU sensor structure pointer
 * @return Pitch angle (degrees, -180° ~ +180°)
 */
float imu_sensor_get_pitch(imu_sensor_t* sensor) {
    return sensor->data.pitch;
}

/**
 * @brief Get current Roll angle
 * @param sensor IMU sensor structure pointer
 * @return Roll angle (degrees, -180° ~ +180°)
 */
float imu_sensor_get_roll(imu_sensor_t* sensor) {
    return sensor->data.roll;
}

/**
 * @brief Get X-axis Gyroscope angular velocity
 * @param sensor IMU sensor structure pointer
 * @return X-axis angular velocity (°/s)
 */
float imu_sensor_get_gyro_x(imu_sensor_t* sensor) {
    return sensor->data.gyro_x;
}

/**
 * @brief Get Y-axis Gyroscope angular velocity
 * @param sensor IMU sensor structure pointer
 * @return Y-axis angular velocity (°/s)
 */
float imu_sensor_get_gyro_y(imu_sensor_t* sensor) {
    return sensor->data.gyro_y;
}

/**
 * @brief Get Z-axis Gyroscope angular velocity
 * @param sensor IMU sensor structure pointer
 * @return Z-axis angular velocity (°/s)
 */
float imu_sensor_get_gyro_z(imu_sensor_t* sensor) {
    return sensor->data.gyro_z;
}

/**
 * @brief Get X-axis Acceleration
 * @param sensor IMU sensor structure pointer
 * @return X-axis acceleration (g)
 */
float imu_sensor_get_accel_x(imu_sensor_t* sensor) {
    return sensor->data.accel_x;
}

/**
 * @brief Get Y-axis Acceleration
 * @param sensor IMU sensor structure pointer
 * @return Y-axis acceleration (g)
 */
float imu_sensor_get_accel_y(imu_sensor_t* sensor) {
    return sensor->data.accel_y;
}

/**
 * @brief Get Z-axis Acceleration
 * @param sensor IMU sensor structure pointer
 * @return Z-axis acceleration (g)
 */
float imu_sensor_get_accel_z(imu_sensor_t* sensor) {
    return sensor->data.accel_z;
}

/**
 * @brief Check if IMU sensor is initialized
 * @param sensor IMU sensor structure pointer
 * @return true if initialized, false otherwise
 */
bool imu_sensor_is_initialized(imu_sensor_t* sensor) {
    return sensor->data.initialized;
}

/**
 * @brief Set Pitch angle offset
 * @param sensor IMU sensor structure pointer
 * @param offset Offset value (degrees)
 */
void imu_sensor_set_pitch_offset(imu_sensor_t* sensor, float offset) {
    sensor->data.pitch_offset = offset;
}
