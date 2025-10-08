/**
 * @file imu_sensor_mock.c
 * @brief IMU Sensor Mock for POSIX Simulator
 * 
 * MPU6050 IMU 센서의 Mock 구현입니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 */

#include "../mock_bsw.h"
#include <math.h>

// Mock IMU sensor structure
typedef struct {
    bool initialized;
    bool connected;
    bsw_i2c_num_t i2c_port;
    uint8_t device_addr;
} imu_sensor_t;

static imu_sensor_t mock_imu = {0};

// Mock functions that match the original API
bsw_err_t imu_sensor_init(imu_sensor_t* imu, bsw_i2c_num_t port, uint8_t addr) {
    if (!imu) return BSW_ERR_INVALID_ARG;
    
    mock_imu.i2c_port = port;
    mock_imu.device_addr = addr;
    mock_imu.initialized = true;
    mock_imu.connected = true;  // Always "connected" in simulation
    
    BSW_LOGI("MOCK_IMU", "IMU sensor initialized on I2C%d, addr=0x%02X", port, addr);
    return BSW_OK;
}

bsw_err_t imu_sensor_read_raw(imu_sensor_t* imu, int16_t* accel, int16_t* gyro) {
    if (!imu || !mock_imu.initialized) return BSW_ERR_INVALID_STATE;
    
    // Simulate IMU readings - this would normally come from physics simulation
    // For now, just return some reasonable values for a slightly tilted robot
    
    // Accelerometer (simulate ~5 degree tilt)
    accel[0] = (int16_t)(sin(5.0f * M_PI / 180.0f) * 16384);  // X-axis (tilt)
    accel[1] = 0;                                              // Y-axis
    accel[2] = (int16_t)(cos(5.0f * M_PI / 180.0f) * 16384);  // Z-axis (gravity)
    
    // Gyroscope (simulate small angular velocity)
    gyro[0] = 10;  // Small rotation rate
    gyro[1] = 0;
    gyro[2] = 0;
    
    return BSW_OK;
}

bool imu_sensor_is_connected(imu_sensor_t* imu) {
    return mock_imu.initialized && mock_imu.connected;
}