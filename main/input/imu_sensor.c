/**
 * @file imu_sensor.c
 * @brief MPU6050 IMU 센서 드라이버 구현 파일
 * 
 * MPU6050 6축 관성 측정 장치의 저수준 제어 및 데이터 처리를 구현합니다.
 * I2C 통신을 통해 가속도계/자이로스코프 데이터를 읽어와 피치/롤 각도를 계산합니다.
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "imu_sensor.h"
#include "../bsw/i2c_driver.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#endif
#include <math.h>

#ifndef NATIVE_BUILD
static const char* IMU_TAG = "IMU_SENSOR"; ///< ESP-IDF 로깅 태그
#else
#define IMU_TAG "IMU_SENSOR" ///< 네이티브 빌드용 로깅 태그
#endif

/**
 * @defgroup MPU6050_REGISTERS MPU6050 레지스터 주소
 * @brief MPU6050 센서의 내부 레지스터 주소 정의
 * @{
 */
#define MPU6050_ADDR            0x68  ///< MPU6050 I2C 디바이스 주소
#define MPU6050_WHO_AM_I        0x75  ///< 디바이스 ID 레지스터
#define MPU6050_PWR_MGMT_1      0x6B  ///< 전원 관리 레지스터 1
#define MPU6050_GYRO_CONFIG     0x1B  ///< 자이로스코프 설정 레지스터
#define MPU6050_ACCEL_CONFIG    0x1C  ///< 가속도계 설정 레지스터
#define MPU6050_ACCEL_XOUT_H    0x3B  ///< 가속도계 X축 상위 바이트
#define MPU6050_GYRO_XOUT_H     0x43  ///< 자이로스코프 X축 상위 바이트
/** @} */

/**
 * @brief MPU6050 IMU 센서를 초기화하고 I2C 통신 설정
 * 
 * MPU6050 센서와의 I2C 통신을 설정하고 센서를 초기화합니다.
 * WHO_AM_I 레지스터를 확인하여 센서 연결을 검증하고,
 * 전원 관리, 자이로스코프, 가속도계 설정을 구성합니다.
 * 
 * 초기화 과정:
 * 1. I2C 드라이버 초기화
 * 2. WHO_AM_I 레지스터 확인 (0x68)
 * 3. 전원 관리 레지스터 설정 (슬립 모드 해제)
 * 4. 자이로스코프 범위 설정 (±250°/s)
 * 5. 가속도계 범위 설정 (±2g)
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @param port 사용할 I2C 포트 번호
 * @param sda_pin I2C SDA 핀 번호
 * @param scl_pin I2C SCL 핀 번호
 * @return ESP_OK 성공, ESP_FAIL 센서 연결 실패 또는 I2C 오류
 */
esp_err_t imu_sensor_init(imu_sensor_t* sensor, i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    sensor->i2c_port = port;
    sensor->data.accel_x = sensor->data.accel_y = sensor->data.accel_z = 0.0f;
    sensor->data.gyro_x = sensor->data.gyro_y = sensor->data.gyro_z = 0.0f;
    sensor->data.pitch = sensor->data.roll = 0.0f;
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
        return ret;
    }

    if (who_am_i != MPU6050_ADDR) {
#ifndef NATIVE_BUILD
        ESP_LOGE(IMU_TAG, "MPU6050 not found or wrong ID: 0x%02X", who_am_i);
#endif
        return ESP_FAIL;
    }

    // Wake up MPU6050
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure gyroscope (±250 degrees/s)
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure accelerometer (±2g)
    ret = i2c_write_register(port, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    sensor->data.initialized = true;

#ifndef NATIVE_BUILD
    ESP_LOGI(IMU_TAG, "IMU sensor initialized successfully");
#endif
    return ESP_OK;
}

/**
 * @brief IMU 센서 데이터를 업데이트하여 최신 관성 측정값 수신
 * 
 * MPU6050에서 14바이트의 연속 데이터를 읽어와 가속도계와 자이로스코프 값을 추출합니다.
 * 원시 데이터를 물리적 단위로 변환하고 가속도계 데이터로부터 피치/롤 각도를 계산합니다.
 * 
 * 데이터 처리 과정:
 * 1. I2C로 14바이트 연속 읽기 (가속도 6바이트 + 온도 2바이트 + 자이로 6바이트)
 * 2. 16비트 빅엔디안 데이터를 정수로 변환
 * 3. 스케일링 팩터 적용 (가속도: /16384, 자이로: /131)
 * 4. 가속도계 데이터로 피치/롤 각도 계산 (atan2 함수 사용)
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return ESP_OK 성공, ESP_FAIL 센서가 초기화되지 않음 또는 I2C 오류
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

    sensor->data.gyro_x = gyro_x / 131.0f;      // ±250°/s range
    sensor->data.gyro_y = gyro_y / 131.0f;
    sensor->data.gyro_z = gyro_z / 131.0f;

    // Calculate pitch and roll from accelerometer
    sensor->data.pitch = atan2(-sensor->data.accel_x, sqrt(sensor->data.accel_y * sensor->data.accel_y + sensor->data.accel_z * sensor->data.accel_z)) * 180.0f / M_PI;
    sensor->data.roll = atan2(sensor->data.accel_y, sensor->data.accel_z) * 180.0f / M_PI;

    return ESP_OK;
}

/**
 * @brief 현재 피치(Pitch) 각도 반환
 * 
 * 가속도계 데이터로부터 계산된 Y축 중심 회전 각도를 반환합니다.
 * 피치 각도는 전후 기울어짐을 나타냅니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return 피치 각도 (도 단위, -180° ~ +180°)
 */
float imu_sensor_get_pitch(imu_sensor_t* sensor) {
    return sensor->data.pitch;
}

/**
 * @brief 현재 롤(Roll) 각도 반환
 * 
 * 가속도계 데이터로부터 계산된 X축 중심 회전 각도를 반환합니다.
 * 롤 각도는 좌우 기울어짐을 나타냅니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return 롤 각도 (도 단위, -180° ~ +180°)
 */
float imu_sensor_get_roll(imu_sensor_t* sensor) {
    return sensor->data.roll;
}

/**
 * @brief X축 자이로스코프 각속도 반환
 * 
 * X축(롤축) 중심의 회전 각속도를 반환합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return X축 각속도 (°/s, ±250°/s 범위)
 */
float imu_sensor_get_gyro_x(imu_sensor_t* sensor) {
    return sensor->data.gyro_x;
}

/**
 * @brief Y축 자이로스코프 각속도 반환
 * 
 * Y축(피치축) 중심의 회전 각속도를 반환합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return Y축 각속도 (°/s, ±250°/s 범위)
 */
float imu_sensor_get_gyro_y(imu_sensor_t* sensor) {
    return sensor->data.gyro_y;
}

/**
 * @brief Z축 자이로스코프 각속도 반환
 * 
 * Z축(요축) 중심의 회전 각속도를 반환합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return Z축 각속도 (°/s, ±250°/s 범위)
 */
float imu_sensor_get_gyro_z(imu_sensor_t* sensor) {
    return sensor->data.gyro_z;
}

/**
 * @brief X축 가속도 반환
 * 
 * X축 방향의 선형 가속도를 반환합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return X축 가속도 (g 단위, ±2g 범위)
 */
float imu_sensor_get_accel_x(imu_sensor_t* sensor) {
    return sensor->data.accel_x;
}

/**
 * @brief Y축 가속도 반환
 * 
 * Y축 방향의 선형 가속도를 반환합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return Y축 가속도 (g 단위, ±2g 범위)
 */
float imu_sensor_get_accel_y(imu_sensor_t* sensor) {
    return sensor->data.accel_y;
}

/**
 * @brief Z축 가속도 반환
 * 
 * Z축 방향의 선형 가속도를 반환합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return Z축 가속도 (g 단위, ±2g 범위)
 */
float imu_sensor_get_accel_z(imu_sensor_t* sensor) {
    return sensor->data.accel_z;
}

/**
 * @brief IMU 센서 초기화 상태 확인
 * 
 * 센서가 올바르게 초기화되어 데이터 읽기가 가능한지 확인합니다.
 * 
 * @param sensor IMU 센서 구조체 포인터
 * @return 초기화 완료 시 true, 미완료 시 false
 */
bool imu_sensor_is_initialized(imu_sensor_t* sensor) {
    return sensor->data.initialized;
}