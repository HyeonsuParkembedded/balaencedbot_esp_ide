/**
 * @file i2c_driver.c
 * @brief I2C 통신 드라이버 구현 파일
 * 
 * ESP32-S3의 I2C 하드웨어를 이용한 마스터 모드 통신 구현입니다.
 * MPU6050 IMU 센서와의 통신에 최적화되어 있습니다.
 * 
 * 구현 특징:
 * - 400kHz 고속 통신 지원
 * - 내부 풀업 저항 활성화
 * - 1초 타임아웃 설정
 * - 네이티브 빌드 지원 (테스트용)
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "i2c_driver.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#endif

#ifndef NATIVE_BUILD
static const char* I2C_TAG = "I2C_DRIVER"; ///< ESP-IDF 로깅 태그
#else
#define I2C_TAG "I2C_DRIVER" ///< 네이티브 빌드용 로깅 태그
#endif

/**
 * @brief I2C 인터페이스 초기화 구현
 * 
 * ESP-IDF I2C 드라이버를 사용하여 마스터 모드로 초기화합니다.
 * 
 * 설정 파라미터:
 * - 모드: I2C_MODE_MASTER
 * - 클록 속도: 400kHz (고속 모드)
 * - 풀업 저항: 활성화 (긴 케이블 대응)
 * - 버퍼 크기: 0 (폴링 모드)
 * 
 * @param port I2C 포트 번호
 * @param sda_pin SDA 핀 번호
 * @param scl_pin SCL 핀 번호
 * @return esp_err_t 초기화 결과
 */
esp_err_t i2c_driver_init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
#ifndef NATIVE_BUILD
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // 400kHz
    };

    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "I2C param config failed");
        return ret;
    }

    ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "I2C driver install failed");
        return ret;
    }

    ESP_LOGI(I2C_TAG, "I2C driver initialized");
#endif
    return ESP_OK;
}

/**
 * @brief I2C 디바이스 레지스터 쓰기 구현
 * 
 * I2C 명령 링크를 사용하여 디바이스 레지스터에 1바이트 데이터를 씁니다.
 * 
 * I2C 전송 시퀀스:
 * 1. START 조건 생성
 * 2. 디바이스 주소 + WRITE 비트 전송
 * 3. 레지스터 주소 전송
 * 4. 데이터 값 전송
 * 5. STOP 조건 생성
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 값
 * @return esp_err_t 전송 결과
 */
esp_err_t i2c_write_register(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
#ifndef NATIVE_BUILD
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1), true);  // Removed | I2C_MASTER_WRITE (redundant)
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
#else
    return ESP_OK;
#endif
}

/**
 * @brief I2C 디바이스 레지스터 읽기 구현
 * 
 * I2C 명령 링크를 사용하여 디바이스 레지스터에서 멀티바이트 데이터를 읽습니다.
 * 
 * I2C 전송 시퀀스:
 * 1. START 조건 생성
 * 2. 디바이스 주소 + WRITE 비트 전송
 * 3. 레지스터 주소 전송
 * 4. Repeated START 조건 생성
 * 5. 디바이스 주소 + READ 비트 전송
 * 6. 데이터 읽기 (마지막 바이트는 NACK)
 * 7. STOP 조건 생성
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param data 읽은 데이터를 저장할 버퍼
 * @param len 읽을 데이터 길이
 * @return esp_err_t 전송 결과
 */
esp_err_t i2c_read_register(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
#ifndef NATIVE_BUILD
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1), true);  // Removed | I2C_MASTER_WRITE (redundant)
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
#else
    // Mock data for native build
    for (size_t i = 0; i < len; i++) {
        data[i] = 0x42 + i;
    }
    return ESP_OK;
#endif
}