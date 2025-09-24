/**
 * @file i2c_driver.c
 * @brief ESP32-C6 I2C Master Driver Implementation using new ESP-IDF v5.x API
 * 
 * ESP32-C6의 I2C 하드웨어를 이용한 마스터 모드 통신 구현입니다.
 * MPU6050 IMU 센서와의 통신에 최적화되어 있습니다.
 * 
 * 구현 특징:
 * - 새로운 ESP-IDF v5.x I2C API 사용
 * - 400kHz 고속 통신 지원
 * - 내부 풀업 저항 활성화
 * - 무제한 타임아웃 설정 (-1)
 * - 네이티브 빌드 지원 (테스트용)
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 2.0
 */

#include "i2c_driver.h"
#include "config.h"
#ifndef NATIVE_BUILD
#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "esp_log.h"
#endif

#ifndef NATIVE_BUILD
static const char* I2C_TAG = "I2C_DRIVER";
static i2c_master_bus_handle_t bus_handle = NULL;

// ESP-IDF I2C 포트 매핑
static const i2c_port_t esp_i2c_port_map[] = {
    I2C_NUM_0  // ESP32-C6는 I2C_NUM_0만 지원
};
#else
#define I2C_TAG "I2C_DRIVER"
#endif

/**
 * @brief I2C 인터페이스 초기화 구현 (새로운 API 사용)
 * 
 * ESP-IDF v5.x의 새로운 I2C 드라이버를 사용하여 마스터 버스를 초기화합니다.
 * 
 * 설정 파라미터:
 * - 클록 소스: I2C_CLK_SRC_DEFAULT (ESP32-C6 호환)
 * - 클록 속도: 400kHz (고속 모드) 
 * - 풀업 저항: 활성화 (긴 케이블 대응)
 * - 글리치 필터: 7카운트
 * 
 * @param port I2C 포트 번호 (새 API에서는 자동 할당)
 * @param sda_pin SDA 핀 번호
 * @param scl_pin SCL 핀 번호
 * @return esp_err_t 초기화 결과
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
#ifndef NATIVE_BUILD
    // 새로운 I2C 마스터 버스 구성
    // BSW 포트를 ESP-IDF 포트로 매핑
    if (port >= BSW_I2C_PORT_MAX) {
        ESP_LOGE(I2C_TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,        // ESP32-C6 기본 클록 소스
        .i2c_port = esp_i2c_port_map[port],       // ESP-IDF I2C 포트 번호
        .scl_io_num = scl_pin,                    // SCL 핀
        .sda_io_num = sda_pin,                    // SDA 핀
        .glitch_ignore_cnt = 7,                   // 표준 글리치 필터
        .flags.enable_internal_pullup = true,    // ESP32-C6 내부 풀업 활성화
    };
    
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "I2C master bus creation failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(I2C_TAG, "I2C driver initialized on ESP32-C6 with pins SCL=%d, SDA=%d", 
             scl_pin, sda_pin);
#endif
    return ESP_OK;
}

/**
 * @brief I2C 디바이스 레지스터 쓰기 구현 (새로운 API 사용)
 * 
 * 새로운 ESP-IDF v5.x API를 사용하여 디바이스 레지스터에 1바이트 데이터를 씁니다.
 * 
 * 전송 절차:
 * 1. 디바이스 구성 생성 (주소, 속도)
 * 2. 디바이스를 버스에 추가
 * 3. 레지스터 주소와 데이터 전송
 * 4. 디바이스 제거
 * 
 * @param port I2C 포트 번호 (새 API에서는 사용 안함)
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 값
 * @return esp_err_t 전송 결과
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
#ifndef NATIVE_BUILD
    if (bus_handle == NULL) {
        ESP_LOGE(I2C_TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // I2C 디바이스 구성
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,   // 7비트 주소
        .device_address = device_addr,           // 디바이스 주소
        .scl_speed_hz = 400000,                  // 400kHz 클록 속도
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 전송 데이터 준비 (레지스터 주소 + 값)
    uint8_t write_buf[2] = {reg_addr, value};
    
    // 데이터 전송
    ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
    
    // 디바이스 제거
    i2c_master_bus_rm_device(dev_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
#else
    return ESP_OK;
#endif
}

/**
 * @brief I2C 디바이스 레지스터 읽기 구현 (새로운 API 사용)
 * 
 * 새로운 ESP-IDF v5.x API를 사용하여 디바이스 레지스터에서 멀티바이트 데이터를 읽습니다.
 * 
 * 전송 절차:
 * 1. 디바이스 구성 생성 (주소, 속도)
 * 2. 디바이스를 버스에 추가
 * 3. 레지스터 주소 쓰기 후 데이터 읽기 (transmit_receive)
 * 4. 디바이스 제거
 * 
 * @param port I2C 포트 번호 (새 API에서는 사용 안함)
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param data 읽은 데이터를 저장할 버퍼
 * @param len 읽을 데이터 길이
 * @return esp_err_t 전송 결과
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
#ifndef NATIVE_BUILD
    if (bus_handle == NULL) {
        ESP_LOGE(I2C_TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // I2C 디바이스 구성
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,   // 7비트 주소
        .device_address = device_addr,           // 디바이스 주소
        .scl_speed_hz = 400000,                  // 400kHz 클록 속도
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 레지스터 주소 쓰기 후 데이터 읽기 (한 번의 트랜잭션)
    ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, -1);
    
    // 디바이스 제거
    i2c_master_bus_rm_device(dev_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
#else
    // 네이티브 빌드용 모의 데이터
    for (size_t i = 0; i < len; i++) {
        data[i] = 0x42 + i;
    }
    return ESP_OK;
#endif
}

/**
 * @brief I2C 드라이버 해제
 * 
 * I2C 마스터 버스와 관련 자원을 해제합니다.
 * 
 * @return esp_err_t 해제 결과
 */
esp_err_t i2c_driver_deinit(void) {
#ifndef NATIVE_BUILD
    if (bus_handle != NULL) {
        esp_err_t ret = i2c_del_master_bus(bus_handle);
        if (ret == ESP_OK) {
            bus_handle = NULL;
            ESP_LOGI(I2C_TAG, "I2C driver deinitialized");
        } else {
            ESP_LOGE(I2C_TAG, "I2C driver deinit failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
#endif
    return ESP_OK;
}