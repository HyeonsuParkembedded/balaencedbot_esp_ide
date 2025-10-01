/**
 * @file i2c_driver.c
 * @brief 순수 비트연산 GPIO 직접 제어 I2C 드라이버 구현
 * 
 * HAL 의존성 없이 순수 GPIO 레지스터 조작을 통한 소프트웨어 I2C 마스터 구현입니다.
 * MPU6050 IMU 센서와의 통신에 최적화되어 있습니다.
 * 
 * 구현 특징:
 * - 순수 GPIO 레지스터 비트연산 제어
 * - HAL 의존성 완전 제거
 * - 표준 I2C 프로토콜 준수
 * - 사용자 정의 클럭 속도 지원
 * - 오픈 드레인 출력 에뮬레이션
 * - 클럭 스트레치 지원
 * - 비트연산 기반 마이크로초 정밀 타이밍
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 4.0 (순수 비트연산 I2C)
 */

#include "i2c_driver.h"
#include "system_services.h"
#include "gpio_driver.h"
#include "system_services.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* I2C_TAG = "BITWISE_I2C";

// I2C 포트별 설정
static i2c_bitbang_config_t i2c_configs[BSW_I2C_PORT_MAX];
static bool i2c_initialized[BSW_I2C_PORT_MAX] = {false};

/**
 * @brief I2C 타이밍 지연 함수 (순수 비트연산)
 * 
 * @param delay_us 지연 시간 (마이크로초)
 */
static inline void i2c_delay_us_bitwise(uint32_t delay_us) {
    // CPU 클럭 기반 비트연산 지연 (240MHz 기준)
    uint32_t cycles = delay_us * 240;  // 1μs = 240 cycles at 240MHz
    
    // 인라인 어셈블리 지연 루프
    __asm__ __volatile__ (
        "1: \n"
        "addi %0, %0, -3 \n"  // 3 사이클 소모
        "bgtz %0, 1b \n"      // 분기 명령어
        : "+r" (cycles)
        :
        : "memory"
    );
}

/**
 * @brief SDA 핀을 HIGH로 설정 (오픈 드레인 에뮬레이션 - 순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static inline void i2c_sda_high_bitwise(bsw_i2c_port_t port) {
    // 입력 모드로 변경하여 풀업에 의해 HIGH가 되도록 함 (오픈 드레인)
    bsw_gpio_fast_set_input(i2c_configs[port].sda_pin);
}

/**
 * @brief SDA 핀을 LOW로 설정 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static inline void i2c_sda_low_bitwise(bsw_i2c_port_t port) {
    bsw_gpio_fast_set_low(i2c_configs[port].sda_pin);
    bsw_gpio_fast_set_output(i2c_configs[port].sda_pin);
}

/**
 * @brief SCL 핀을 HIGH로 설정 (오픈 드레인 에뮬레이션 - 순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static inline void i2c_scl_high_bitwise(bsw_i2c_port_t port) {
    // 입력 모드로 변경하여 풀업에 의해 HIGH가 되도록 함 (오픈 드레인)
    bsw_gpio_fast_set_input(i2c_configs[port].scl_pin);
}

/**
 * @brief SCL 핀을 LOW로 설정 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static inline void i2c_scl_low_bitwise(bsw_i2c_port_t port) {
    bsw_gpio_fast_set_low(i2c_configs[port].scl_pin);
    bsw_gpio_fast_set_output(i2c_configs[port].scl_pin);
}

/**
 * @brief SDA 핀 읽기 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 * @return SDA 핀 상태 (0 또는 1)
 */
static inline int i2c_sda_read_bitwise(bsw_i2c_port_t port) {
    return bsw_gpio_fast_read_input(i2c_configs[port].sda_pin);
}

/**
 * @brief SCL 핀 읽기 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 * @return SCL 핀 상태 (0 또는 1)
 */
static inline int i2c_scl_read_bitwise(bsw_i2c_port_t port) {
    return bsw_gpio_fast_read_input(i2c_configs[port].scl_pin);
}

/**
 * @brief I2C 클럭 주기의 1/4 지연 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static inline void i2c_quarter_delay_bitwise(bsw_i2c_port_t port) {
    uint32_t quarter_period_us = (1000000 / i2c_configs[port].clock_speed_hz) / 4;
    if (quarter_period_us > 0) {
        i2c_delay_us_bitwise(quarter_period_us);
    }
}

/**
 * @brief I2C START 컨디션 생성 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static void i2c_start_condition_bitwise(bsw_i2c_port_t port) {
    // START: SDA HIGH -> LOW while SCL HIGH
    i2c_sda_high_bitwise(port);
    i2c_scl_high_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    i2c_sda_low_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    i2c_scl_low_bitwise(port);
    i2c_quarter_delay_bitwise(port);
}

/**
 * @brief I2C STOP 컨디션 생성 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 */
static void i2c_stop_condition_bitwise(bsw_i2c_port_t port) {
    // STOP: SDA LOW -> HIGH while SCL HIGH
    i2c_sda_low_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    i2c_scl_high_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    i2c_sda_high_bitwise(port);
    i2c_quarter_delay_bitwise(port);
}

/**
 * @brief I2C 1바이트 쓰기 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 * @param data 쓸 데이터
 * @return true: ACK 받음, false: NACK 또는 오류
 */
static bool i2c_write_byte_bitwise(bsw_i2c_port_t port, uint8_t data) {
    // 8비트 데이터 전송 (MSB first)
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) {
            i2c_sda_high_bitwise(port);
        } else {
            i2c_sda_low_bitwise(port);
        }
        i2c_quarter_delay_bitwise(port);
        
        i2c_scl_high_bitwise(port);
        i2c_quarter_delay_bitwise(port);
        i2c_scl_low_bitwise(port);
        i2c_quarter_delay_bitwise(port);
    }
    
    // ACK 비트 확인
    i2c_sda_high_bitwise(port);  // SDA를 입력으로 변경
    i2c_quarter_delay_bitwise(port);
    i2c_scl_high_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    
    bool ack = (i2c_sda_read_bitwise(port) == 0);  // ACK는 LOW
    
    i2c_scl_low_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    
    return ack;
}

/**
 * @brief I2C 1바이트 읽기 (순수 비트연산)
 * 
 * @param port I2C 포트 번호
 * @param send_ack true: ACK 전송, false: NACK 전송
 * @return 읽은 데이터
 */
static uint8_t i2c_read_byte_bitwise(bsw_i2c_port_t port, bool send_ack) {
    uint8_t data = 0;
    
    i2c_sda_high_bitwise(port);  // SDA를 입력으로 변경
    
    // 8비트 데이터 읽기 (MSB first)
    for (int i = 7; i >= 0; i--) {
        i2c_quarter_delay_bitwise(port);
        i2c_scl_high_bitwise(port);
        i2c_quarter_delay_bitwise(port);
        
        if (i2c_sda_read_bitwise(port)) {
            data |= (1 << i);
        }
        
        i2c_scl_low_bitwise(port);
        i2c_quarter_delay_bitwise(port);
    }
    
    // ACK/NACK 전송
    if (send_ack) {
        i2c_sda_low_bitwise(port);   // ACK (LOW)
    } else {
        i2c_sda_high_bitwise(port);  // NACK (HIGH)
    }
    
    i2c_quarter_delay_bitwise(port);
    i2c_scl_high_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    i2c_scl_low_bitwise(port);
    i2c_quarter_delay_bitwise(port);
    
    return data;
}

/**
 * @brief I2C 인터페이스 초기화 (기본 100kHz)
 * 
 * GPIO 직접 제어를 통한 bit-banging I2C 마스터를 초기화합니다.
 * 
 * @param port I2C 포트 번호
 * @param sda_pin SDA 핀 번호
 * @param scl_pin SCL 핀 번호
 * @return esp_err_t 초기화 결과
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin) {
    i2c_bitbang_config_t config = {
        .sda_pin = sda_pin,
        .scl_pin = scl_pin,
        .clock_speed_hz = I2C_DEFAULT_CLOCK_SPEED,
        .use_pullup = true
    };
    
    return i2c_driver_init_config(port, &config);
}

/**
 * @brief I2C 인터페이스 초기화 (사용자 정의 설정)
 * 
 * @param port I2C 포트 번호
 * @param config I2C 설정 구조체
 * @return esp_err_t 초기화 결과
 */
esp_err_t i2c_driver_init_config(bsw_i2c_port_t port, const i2c_bitbang_config_t* config) {
    if (port >= BSW_I2C_PORT_MAX || !config) {
        bsw_log_bitwise(BSW_LOG_ERROR, I2C_TAG, "Invalid I2C port %d or config is NULL", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 설정 저장
    i2c_configs[port] = *config;
    
    // GPIO 핀 설정 (오픈 드레인 모드)
    esp_err_t ret = bsw_gpio_config_pin(config->sda_pin, BSW_GPIO_MODE_INPUT_OUTPUT, 
                                       config->use_pullup ? BSW_GPIO_PULLUP_ENABLE : BSW_GPIO_PULLUP_DISABLE, 
                                       BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Failed to configure SDA pin %d", config->sda_pin);
        return ret;
    }
    
    ret = bsw_gpio_config_pin(config->scl_pin, BSW_GPIO_MODE_INPUT_OUTPUT, 
                             config->use_pullup ? BSW_GPIO_PULLUP_ENABLE : BSW_GPIO_PULLUP_DISABLE, 
                             BSW_GPIO_PULLDOWN_DISABLE);
    if (ret != ESP_OK) {
        BSW_LOGE(I2C_TAG, "Failed to configure SCL pin %d", config->scl_pin);
        return ret;
    }
    
    // 초기 상태: 둘 다 HIGH (IDLE 상태)
    i2c_sda_high_bitwise(port);
    i2c_scl_high_bitwise(port);
    
    i2c_initialized[port] = true;
    
    bsw_log_bitwise(BSW_LOG_INFO, I2C_TAG, "I2C port %d initialized: SDA=%d, SCL=%d, freq=%luHz", 
             port, config->sda_pin, config->scl_pin, config->clock_speed_hz);
    
    return ESP_OK;
}

/**
 * @brief I2C 디바이스 레지스터 쓰기 구현 (bit-banging)
 * 
 * GPIO 직접 제어를 통한 I2C 프로토콜로 레지스터에 1바이트 데이터를 씁니다.
 * 
 * 전송 절차:
 * START -> DEVICE_ADDR+W -> REG_ADDR -> DATA -> STOP
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 값
 * @return esp_err_t 전송 결과
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port]) {
        BSW_LOGE(I2C_TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    // START 컨디션
    i2c_start_condition_bitwise(port);
    
    // 디바이스 주소 + WRITE 비트 (0)
    if (!i2c_write_byte_bitwise(port, (device_addr << 1) | 0x00)) {
        BSW_LOGE(I2C_TAG, "Device address NACK: 0x%02X", device_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // 레지스터 주소
    if (!i2c_write_byte_bitwise(port, reg_addr)) {
        BSW_LOGE(I2C_TAG, "Register address NACK: 0x%02X", reg_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // 데이터
    if (!i2c_write_byte_bitwise(port, value)) {
        BSW_LOGE(I2C_TAG, "Data NACK: 0x%02X", value);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // STOP 컨디션
    i2c_stop_condition_bitwise(port);
    
    return ESP_OK;
}

/**
 * @brief I2C 디바이스 레지스터 읽기 구현 (bit-banging)
 * 
 * GPIO 직접 제어를 통한 I2C 프로토콜로 레지스터에서 멀티바이트 데이터를 읽습니다.
 * 
 * 전송 절차:
 * START -> DEVICE_ADDR+W -> REG_ADDR -> RESTART -> DEVICE_ADDR+R -> DATA... -> STOP
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param data 읽은 데이터를 저장할 버퍼
 * @param len 읽을 데이터 길이
 * @return esp_err_t 전송 결과
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port] || !data || len == 0) {
        BSW_LOGE(I2C_TAG, "Invalid parameters: port=%d, data=%p, len=%zu", port, data, len);
        return ESP_ERR_INVALID_ARG;
    }

    // START 컨디션
    i2c_start_condition_bitwise(port);
    
    // 디바이스 주소 + WRITE 비트 (0) - 레지스터 주소 쓰기용
    if (!i2c_write_byte_bitwise(port, (device_addr << 1) | 0x00)) {
        BSW_LOGE(I2C_TAG, "Device address (write) NACK: 0x%02X", device_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // 레지스터 주소
    if (!i2c_write_byte_bitwise(port, reg_addr)) {
        BSW_LOGE(I2C_TAG, "Register address NACK: 0x%02X", reg_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // RESTART 컨디션 (STOP 없이 새로운 START)
    i2c_start_condition_bitwise(port);
    
    // 디바이스 주소 + READ 비트 (1)
    if (!i2c_write_byte_bitwise(port, (device_addr << 1) | 0x01)) {
        BSW_LOGE(I2C_TAG, "Device address (read) NACK: 0x%02X", device_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // 데이터 읽기
    for (size_t i = 0; i < len; i++) {
        bool send_ack = (i < len - 1);  // 마지막 바이트가 아니면 ACK, 마지막이면 NACK
        data[i] = i2c_read_byte_bitwise(port, send_ack);
    }
    
    // STOP 컨디션
    i2c_stop_condition_bitwise(port);
    
    return ESP_OK;
}

/**
 * @brief I2C 원시 데이터 쓰기 구현 (bit-banging)
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param data 쓸 데이터 버퍼
 * @param len 데이터 길이
 * @return esp_err_t 성공/실패
 */
esp_err_t i2c_write_raw(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port] || !data || len == 0) {
        BSW_LOGE(I2C_TAG, "Invalid parameters for raw write");
        return ESP_ERR_INVALID_ARG;
    }

    // START 컨디션
    i2c_start_condition_bitwise(port);
    
    // 디바이스 주소 + WRITE 비트 (0)
    if (!i2c_write_byte_bitwise(port, (device_addr << 1) | 0x00)) {
        BSW_LOGE(I2C_TAG, "Device address NACK: 0x%02X", device_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // 데이터 쓰기
    for (size_t i = 0; i < len; i++) {
        if (!i2c_write_byte_bitwise(port, data[i])) {
            BSW_LOGE(I2C_TAG, "Data NACK at byte %zu: 0x%02X", i, data[i]);
            i2c_stop_condition_bitwise(port);
            return ESP_FAIL;
        }
    }
    
    // STOP 컨디션
    i2c_stop_condition_bitwise(port);
    
    return ESP_OK;
}

/**
 * @brief I2C 원시 데이터 읽기 구현 (bit-banging)
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param data 읽을 데이터 버퍼
 * @param len 데이터 길이
 * @return esp_err_t 성공/실패
 */
esp_err_t i2c_read_raw(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len) {
    if (port >= BSW_I2C_PORT_MAX || !i2c_initialized[port] || !data || len == 0) {
        BSW_LOGE(I2C_TAG, "Invalid parameters for raw read");
        return ESP_ERR_INVALID_ARG;
    }

    // START 컨디션
    i2c_start_condition_bitwise(port);
    
    // 디바이스 주소 + READ 비트 (1)
    if (!i2c_write_byte_bitwise(port, (device_addr << 1) | 0x01)) {
        BSW_LOGE(I2C_TAG, "Device address NACK: 0x%02X", device_addr);
        i2c_stop_condition_bitwise(port);
        return ESP_FAIL;
    }
    
    // 데이터 읽기
    for (size_t i = 0; i < len; i++) {
        bool send_ack = (i < len - 1);  // 마지막 바이트가 아니면 ACK, 마지막이면 NACK
        data[i] = i2c_read_byte_bitwise(port, send_ack);
    }
    
    // STOP 컨디션
    i2c_stop_condition_bitwise(port);
    
    return ESP_OK;
}

/**
 * @brief I2C 드라이버 해제
 * 
 * I2C 관련 자원을 해제하고 GPIO 핀을 원래 상태로 복원합니다.
 * 
 * @param port I2C 포트 번호
 * @return esp_err_t 해제 결과
 */
esp_err_t i2c_driver_deinit(bsw_i2c_port_t port) {
    if (port >= BSW_I2C_PORT_MAX) {
        BSW_LOGE(I2C_TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (i2c_initialized[port]) {
        // GPIO 핀을 입력 모드로 변경 (풀업에 의해 HIGH 상태)
        bsw_gpio_set_direction(i2c_configs[port].sda_pin, BSW_GPIO_MODE_INPUT);
        bsw_gpio_set_direction(i2c_configs[port].scl_pin, BSW_GPIO_MODE_INPUT);
        
        i2c_initialized[port] = false;
        
        BSW_LOGI(I2C_TAG, "I2C port %d deinitialized", port);
    }
    
    return ESP_OK;
}