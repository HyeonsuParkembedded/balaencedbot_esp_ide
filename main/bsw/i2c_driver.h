/**
 * @file i2c_driver.h
 * @brief ESP32-C6 Hardware I2C Controller Direct Register Control Driver
 * 
 * ESP32-C6 I2C 하드웨어 컨트롤러의 레지스터를 직접 제어하는 드라이버입니다.
 * MPU6050 IMU 센서와의 통신에 사용되며, 하드웨어 기반으로 CPU 부하를 최소화합니다.
 * 
 * 주요 기능:
 * - ESP32-C6 I2C 컨트롤러 레지스터 직접 제어
 * - 하드웨어 FIFO 기반 데이터 송수신 (32바이트 초과 데이터 자동 분할)
 * - GPIO Matrix를 통한 유연한 핀 매핑
 * - 고속 통신 지원 (Standard 100kHz, Fast 400kHz, Fast+ 1MHz)
 * - 멀티태스크 환경 지원 (Mutex 기반 동기화)
 * - 자동 버스 복구 메커니즘
 * - 동적 APB 클럭 주파수 감지
 * - CPU 부하 최소화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 6.0 (Enhanced with Mutex, Bus Recovery, Large Data Support)
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "gpio_driver.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief ESP32-C6 I2C Hardware Controller Register Definitions
 * ESP32-C6 TRM Chapter 24: I2C Controller
 * 
 * @note ESP32-C6 only has ONE I2C controller (I2C0)
 * @warning I2C1 does NOT exist on ESP32-C6
 */

// I2C Controller Base Addresses (ESP32-C6 has only 1 I2C controller)
#define I2C0_BASE_ADDR          0x60013000UL
// #define I2C1_BASE_ADDR       0x60014000UL  // ❌ NOT AVAILABLE on ESP32-C6

// I2C Register Offsets (ESP32-C6 TRM Chapter 24.4)
#define I2C_SCL_LOW_PERIOD_REG_OFFSET       0x0000  ///< SCL Low Period Register
#define I2C_CTR_REG_OFFSET                  0x0004  ///< Control Register
#define I2C_SR_REG_OFFSET                   0x0008  ///< Status Register
#define I2C_TO_REG_OFFSET                   0x000C  ///< Timeout Register
#define I2C_SLAVE_ADDR_REG_OFFSET           0x0010  ///< Slave Address Register
#define I2C_FIFO_ST_REG_OFFSET              0x0014  ///< FIFO Status Register
#define I2C_FIFO_CONF_REG_OFFSET            0x0018  ///< FIFO Configuration Register
#define I2C_DATA_REG_OFFSET                 0x001C  ///< Data FIFO Register
#define I2C_INT_RAW_REG_OFFSET              0x0020  ///< Raw Interrupt Register
#define I2C_INT_CLR_REG_OFFSET              0x0024  ///< Interrupt Clear Register
#define I2C_INT_ENA_REG_OFFSET              0x0028  ///< Interrupt Enable Register
#define I2C_INT_STATUS_REG_OFFSET           0x002C  ///< Interrupt Status Register
#define I2C_SDA_HOLD_REG_OFFSET             0x0030  ///< SDA Hold Time Register
#define I2C_SDA_SAMPLE_REG_OFFSET           0x0034  ///< SDA Sample Time Register
#define I2C_SCL_HIGH_PERIOD_REG_OFFSET      0x0038  ///< SCL High Period Register
#define I2C_SCL_START_HOLD_REG_OFFSET       0x0040  ///< SCL Start Hold Time Register
#define I2C_SCL_RSTART_SETUP_REG_OFFSET     0x0044  ///< SCL Restart Setup Time Register
#define I2C_SCL_STOP_HOLD_REG_OFFSET        0x0048  ///< SCL Stop Hold Time Register
#define I2C_SCL_STOP_SETUP_REG_OFFSET       0x004C  ///< SCL Stop Setup Time Register
#define I2C_FILTER_CFG_REG_OFFSET           0x0050  ///< Filter Configuration Register
#define I2C_CLK_CONF_REG_OFFSET             0x0054  ///< Clock Configuration Register
#define I2C_COMD0_REG_OFFSET                0x0058  ///< Command 0 Register
#define I2C_COMD1_REG_OFFSET                0x005C  ///< Command 1 Register
#define I2C_COMD2_REG_OFFSET                0x0060  ///< Command 2 Register
#define I2C_COMD3_REG_OFFSET                0x0064  ///< Command 3 Register
#define I2C_COMD4_REG_OFFSET                0x0068  ///< Command 4 Register
#define I2C_COMD5_REG_OFFSET                0x006C  ///< Command 5 Register
#define I2C_COMD6_REG_OFFSET                0x0070  ///< Command 6 Register
#define I2C_COMD7_REG_OFFSET                0x0074  ///< Command 7 Register

// I2C Control Register Bits (CTR_REG)
#define I2C_CTR_TRANS_START_BIT             (1U << 5)   ///< Transaction Start
#define I2C_CTR_MS_MODE_BIT                 (1U << 4)   ///< Master Mode Enable
#define I2C_CTR_RX_LSB_FIRST_BIT            (1U << 1)   ///< RX LSB First
#define I2C_CTR_TX_LSB_FIRST_BIT            (1U << 0)   ///< TX LSB First

// I2C Status Register Bits (SR_REG)
#define I2C_SR_BUS_BUSY_BIT                 (1U << 4)   ///< Bus Busy Status
#define I2C_SR_SLAVE_ADDRESSED_BIT          (1U << 3)   ///< Slave Addressed
#define I2C_SR_BYTE_TRANS_BIT               (1U << 2)   ///< Byte Transmission
#define I2C_SR_ARB_LOST_BIT                 (1U << 0)   ///< Arbitration Lost

// I2C Interrupt Bits
#define I2C_INT_TRANS_COMPLETE_BIT          (1U << 7)   ///< Transaction Complete
#define I2C_INT_MASTER_TRAN_COMP_BIT        (1U << 6)   ///< Master Transaction Complete
#define I2C_INT_ARBITRATION_LOST_BIT        (1U << 5)   ///< Arbitration Lost
#define I2C_INT_TIME_OUT_BIT                (1U << 8)   ///< Timeout
#define I2C_INT_NACK_BIT                    (1U << 10)  ///< NACK Received
#define I2C_INT_TXFIFO_EMPTY_BIT            (1U << 1)   ///< TX FIFO Empty
#define I2C_INT_RXFIFO_FULL_BIT             (1U << 0)   ///< RX FIFO Full

// I2C Command Register Bits (COMDn_REG)
#define I2C_COMMAND_DONE_BIT                (1U << 31)  ///< Command Done
#define I2C_COMMAND_OPCODE_SHIFT            11          ///< Command Opcode Shift
#define I2C_COMMAND_BYTE_NUM_SHIFT          0           ///< Command Byte Number Shift

// I2C Command Opcodes
#define I2C_CMD_RSTART                      0x06        ///< Restart
#define I2C_CMD_WRITE                       0x01        ///< Write
#define I2C_CMD_READ                        0x02        ///< Read
#define I2C_CMD_STOP                        0x03        ///< Stop
#define I2C_CMD_END                         0x04        ///< End

// Helper Macros for Register Access
#define I2C_REG_ADDR(base, offset)          ((base) + (offset))
#define I2C_READ_REG(base, offset)          (*(volatile uint32_t*)I2C_REG_ADDR(base, offset))
#define I2C_WRITE_REG(base, offset, val)    (*(volatile uint32_t*)I2C_REG_ADDR(base, offset) = (val))
#define I2C_SET_BITS(base, offset, bits)    I2C_WRITE_REG(base, offset, I2C_READ_REG(base, offset) | (bits))
#define I2C_CLEAR_BITS(base, offset, bits)  I2C_WRITE_REG(base, offset, I2C_READ_REG(base, offset) & ~(bits))

/**
 * @brief BSW I2C Port Type Definition
 * 
 * @warning ESP32-C6 ONLY supports I2C0!
 * @note BSW_I2C_PORT_1 is defined for code compatibility but CANNOT be used on ESP32-C6
 * @note Attempting to use I2C1 will result in ESP_ERR_INVALID_ARG
 */
typedef enum {
    BSW_I2C_PORT_0 = 0,         ///< I2C Port 0 (Base: 0x60013000) - ✅ Available on ESP32-C6
    BSW_I2C_PORT_1,             ///< I2C Port 1 - ❌ NOT available on ESP32-C6 (reserved for compatibility)
    BSW_I2C_PORT_MAX            ///< Maximum port count (=2 for enum, but only 1 usable on ESP32-C6)
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
#define I2C_FIFO_SIZE               32                 ///< Hardware FIFO Size

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup I2C_DRIVER ESP32-C6 Hardware I2C Controller Driver API
 * @brief ESP32-C6 하드웨어 I2C 컨트롤러 레지스터 직접 제어 인터페이스
 * @{
 */

/**
 * @brief I2C Hardware Controller Initialization (Default 100kHz)
 * 
 * ESP32-C6 I2C 하드웨어 컨트롤러의 레지스터를 직접 설정하여 초기화합니다.
 * GPIO 핀 매핑, 클럭 설정, FIFO 설정 등을 수행합니다.
 * 
 * @param port I2C 포트 번호 (0 또는 1)
 * @param sda_pin SDA GPIO 핀 번호
 * @param scl_pin SCL GPIO 핀 번호
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_ERR_INVALID_ARG: 잘못된 인자
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note I2C 컨트롤러 클럭, 타이밍 파라미터, FIFO 모두 하드웨어적으로 설정됩니다.
 * @note GPIO는 I2C 기능으로 매핑되며 오픈 드레인 모드로 설정됩니다.
 */
esp_err_t i2c_driver_init(bsw_i2c_port_t port, bsw_gpio_num_t sda_pin, bsw_gpio_num_t scl_pin);

/**
 * @brief I2C Hardware Controller Initialization (Custom Configuration)
 * 
 * 사용자 정의 설정으로 I2C 하드웨어 컨트롤러를 초기화합니다.
 * 
 * @param port I2C 포트 번호
 * @param config I2C 하드웨어 설정 구조체
 * @return esp_err_t 성공/실패
 * 
 * @note 고속 모드(400kHz, 1MHz) 설정 가능
 */
esp_err_t i2c_driver_init_config(bsw_i2c_port_t port, const i2c_hw_config_t* config);

/**
 * @brief I2C Device Register Write (Hardware Controller)
 * 
 * I2C 하드웨어 컨트롤러의 COMMAND 레지스터와 FIFO를 사용하여
 * 디바이스 레지스터에 1바이트 데이터를 씁니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param value 쓸 데이터 (1바이트)
 * @return esp_err_t 
 *         - ESP_OK: 쓰기 성공
 *         - ESP_ERR_TIMEOUT: 타임아웃
 *         - ESP_FAIL: NACK 또는 통신 실패
 * 
 * @note 하드웨어 FIFO와 COMMAND 레지스터로 자동 처리
 * @note Protocol: START -> ADDR+W -> REG -> DATA -> STOP
 */
esp_err_t i2c_write_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

/**
 * @brief I2C Device Register Read (Hardware Controller)
 * 
 * I2C 하드웨어 컨트롤러를 사용하여 디바이스 레지스터에서
 * 멀티바이트 데이터를 읽습니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param reg_addr 레지스터 주소
 * @param data 읽은 데이터를 저장할 버퍼
 * @param len 읽을 데이터 길이 (바이트)
 * @return esp_err_t 
 *         - ESP_OK: 읽기 성공
 *         - ESP_ERR_TIMEOUT: 타임아웃
 *         - ESP_FAIL: NACK 또는 통신 실패
 * 
 * @note 하드웨어 FIFO 자동 처리로 CPU 부하 최소화
 * @note Protocol: START -> ADDR+W -> REG -> RESTART -> ADDR+R -> DATA... -> STOP
 * @warning data 버퍼는 len 바이트 이상이어야 함
 */
esp_err_t i2c_read_register(bsw_i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);

/**
 * @brief I2C Raw Data Write (Hardware Controller)
 * 
 * I2C 버스에 원시 데이터를 하드웨어 컨트롤러로 씁니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param data 쓸 데이터 버퍼
 * @param len 데이터 길이
 * @return esp_err_t 성공/실패
 * 
 * @note FIFO 기반 고속 전송
 */
esp_err_t i2c_write_raw(bsw_i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len);

/**
 * @brief I2C Raw Data Read (Hardware Controller)
 * 
 * I2C 버스에서 원시 데이터를 하드웨어 컨트롤러로 읽습니다.
 * 
 * @param port I2C 포트 번호
 * @param device_addr I2C 디바이스 주소 (7비트)
 * @param data 읽을 데이터 버퍼
 * @param len 데이터 길이
 * @return esp_err_t 성공/실패
 * 
 * @note FIFO 기반 고속 수신
 */
esp_err_t i2c_read_raw(bsw_i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len);

/**
 * @brief I2C 드라이버 해제
 * 
 * I2C 관련 자원을 해제하고 GPIO 핀을 원래 상태로 복원합니다.
 * 
 * @param port I2C 포트 번호
 * @return esp_err_t 
 *         - ESP_OK: 해제 성공
 *         - ESP_FAIL: 해제 실패
 */
esp_err_t i2c_driver_deinit(bsw_i2c_port_t port);

/**
 * @brief I2C Bus Recovery Function
 * 
 * I2C 버스가 hang 상태일 때 SCL 클럭 펄스를 생성하여 복구합니다.
 * 
 * @param port I2C 포트 번호
 * @return esp_err_t 
 *         - ESP_OK: 복구 성공
 *         - ESP_ERR_INVALID_STATE: 초기화되지 않음
 * 
 * @note 이 함수는 통신 실패 시 자동으로 호출됩니다.
 * @note 필요한 경우 수동으로 호출할 수도 있습니다.
 */
esp_err_t i2c_bus_recovery(bsw_i2c_port_t port);

/** @} */ // I2C_DRIVER

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H