/**
 * @file spi_driver.h
 * @brief ESP32-C6 Hardware SPI Controller Direct Register Control Driver
 * 
 * ESP32-C6 SPI 하드웨어 컨트롤러의 레지스터를 직접 제어하는 드라이버입니다.
 * 외부 SPI 디바이스와의 고속 통신에 사용되며, 하드웨어 기반으로 CPU 부하를 최소화합니다.
 * 
 * 주요 기능:
 * - ESP32-C6 SPI 컨트롤러 레지스터 직접 제어
 * - Master 모드 지원
 * - 고속 통신 지원 (최대 40MHz)
 * - Full-duplex 및 Half-duplex 모드
 * - CPOL/CPHA 설정 가능 (SPI Mode 0-3)
 * - 하드웨어 기반 Chip Select 제어
 * - CPU 부하 최소화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 1.0 (Hardware SPI Controller)
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "gpio_driver.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief ESP32-C6 SPI Hardware Controller Register Definitions
 * ESP32-C6 TRM Chapter 28: SPI Controller
 */

// SPI Controller Base Addresses (ESP32-C6 has 2 SPI controllers: SPI2)
#define SPI2_BASE_ADDR          0x60003000UL    ///< SPI2 Base Address

// SPI Register Offsets (ESP32-C6 TRM Chapter 28)
#define SPI_CMD_REG_OFFSET              0x0000  ///< Command Register
#define SPI_ADDR_REG_OFFSET             0x0004  ///< Address Register
#define SPI_CTRL_REG_OFFSET             0x0008  ///< Control Register
#define SPI_CLOCK_REG_OFFSET            0x0014  ///< Clock Register
#define SPI_USER_REG_OFFSET             0x0018  ///< User Register
#define SPI_USER1_REG_OFFSET            0x001C  ///< User1 Register
#define SPI_USER2_REG_OFFSET            0x0020  ///< User2 Register
#define SPI_MS_DLEN_REG_OFFSET          0x0024  ///< Data Length Register
#define SPI_MISC_REG_OFFSET             0x0028  ///< Misc Register
#define SPI_DIN_MODE_REG_OFFSET         0x002C  ///< Input mode Register
#define SPI_DIN_NUM_REG_OFFSET          0x0030  ///< Input delay number
#define SPI_DOUT_MODE_REG_OFFSET        0x0034  ///< Output mode Register
#define SPI_DMA_CONF_REG_OFFSET         0x0038  ///< DMA Configuration
#define SPI_DMA_INT_ENA_REG_OFFSET      0x003C  ///< DMA Interrupt Enable
#define SPI_DMA_INT_RAW_REG_OFFSET      0x0040  ///< DMA Interrupt Raw
#define SPI_DMA_INT_ST_REG_OFFSET       0x0044  ///< DMA Interrupt Status
#define SPI_DMA_INT_CLR_REG_OFFSET      0x0048  ///< DMA Interrupt Clear
#define SPI_W0_REG_OFFSET               0x0098  ///< Data buffer 0

// SPI Command Register Bits (CMD_REG)
#define SPI_CMD_USR_BIT                 (1U << 18)  ///< User defined command

// SPI Control Register Bits (CTRL_REG)
#define SPI_CTRL_WR_BIT_ORDER_BIT       (1U << 26)  ///< Write bit order
#define SPI_CTRL_RD_BIT_ORDER_BIT       (1U << 25)  ///< Read bit order

// SPI Clock Register Configuration
#define SPI_CLOCK_CLK_EQU_SYSCLK_BIT    (1U << 31)  ///< Clock equals system clock
#define SPI_CLOCK_CLKDIV_PRE_SHIFT      16          ///< Clock divider prescaler
#define SPI_CLOCK_CLKCNT_N_SHIFT        12          ///< Clock count N
#define SPI_CLOCK_CLKCNT_H_SHIFT        6           ///< Clock count high
#define SPI_CLOCK_CLKCNT_L_SHIFT        0           ///< Clock count low

// SPI User Register Bits (USER_REG)
#define SPI_USER_DOUTDIN_BIT            (1U << 0)   ///< Full-duplex mode
#define SPI_USER_CK_OUT_EDGE_BIT        (1U << 7)   ///< Clock out edge (CPHA)
#define SPI_USER_CS_SETUP_BIT           (1U << 5)   ///< CS setup
#define SPI_USER_CS_HOLD_BIT            (1U << 6)   ///< CS hold
#define SPI_USER_USR_MISO_BIT           (1U << 27)  ///< MISO enable
#define SPI_USER_USR_MOSI_BIT           (1U << 26)  ///< MOSI enable

// SPI Misc Register Bits
#define SPI_MISC_CK_IDLE_EDGE_BIT       (1U << 29)  ///< Clock idle edge (CPOL)
#define SPI_MISC_CS_KEEP_ACTIVE_BIT     (1U << 30)  ///< CS keep active

// Helper Macros for Register Access
#define SPI_REG_ADDR(base, offset)      ((base) + (offset))
#define SPI_READ_REG(base, offset)      (*(volatile uint32_t*)SPI_REG_ADDR(base, offset))
#define SPI_WRITE_REG(base, offset, val) (*(volatile uint32_t*)SPI_REG_ADDR(base, offset) = (val))
#define SPI_SET_BITS(base, offset, bits) SPI_WRITE_REG(base, offset, SPI_READ_REG(base, offset) | (bits))
#define SPI_CLEAR_BITS(base, offset, bits) SPI_WRITE_REG(base, offset, SPI_READ_REG(base, offset) & ~(bits))

/**
 * @brief BSW SPI Port Type Definition
 */
typedef enum {
    BSW_SPI_PORT_2 = 0,         ///< SPI Port 2 (Base: 0x60003000)
    BSW_SPI_PORT_MAX
} bsw_spi_port_t;

/**
 * @brief SPI Clock Speed Options
 */
typedef enum {
    BSW_SPI_FREQ_1M   = 1000000,    ///< 1MHz
    BSW_SPI_FREQ_5M   = 5000000,    ///< 5MHz
    BSW_SPI_FREQ_10M  = 10000000,   ///< 10MHz
    BSW_SPI_FREQ_20M  = 20000000,   ///< 20MHz
    BSW_SPI_FREQ_40M  = 40000000    ///< 40MHz
} bsw_spi_clock_speed_t;

/**
 * @brief SPI Mode (CPOL/CPHA Configuration)
 * 
 * - Mode 0: CPOL=0, CPHA=0 (Clock idle low, sample on rising edge)
 * - Mode 1: CPOL=0, CPHA=1 (Clock idle low, sample on falling edge)
 * - Mode 2: CPOL=1, CPHA=0 (Clock idle high, sample on falling edge)
 * - Mode 3: CPOL=1, CPHA=1 (Clock idle high, sample on rising edge)
 */
typedef enum {
    BSW_SPI_MODE_0 = 0,         ///< CPOL=0, CPHA=0
    BSW_SPI_MODE_1,             ///< CPOL=0, CPHA=1
    BSW_SPI_MODE_2,             ///< CPOL=1, CPHA=0
    BSW_SPI_MODE_3,             ///< CPOL=1, CPHA=1
    BSW_SPI_MODE_MAX
} bsw_spi_mode_t;

/**
 * @brief SPI Hardware Configuration Structure
 */
typedef struct {
    bsw_gpio_num_t mosi_pin;        ///< MOSI Pin Number
    bsw_gpio_num_t miso_pin;        ///< MISO Pin Number
    bsw_gpio_num_t sclk_pin;        ///< SCLK Pin Number
    bsw_gpio_num_t cs_pin;          ///< CS Pin Number
    bsw_spi_clock_speed_t clock_speed; ///< Clock Speed
    bsw_spi_mode_t mode;            ///< SPI Mode (CPOL/CPHA)
    bool cs_active_high;            ///< CS Active High (default: false)
} spi_hw_config_t;

/**
 * @brief SPI Default Configuration Constants
 */
#define SPI_DEFAULT_CLOCK_SPEED     BSW_SPI_FREQ_1M    ///< Default: 1MHz
#define SPI_DEFAULT_MODE            BSW_SPI_MODE_0     ///< Default: Mode 0
#define SPI_MAX_TRANSFER_SIZE       64                 ///< Maximum transfer size (bytes)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup SPI_DRIVER ESP32-C6 Hardware SPI Controller Driver API
 * @brief ESP32-C6 하드웨어 SPI 컨트롤러 레지스터 직접 제어 인터페이스
 * @{
 */

/**
 * @brief SPI Hardware Controller Initialization
 * 
 * ESP32-C6 SPI 하드웨어 컨트롤러의 레지스터를 직접 설정하여 초기화합니다.
 * GPIO 핀 매핑, 클럭 설정, SPI 모드를 설정합니다.
 * 
 * @param port SPI 포트 번호
 * @param config SPI 하드웨어 설정 구조체
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_ERR_INVALID_ARG: 잘못된 인자
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note SPI 컨트롤러 클럭, 타이밍 파라미터 모두 하드웨어적으로 설정됩니다.
 * @note GPIO는 SPI 기능으로 매핑되며 적절한 모드로 설정됩니다.
 */
esp_err_t bsw_spi_init(bsw_spi_port_t port, const spi_hw_config_t* config);

/**
 * @brief SPI Single Byte Transfer (Full-Duplex)
 * 
 * SPI 하드웨어 컨트롤러를 사용하여 1바이트를 송신하고 동시에 1바이트를 수신합니다.
 * 
 * @param port SPI 포트 번호
 * @param tx_data 송신할 데이터
 * @param rx_data 수신한 데이터를 저장할 포인터 (NULL 가능)
 * @return esp_err_t 
 *         - ESP_OK: 전송 성공
 *         - ESP_ERR_TIMEOUT: 타임아웃
 *         - ESP_FAIL: 전송 실패
 * 
 * @note CS는 자동으로 제어됩니다 (전송 시작 시 LOW, 종료 시 HIGH)
 */
esp_err_t bsw_spi_transfer_byte(bsw_spi_port_t port, uint8_t tx_data, uint8_t* rx_data);

/**
 * @brief SPI Block Transfer (Full-Duplex)
 * 
 * SPI 하드웨어 컨트롤러를 사용하여 여러 바이트를 송수신합니다.
 * 
 * @param port SPI 포트 번호
 * @param tx_buffer 송신 데이터 버퍼 (NULL 가능, 수신만 할 경우)
 * @param rx_buffer 수신 데이터 버퍼 (NULL 가능, 송신만 할 경우)
 * @param length 전송할 데이터 길이 (바이트)
 * @return esp_err_t 
 *         - ESP_OK: 전송 성공
 *         - ESP_ERR_INVALID_ARG: 잘못된 인자
 *         - ESP_ERR_TIMEOUT: 타임아웃
 *         - ESP_FAIL: 전송 실패
 * 
 * @note CS는 자동으로 제어됩니다
 * @warning length는 SPI_MAX_TRANSFER_SIZE 이하여야 함
 */
esp_err_t bsw_spi_transfer_block(bsw_spi_port_t port, const uint8_t* tx_buffer, 
                                 uint8_t* rx_buffer, size_t length);

/**
 * @brief SPI Chip Select Assert (CS Low)
 * 
 * SPI CS 핀을 수동으로 LOW로 설정합니다.
 * 여러 번의 전송을 하나의 트랜잭션으로 묶을 때 사용합니다.
 * 
 * @param port SPI 포트 번호
 * @return esp_err_t 
 *         - ESP_OK: 성공
 *         - ESP_FAIL: 실패
 * 
 * @note 사용 후 반드시 bsw_spi_cs_deselect()를 호출해야 합니다
 */
esp_err_t bsw_spi_cs_select(bsw_spi_port_t port);

/**
 * @brief SPI Chip Select Deassert (CS High)
 * 
 * SPI CS 핀을 수동으로 HIGH로 설정합니다.
 * 
 * @param port SPI 포트 번호
 * @return esp_err_t 
 *         - ESP_OK: 성공
 *         - ESP_FAIL: 실패
 */
esp_err_t bsw_spi_cs_deselect(bsw_spi_port_t port);

/**
 * @brief SPI 드라이버 해제
 * 
 * SPI 관련 자원을 해제하고 GPIO 핀을 원래 상태로 복원합니다.
 * 
 * @param port SPI 포트 번호
 * @return esp_err_t 
 *         - ESP_OK: 해제 성공
 *         - ESP_FAIL: 해제 실패
 */
esp_err_t bsw_spi_deinit(bsw_spi_port_t port);

/** @} */ // SPI_DRIVER

#ifdef __cplusplus
}
#endif

#endif // SPI_DRIVER_H
