/**
 * @file adc_driver.h
 * @brief ESP32-C6 Hardware SAR ADC Direct Register Control Driver
 * 
 * ESP32-C6 SAR ADC 하드웨어 컨트롤러의 레지스터를 직접 제어하는 드라이버입니다.
 * 아날로그 센서 입력을 디지털 값으로 변환하며, 하드웨어 기반으로 CPU 부하를 최소화합니다.
 * 
 * 주요 기능:
 * - ESP32-C6 SAR ADC 컨트롤러 레지스터 직접 제어
 * - 12비트 해상도 ADC 변환 (0-4095)
 * - 다양한 감쇠 레벨 지원 (0dB, 2.5dB, 6dB, 11dB)
 * - 전압 범위: 0-3.3V (11dB 감쇠 시)
 * - 하드웨어 기반 변환으로 CPU 부하 최소화
 * - Oneshot 모드 지원 (필요 시 변환)
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 1.0 (Hardware SAR ADC Controller)
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "gpio_driver.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief ESP32-C6 SAR ADC Hardware Controller Register Definitions
 * ESP32-C6 TRM Chapter 30: SAR ADC
 */

// ADC Controller Base Addresses (ESP32-C6)
#define ADC_BASE_ADDR           0x60040000UL    ///< SAR ADC Base Address

// ADC Register Offsets (ESP32-C6 TRM Chapter 30)
#define APB_SARADC_CTRL_REG_OFFSET          0x0000  ///< ADC Control Register
#define APB_SARADC_CTRL2_REG_OFFSET         0x0004  ///< ADC Control Register 2
#define APB_SARADC_FILTER_CTRL1_REG_OFFSET  0x0008  ///< ADC Filter Control 1
#define APB_SARADC_FSM_WAIT_REG_OFFSET      0x000C  ///< ADC FSM Wait Register
#define APB_SARADC_SAR1_STATUS_REG_OFFSET   0x0010  ///< ADC1 Status Register
#define APB_SARADC_SAR2_STATUS_REG_OFFSET   0x0014  ///< ADC2 Status Register
#define APB_SARADC_SAR1_PATT_TAB1_REG_OFFSET 0x0018 ///< ADC1 Pattern Table 1
#define APB_SARADC_SAR2_PATT_TAB1_REG_OFFSET 0x001C ///< ADC2 Pattern Table 1
#define APB_SARADC_ONETIME_SAMPLE_REG_OFFSET 0x0020 ///< ADC Oneshot Sample Register
#define APB_SARADC_ARB_CTRL_REG_OFFSET      0x0024  ///< ADC Arbiter Control
#define APB_SARADC_FILTER_CTRL0_REG_OFFSET  0x0028  ///< ADC Filter Control 0
#define APB_SARADC_SAR1_DATA_STATUS_REG_OFFSET 0x002C ///< ADC1 Data Status
#define APB_SARADC_SAR2_DATA_STATUS_REG_OFFSET 0x0030 ///< ADC2 Data Status
#define APB_SARADC_THRES0_CTRL_REG_OFFSET   0x0034  ///< ADC Threshold 0 Control
#define APB_SARADC_THRES1_CTRL_REG_OFFSET   0x0038  ///< ADC Threshold 1 Control
#define APB_SARADC_THRES_CTRL_REG_OFFSET    0x003C  ///< ADC Threshold Control
#define APB_SARADC_INT_ENA_REG_OFFSET       0x0040  ///< ADC Interrupt Enable
#define APB_SARADC_INT_RAW_REG_OFFSET       0x0044  ///< ADC Interrupt Raw
#define APB_SARADC_INT_ST_REG_OFFSET        0x0048  ///< ADC Interrupt Status
#define APB_SARADC_INT_CLR_REG_OFFSET       0x004C  ///< ADC Interrupt Clear
#define APB_SARADC_DMA_CONF_REG_OFFSET      0x0050  ///< ADC DMA Configuration
#define APB_SARADC_CLKM_CONF_REG_OFFSET     0x0054  ///< ADC Clock Configuration
#define APB_SARADC_APB_ADC_CLKM_CONF_REG_OFFSET 0x0058 ///< APB ADC Clock Config

// ADC Control Register Bits (CTRL_REG)
#define ADC_CTRL_START_FORCE_BIT            (1U << 0)   ///< Force start ADC
#define ADC_CTRL_START_BIT                  (1U << 1)   ///< Start ADC
#define ADC_CTRL_SAR_CLK_GATED_BIT          (1U << 6)   ///< SAR Clock Gated
#define ADC_CTRL_XPD_SAR_FORCE_BIT          (3U << 27)  ///< Power up SAR

// ADC Oneshot Sample Register Bits
#define ADC_ONETIME_START_BIT               (1U << 31)  ///< Start oneshot conversion
#define ADC_ONETIME_CHANNEL_SHIFT           24          ///< Channel selection shift
#define ADC_ONETIME_ATTEN_SHIFT             16          ///< Attenuation selection shift

// ADC Status Register Bits
#define ADC_SAR1_STATUS_SHIFT               0           ///< ADC1 Status data shift

// Helper Macros for Register Access
#define ADC_REG_ADDR(offset)                ((ADC_BASE_ADDR) + (offset))
#define ADC_READ_REG(offset)                (*(volatile uint32_t*)ADC_REG_ADDR(offset))
#define ADC_WRITE_REG(offset, val)          (*(volatile uint32_t*)ADC_REG_ADDR(offset) = (val))
#define ADC_SET_BITS(offset, bits)          ADC_WRITE_REG(offset, ADC_READ_REG(offset) | (bits))
#define ADC_CLEAR_BITS(offset, bits)        ADC_WRITE_REG(offset, ADC_READ_REG(offset) & ~(bits))

/**
 * @brief BSW ADC Unit Type Definition
 */
typedef enum {
    BSW_ADC_UNIT_1 = 0,         ///< ADC Unit 1
    BSW_ADC_UNIT_2,             ///< ADC Unit 2
    BSW_ADC_UNIT_MAX
} bsw_adc_unit_t;

/**
 * @brief ADC Channel Numbers
 */
typedef enum {
    BSW_ADC_CHANNEL_0 = 0,      ///< ADC Channel 0
    BSW_ADC_CHANNEL_1,          ///< ADC Channel 1
    BSW_ADC_CHANNEL_2,          ///< ADC Channel 2
    BSW_ADC_CHANNEL_3,          ///< ADC Channel 3
    BSW_ADC_CHANNEL_4,          ///< ADC Channel 4
    BSW_ADC_CHANNEL_5,          ///< ADC Channel 5
    BSW_ADC_CHANNEL_6,          ///< ADC Channel 6
    BSW_ADC_CHANNEL_MAX
} bsw_adc_channel_t;

/**
 * @brief ADC Attenuation Levels
 * 
 * Different attenuation levels change the input voltage range:
 * - 0dB: ~800mV
 * - 2.5dB: ~1100mV
 * - 6dB: ~1350mV
 * - 11dB: ~3300mV (full range)
 */
typedef enum {
    BSW_ADC_ATTEN_DB_0 = 0,     ///< No attenuation (~800mV max)
    BSW_ADC_ATTEN_DB_2_5,       ///< 2.5dB attenuation (~1100mV max)
    BSW_ADC_ATTEN_DB_6,         ///< 6dB attenuation (~1350mV max)
    BSW_ADC_ATTEN_DB_11,        ///< 11dB attenuation (~3300mV max)
    BSW_ADC_ATTEN_MAX
} bsw_adc_atten_t;

/**
 * @brief ADC Resolution
 */
#define ADC_MAX_RAW_VALUE       4095        ///< 12-bit ADC (0-4095)
#define ADC_VREF_MV             3300        ///< Reference voltage in mV

/**
 * @brief ADC Channel Configuration Structure
 */
typedef struct {
    bsw_adc_unit_t unit;            ///< ADC unit (1 or 2)
    bsw_adc_channel_t channel;      ///< ADC channel number
    bsw_adc_atten_t attenuation;    ///< Attenuation level
    bsw_gpio_num_t gpio_pin;        ///< GPIO pin number
} adc_channel_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup ADC_DRIVER ESP32-C6 Hardware SAR ADC Controller Driver API
 * @brief ESP32-C6 하드웨어 SAR ADC 컨트롤러 레지스터 직접 제어 인터페이스
 * @{
 */

/**
 * @brief ADC Hardware Controller Initialization
 * 
 * ESP32-C6 SAR ADC 하드웨어 컨트롤러의 레지스터를 직접 설정하여 초기화합니다.
 * ADC 클럭, 파워, 타이밍 파라미터를 설정합니다.
 * 
 * @return esp_err_t 
 *         - ESP_OK: 초기화 성공
 *         - ESP_FAIL: 초기화 실패
 * 
 * @note ADC 컨트롤러 클럭, 파워 관리 모두 하드웨어적으로 설정됩니다.
 */
esp_err_t bsw_adc_init(void);

/**
 * @brief ADC Channel Configuration
 * 
 * 특정 ADC 채널을 설정하고 해당 GPIO 핀을 ADC 기능으로 매핑합니다.
 * 
 * @param config ADC 채널 설정 구조체
 * @return esp_err_t 
 *         - ESP_OK: 설정 성공
 *         - ESP_ERR_INVALID_ARG: 잘못된 인자
 *         - ESP_FAIL: 설정 실패
 * 
 * @note GPIO 핀은 자동으로 아날로그 입력 모드로 설정됩니다.
 */
esp_err_t bsw_adc_config_channel(const adc_channel_config_t* config);

/**
 * @brief ADC Raw Value Read (Oneshot Mode)
 * 
 * 설정된 ADC 채널에서 단일 변환을 수행하고 raw 값을 읽습니다.
 * 
 * @param unit ADC 유닛 번호
 * @param channel ADC 채널 번호
 * @param raw_value 읽은 raw 값을 저장할 포인터 (0-4095)
 * @return esp_err_t 
 *         - ESP_OK: 읽기 성공
 *         - ESP_ERR_INVALID_ARG: 잘못된 인자
 *         - ESP_ERR_TIMEOUT: 변환 타임아웃
 *         - ESP_FAIL: 읽기 실패
 * 
 * @note 하드웨어 컨트롤러가 변환을 수행하므로 CPU 부하가 낮습니다.
 * @warning 변환 완료까지 대기하므로 블로킹 함수입니다.
 */
esp_err_t bsw_adc_get_raw(bsw_adc_unit_t unit, bsw_adc_channel_t channel, uint32_t* raw_value);

/**
 * @brief ADC Raw Value to Voltage Conversion
 * 
 * ADC raw 값을 실제 전압(mV)으로 변환합니다.
 * 감쇠 레벨에 따라 적절한 전압 범위로 계산됩니다.
 * 
 * @param raw_value ADC raw 값 (0-4095)
 * @param attenuation 감쇠 레벨
 * @param voltage_mv 변환된 전압(mV)을 저장할 포인터
 * @return esp_err_t 
 *         - ESP_OK: 변환 성공
 *         - ESP_ERR_INVALID_ARG: 잘못된 인자
 * 
 * @note 선형 변환을 사용합니다. 더 정확한 변환은 calibration이 필요합니다.
 */
esp_err_t bsw_adc_raw_to_voltage(uint32_t raw_value, bsw_adc_atten_t attenuation, uint32_t* voltage_mv);

/**
 * @brief ADC 드라이버 해제
 * 
 * ADC 관련 자원을 해제하고 GPIO 핀을 원래 상태로 복원합니다.
 * 
 * @return esp_err_t 
 *         - ESP_OK: 해제 성공
 *         - ESP_FAIL: 해제 실패
 */
esp_err_t bsw_adc_deinit(void);

/** @} */ // ADC_DRIVER

#ifdef __cplusplus
}
#endif

#endif // ADC_DRIVER_H
