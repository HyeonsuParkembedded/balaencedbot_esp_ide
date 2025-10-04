/**
 * @file gpio_driver.h
 * @brief BSW GPIO 드라이버 헤더 파일
 * 
 * GPIO 하드웨어 레지스터 직접 제어를 통한 고성능 GPIO 드라이버입니다.
 * ESP32-C6의 GPIO 레지스터를 직접 조작하여 빠른 속도와 정밀한 제어를 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 2.0
 */

#ifndef BSW_GPIO_DRIVER_H
#define BSW_GPIO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSW GPIO 핀 번호 타입
 */
typedef uint8_t bsw_gpio_num_t;

/**
 * @brief BSW GPIO 모드 열거형 (TRM 기반 오픈 드레인 모드 추가)
 */
typedef enum {
    BSW_GPIO_MODE_DISABLE = 0,     ///< GPIO 비활성화
    BSW_GPIO_MODE_INPUT = 1,       ///< 입력 모드
    BSW_GPIO_MODE_OUTPUT = 2,      ///< 출력 모드 (Push-Pull)
    BSW_GPIO_MODE_OUTPUT_OD = 3,   ///< 오픈 드레인 출력 (Open Drain)
    BSW_GPIO_MODE_INPUT_OUTPUT_OD = 4, ///< 입출력 오픈 드레인
    BSW_GPIO_MODE_INPUT_OUTPUT = 5 ///< 입출력 모드
} bsw_gpio_mode_t;

/**
 * @brief BSW GPIO 풀업/풀다운 열거형
 */
typedef enum {
    BSW_GPIO_PULLUP_DISABLE = 0,   ///< 풀업 비활성화
    BSW_GPIO_PULLUP_ENABLE = 1,    ///< 풀업 활성화
    BSW_GPIO_PULLDOWN_DISABLE = 0, ///< 풀다운 비활성화
    BSW_GPIO_PULLDOWN_ENABLE = 1   ///< 풀다운 활성화
} bsw_gpio_pull_mode_t;

/**
 * @brief BSW GPIO 인터럽트 타입 열거형
 */
typedef enum {
    BSW_GPIO_INTR_DISABLE = 0,     ///< 인터럽트 비활성화
    BSW_GPIO_INTR_POSEDGE = 1,     ///< 상승 엣지 인터럽트
    BSW_GPIO_INTR_NEGEDGE = 2,     ///< 하강 엣지 인터럽트
    BSW_GPIO_INTR_ANYEDGE = 3,     ///< 양쪽 엣지 인터럽트
    BSW_GPIO_INTR_LOW_LEVEL = 4,   ///< 로우 레벨 인터럽트
    BSW_GPIO_INTR_HIGH_LEVEL = 5   ///< 하이 레벨 인터럽트
} bsw_gpio_int_type_t;

/**
 * @brief BSW GPIO 설정 구조체
 */
typedef struct {
    uint64_t pin_bit_mask;          ///< GPIO 핀 비트마스크
    bsw_gpio_mode_t mode;           ///< GPIO 모드
    bsw_gpio_pull_mode_t pull_up_en;   ///< 풀업 활성화 여부
    bsw_gpio_pull_mode_t pull_down_en; ///< 풀다운 활성화 여부
    bsw_gpio_int_type_t intr_type;     ///< 인터럽트 타입
} bsw_gpio_config_t;

/**
 * @brief ESP32-C6 GPIO 레지스터 직접 제어 매크로 (TRM 기반)
 */
#ifndef GPIO_PIN_COUNT
#define GPIO_PIN_COUNT 31  ///< ESP32-C6 GPIO 핀 수 (0-30)
#endif

// 하드웨어 레지스터 직접 주소 (ESP32-C6 Technical Reference Manual 기준)
// ESP32-C6 GPIO 베이스 주소: 0x60004000 (TRM Chapter 6)
#define GPIO_BASE_ADDR      0x60004000UL
#define IO_MUX_BASE         0x60090000UL

// BSW GPIO 레지스터 오프셋 주소들 (ESP32-C6 TRM 정확한 오프셋)
#define BSW_GPIO_OUT_REG        (GPIO_BASE_ADDR + 0x0004)    ///< GPIO 출력 레지스터
#define BSW_GPIO_OUT_W1TS_REG   (GPIO_BASE_ADDR + 0x0008)    ///< GPIO 출력 set 레지스터 (W1TS: Write 1 to Set)
#define BSW_GPIO_OUT_W1TC_REG   (GPIO_BASE_ADDR + 0x000C)    ///< GPIO 출력 clear 레지스터 (W1TC: Write 1 to Clear)
#define BSW_GPIO_ENABLE_REG     (GPIO_BASE_ADDR + 0x0020)    ///< GPIO 출력 활성화 레지스터
#define BSW_GPIO_ENABLE_W1TS_REG (GPIO_BASE_ADDR + 0x0024)   ///< GPIO 출력 활성화 set 레지스터
#define BSW_GPIO_ENABLE_W1TC_REG (GPIO_BASE_ADDR + 0x0028)   ///< GPIO 출력 활성화 clear 레지스터
#define BSW_GPIO_IN_REG         (GPIO_BASE_ADDR + 0x003C)    ///< GPIO 입력 레지스터
#define BSW_GPIO_PIN_CONFIG_REG_BASE_OFFSET 0x0074           ///< GPIO 핀 설정 레지스터 베이스 오프셋

// 핀(n)에 대한 설정 레지스터 주소 계산 매크로 (TRM 기반)
#define GPIO_PIN_N_REG(n) (GPIO_BASE_ADDR + BSW_GPIO_PIN_CONFIG_REG_BASE_OFFSET + ((n) * 4))

// 핀 설정 비트 필드 정의 (ESP32-C6 TRM 기준)
#define GPIO_PIN_PAD_DRIVER_BIT   (1U << 2)  ///< 오픈 드레인 제어 (비트 2)
#define GPIO_PIN_PULLUP_BIT       (1U << 7)  ///< 풀업 활성화 (비트 7)
#define GPIO_PIN_PULLDOWN_BIT     (1U << 8)  ///< 풀다운 활성화 (비트 8)

/**
 * @note ESP-IDF의 REG_WRITE(), REG_READ() 매크로를 사용합니다.
 *       이 매크로들은 soc/soc.h에 정의되어 있으며, soc/gpio_reg.h를 
 *       include하면 자동으로 포함됩니다.
 *       충돌 방지를 위해 재정의하지 않습니다.
 */

// 순수 비트연산자 기반 레지스터 직접 조작 매크로
#define BSW_WRITE_REG_BITS(addr, mask, value) do { \
    uint32_t temp = (REG_READ(addr) & ~(mask)) | ((value) & (mask)); \
    REG_WRITE(addr, temp); \
} while(0)

#define BSW_SET_REG_BITS(addr, mask) do { \
    REG_WRITE(addr, REG_READ(addr) | (mask)); \
} while(0)

#define BSW_CLEAR_REG_BITS(addr, mask) do { \
    REG_WRITE(addr, REG_READ(addr) & ~(mask)); \
} while(0)

#define BSW_READ_REG_BITS(addr, mask) (REG_READ(addr) & (mask))

// 순수 비트연산 GPIO 매크로 (가장 빠른 방법)
#define GPIO_BIT_SET(gpio_num) do { \
    *(volatile uint32_t*)BSW_GPIO_OUT_W1TS_REG = (1U << (gpio_num)); \
} while(0)

#define GPIO_BIT_CLEAR(gpio_num) do { \
    *(volatile uint32_t*)BSW_GPIO_OUT_W1TC_REG = (1U << (gpio_num)); \
} while(0)

#define GPIO_BIT_READ(gpio_num) \
    ((*(volatile uint32_t*)BSW_GPIO_IN_REG >> (gpio_num)) & 1U)

#define GPIO_OUTPUT_ENABLE(gpio_num) do { \
    *(volatile uint32_t*)BSW_GPIO_ENABLE_W1TS_REG = (1U << (gpio_num)); \
} while(0)

#define GPIO_OUTPUT_DISABLE(gpio_num) do { \
    *(volatile uint32_t*)BSW_GPIO_ENABLE_W1TC_REG = (1U << (gpio_num)); \
} while(0)

// 기존 GPIO 레지스터 직접 액세스 매크로 (호환성 유지)
#define GPIO_OUT_SET(pin_mask)      (GPIO.out_w1ts.val = (pin_mask))
#define GPIO_OUT_CLEAR(pin_mask)    (GPIO.out_w1tc.val = (pin_mask))
#define GPIO_OUT_TOGGLE(pin_mask)   (GPIO.out.val ^= (pin_mask))
#define GPIO_IN_READ()              (GPIO.in.val)
#define GPIO_ENABLE_SET(pin_mask)   (GPIO.enable_w1ts.val = (pin_mask))
#define GPIO_ENABLE_CLEAR(pin_mask) (GPIO.enable_w1tc.val = (pin_mask))

// 고속 GPIO 제어 인라인 함수들 (기존 방식 - 호환성)
static inline void bsw_gpio_set_high(bsw_gpio_num_t gpio_num) {
    GPIO_OUT_SET(1ULL << gpio_num);
}

static inline void bsw_gpio_set_low(bsw_gpio_num_t gpio_num) {
    GPIO_OUT_CLEAR(1ULL << gpio_num);
}

static inline void bsw_gpio_toggle(bsw_gpio_num_t gpio_num) {
    GPIO_OUT_TOGGLE(1ULL << gpio_num);
}

static inline uint32_t bsw_gpio_read_input(bsw_gpio_num_t gpio_num) {
    return (GPIO_IN_READ() >> gpio_num) & 0x1;
}

static inline void bsw_gpio_enable_output(bsw_gpio_num_t gpio_num) {
    GPIO_ENABLE_SET(1ULL << gpio_num);
}

static inline void bsw_gpio_disable_output(bsw_gpio_num_t gpio_num) {
    GPIO_ENABLE_CLEAR(1ULL << gpio_num);
}

// 초고속 비트연산 GPIO 제어 함수들 (순수 비트연산 방식)
static inline void bsw_gpio_fast_set_high(bsw_gpio_num_t gpio_num) {
    GPIO_BIT_SET(gpio_num);
}

static inline void bsw_gpio_fast_set_low(bsw_gpio_num_t gpio_num) {
    GPIO_BIT_CLEAR(gpio_num);
}

static inline uint32_t bsw_gpio_fast_read_input(bsw_gpio_num_t gpio_num) {
    return GPIO_BIT_READ(gpio_num);
}

static inline void bsw_gpio_fast_enable_output(bsw_gpio_num_t gpio_num) {
    GPIO_OUTPUT_ENABLE(gpio_num);
}

static inline void bsw_gpio_fast_disable_output(bsw_gpio_num_t gpio_num) {
    GPIO_OUTPUT_DISABLE(gpio_num);
}

// 멀티 핀 비트연산 제어 함수들
static inline void bsw_gpio_multi_set_high(uint32_t pin_mask) {
    *(volatile uint32_t*)BSW_GPIO_OUT_W1TS_REG = pin_mask;
}

static inline void bsw_gpio_multi_set_low(uint32_t pin_mask) {
    *(volatile uint32_t*)BSW_GPIO_OUT_W1TC_REG = pin_mask;
}

static inline uint32_t bsw_gpio_multi_read_input(uint32_t pin_mask) {
    return (*(volatile uint32_t*)BSW_GPIO_IN_REG) & pin_mask;
}

// I2C에서 사용하는 GPIO 입출력 모드 설정 함수들
static inline void bsw_gpio_fast_set_input(bsw_gpio_num_t gpio_num) {
    GPIO_OUTPUT_DISABLE(gpio_num);
}

static inline void bsw_gpio_fast_set_output(bsw_gpio_num_t gpio_num) {
    GPIO_OUTPUT_ENABLE(gpio_num);
}

/**
 * @brief BSW 인터럽트 플래그 정의
 */
#define BSW_INTR_FLAG_IRAM          (1<<1)  ///< IRAM에 ISR 배치
#define BSW_INTR_FLAG_LEVEL         (1<<2)  ///< 레벨 트리거
#define BSW_INTR_FLAG_EDGE          (1<<3)  ///< 엣지 트리거
#define BSW_INTR_FLAG_DISABLED      0       ///< 인터럽트 비활성화

/**
 * @brief BSW GPIO ISR 핸들러 타입
 */
typedef void (*bsw_gpio_isr_t)(void* arg);

// BSW GPIO 함수 선언 - 직접 레지스터 제어 방식
esp_err_t bsw_gpio_init(void);
esp_err_t bsw_gpio_config_pin(bsw_gpio_num_t gpio_num, bsw_gpio_mode_t mode, 
                              bsw_gpio_pull_mode_t pull_up, bsw_gpio_pull_mode_t pull_down);
esp_err_t bsw_gpio_config(const bsw_gpio_config_t* pGPIOConfig);
int bsw_gpio_get_level(bsw_gpio_num_t gpio_num);
esp_err_t bsw_gpio_set_level(bsw_gpio_num_t gpio_num, uint32_t level);
esp_err_t bsw_gpio_set_direction(bsw_gpio_num_t gpio_num, bsw_gpio_mode_t mode);
esp_err_t bsw_gpio_set_pull_mode(bsw_gpio_num_t gpio_num, bsw_gpio_pull_mode_t pull_up, bsw_gpio_pull_mode_t pull_down);

// 순수 비트연산 기반 레지스터 직접 제어 함수들
void bsw_gpio_raw_write_reg(uint32_t reg_addr, uint32_t value);
uint32_t bsw_gpio_raw_read_reg(uint32_t reg_addr);
void bsw_gpio_raw_set_bits(uint32_t reg_addr, uint32_t bit_mask);
void bsw_gpio_raw_clear_bits(uint32_t reg_addr, uint32_t bit_mask);
void bsw_gpio_raw_toggle_bits(uint32_t reg_addr, uint32_t bit_mask);

// IO_MUX 직접 비트연산 제어 함수들
void bsw_gpio_configure_iomux(bsw_gpio_num_t gpio_num, uint32_t func_sel, bool pullup, bool pulldown);
void bsw_gpio_set_drive_strength(bsw_gpio_num_t gpio_num, uint8_t strength);
void bsw_gpio_set_slew_rate(bsw_gpio_num_t gpio_num, bool fast_slew);

// BSW GPIO 인터럽트 처리 함수들 (순수 비트연산 기반)
esp_err_t bsw_gpio_install_isr_service(int intr_alloc_flags);
esp_err_t bsw_gpio_isr_handler_add(bsw_gpio_num_t gpio_num, bsw_gpio_isr_t isr_handler, void* args);
esp_err_t bsw_gpio_isr_handler_remove(bsw_gpio_num_t gpio_num);
void bsw_gpio_uninstall_isr_service(void);
void bsw_gpio_poll_isr(bsw_gpio_num_t gpio_num);  // 폴링 기반 ISR 시뮬레이션

#ifdef __cplusplus
}
#endif

#endif // BSW_GPIO_DRIVER_H