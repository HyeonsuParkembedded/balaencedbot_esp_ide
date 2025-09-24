/**
 * @file gpio_driver.h
 * @brief BSW GPIO 드라이버 헤더 파일
 * 
 * GPIO 하드웨어 추상화 계층을 제공하여 플랫폼 독립적인 GPIO 제어를 가능하게 합니다.
 * ESP-IDF의 GPIO API를 BSW 인터페이스로 래핑합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 1.0
 */

#ifndef BSW_GPIO_DRIVER_H
#define BSW_GPIO_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSW GPIO 핀 번호 타입
 */
typedef gpio_num_t bsw_gpio_num_t;

/**
 * @brief BSW GPIO 모드 열거형
 */
typedef enum {
    BSW_GPIO_MODE_DISABLE = GPIO_MODE_DISABLE,     ///< GPIO 비활성화
    BSW_GPIO_MODE_INPUT = GPIO_MODE_INPUT,         ///< 입력 모드
    BSW_GPIO_MODE_OUTPUT = GPIO_MODE_OUTPUT,       ///< 출력 모드
    BSW_GPIO_MODE_OUTPUT_OD = GPIO_MODE_OUTPUT_OD, ///< 오픈 드레인 출력
    BSW_GPIO_MODE_INPUT_OUTPUT_OD = GPIO_MODE_INPUT_OUTPUT_OD, ///< 입출력 오픈 드레인
    BSW_GPIO_MODE_INPUT_OUTPUT = GPIO_MODE_INPUT_OUTPUT ///< 입출력 모드
} bsw_gpio_mode_t;

/**
 * @brief BSW GPIO 풀업/풀다운 열거형
 */
typedef enum {
    BSW_GPIO_PULLUP_DISABLE = GPIO_PULLUP_DISABLE, ///< 풀업 비활성화
    BSW_GPIO_PULLUP_ENABLE = GPIO_PULLUP_ENABLE,   ///< 풀업 활성화
    BSW_GPIO_PULLDOWN_DISABLE = GPIO_PULLDOWN_DISABLE, ///< 풀다운 비활성화
    BSW_GPIO_PULLDOWN_ENABLE = GPIO_PULLDOWN_ENABLE ///< 풀다운 활성화
} bsw_gpio_pull_mode_t;

/**
 * @brief BSW GPIO 인터럽트 타입 열거형
 */
typedef enum {
    BSW_GPIO_INTR_DISABLE = GPIO_INTR_DISABLE,     ///< 인터럽트 비활성화
    BSW_GPIO_INTR_POSEDGE = GPIO_INTR_POSEDGE,     ///< 상승 엣지 인터럽트
    BSW_GPIO_INTR_NEGEDGE = GPIO_INTR_NEGEDGE,     ///< 하강 엣지 인터럽트
    BSW_GPIO_INTR_ANYEDGE = GPIO_INTR_ANYEDGE,     ///< 양쪽 엣지 인터럽트
    BSW_GPIO_INTR_LOW_LEVEL = GPIO_INTR_LOW_LEVEL, ///< 로우 레벨 인터럽트
    BSW_GPIO_INTR_HIGH_LEVEL = GPIO_INTR_HIGH_LEVEL ///< 하이 레벨 인터럽트
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
 * @brief BSW 인터럽트 플래그
 */
#define BSW_INTR_FLAG_IRAM ESP_INTR_FLAG_IRAM

/**
 * @brief BSW GPIO ISR 핸들러 타입
 */
typedef void (*bsw_gpio_isr_t)(void* arg);

// BSW GPIO 함수 선언
esp_err_t bsw_gpio_config(const bsw_gpio_config_t* pGPIOConfig);
int bsw_gpio_get_level(bsw_gpio_num_t gpio_num);
esp_err_t bsw_gpio_set_level(bsw_gpio_num_t gpio_num, uint32_t level);
esp_err_t bsw_gpio_install_isr_service(int intr_alloc_flags);
esp_err_t bsw_gpio_isr_handler_add(bsw_gpio_num_t gpio_num, bsw_gpio_isr_t isr_handler, void* args);
esp_err_t bsw_gpio_isr_handler_remove(bsw_gpio_num_t gpio_num);

#ifdef __cplusplus
}
#endif

#endif // BSW_GPIO_DRIVER_H