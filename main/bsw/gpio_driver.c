/**
 * @file gpio_driver.c
 * @brief BSW GPIO 드라이버 구현 파일
 * 
 * GPIO 하드웨어 추상화 계층 구현으로 ESP-IDF GPIO API를 래핑하여
 * 플랫폼 독립적인 GPIO 제어 인터페이스를 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-24
 * @version 1.0
 */

#include "gpio_driver.h"
#include "system_services.h"

static const char* TAG = "BSW_GPIO";

/**
 * @brief GPIO 설정
 * 
 * @param pGPIOConfig GPIO 설정 구조체 포인터
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_config(const bsw_gpio_config_t* pGPIOConfig) {
    if (!pGPIOConfig) {
        BSW_LOGE(TAG, "GPIO config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t config = {
        .pin_bit_mask = pGPIOConfig->pin_bit_mask,
        .mode = (gpio_mode_t)pGPIOConfig->mode,
        .pull_up_en = (gpio_pullup_t)pGPIOConfig->pull_up_en,
        .pull_down_en = (gpio_pulldown_t)pGPIOConfig->pull_down_en,
        .intr_type = (gpio_int_type_t)pGPIOConfig->intr_type
    };

    esp_err_t ret = gpio_config(&config);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to configure GPIO");
    }

    return ret;
}

/**
 * @brief GPIO 레벨 읽기
 * 
 * @param gpio_num GPIO 핀 번호
 * @return GPIO 레벨 (0 또는 1)
 */
int bsw_gpio_get_level(bsw_gpio_num_t gpio_num) {
    return gpio_get_level(gpio_num);
}

/**
 * @brief GPIO 레벨 설정
 * 
 * @param gpio_num GPIO 핀 번호
 * @param level 설정할 레벨 (0 또는 1)
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_set_level(bsw_gpio_num_t gpio_num, uint32_t level) {
    return gpio_set_level(gpio_num, level);
}

/**
 * @brief GPIO ISR 서비스 설치
 * 
 * @param intr_alloc_flags 인터럽트 할당 플래그
 * @return ESP_OK 성공, ESP_ERR_INVALID_STATE 이미 설치됨, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_install_isr_service(int intr_alloc_flags) {
    return gpio_install_isr_service(intr_alloc_flags);
}

/**
 * @brief GPIO ISR 핸들러 추가
 * 
 * @param gpio_num GPIO 핀 번호
 * @param isr_handler ISR 핸들러 함수
 * @param args 핸들러에 전달할 인수
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_isr_handler_add(bsw_gpio_num_t gpio_num, bsw_gpio_isr_t isr_handler, void* args) {
    return gpio_isr_handler_add(gpio_num, (gpio_isr_t)isr_handler, args);
}

/**
 * @brief GPIO ISR 핸들러 제거
 * 
 * @param gpio_num GPIO 핀 번호
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_isr_handler_remove(bsw_gpio_num_t gpio_num) {
    return gpio_isr_handler_remove(gpio_num);
}