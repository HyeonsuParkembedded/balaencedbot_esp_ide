/**
 * @file bitwise_gpio_example.c
 * @brief 순수 비트연산자를 사용한 ESP32-C6 GPIO 직접 제어 예제
 * 
 * HAL 드라이버 없이 순수 비트연산과 레지스터 직접 조작을 통해 
 * GPIO를 제어하는 방법을 보여주는 예제 코드입니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 1.0
 */

#include "bitwise_gpio_example.h"
#include "gpio_driver.h"
#include "system_services.h"
#include "esp_log.h"
#include "system_services.h"
#include <stdio.h>

static const char* TAG = "BITWISE_GPIO_EXAMPLE";

/**
 * @brief 순수 비트연산을 사용한 GPIO 제어 데모
 */
void demo_bitwise_gpio_control(void) {
    BSW_LOGI(TAG, "=== Pure Bitwise GPIO Control Demo Start ===");
    
    // 1. 레지스터 직접 주소 사용 방법
    BSW_LOGI(TAG, "1. 레지스터 직접 주소 접근:");
    
    // GPIO 2를 출력으로 설정 (LED 제어용)
    uint8_t led_gpio = 2;
    
    // 출력 활성화 - 비트연산으로 직접 제어
    *(volatile uint32_t*)BSW_GPIO_ENABLE_W1TS_REG = (1U << led_gpio);
    BSW_LOGI(TAG, "GPIO %d 출력 활성화 완료", led_gpio);
    
    // GPIO를 HIGH로 설정 - 순수 비트연산
    *(volatile uint32_t*)BSW_GPIO_OUT_W1TS_REG = (1U << led_gpio);
    BSW_LOGI(TAG, "GPIO %d HIGH 설정", led_gpio);
    
    // 잠시 대기
    bsw_delay_ms(1000);
    
    // GPIO를 LOW로 설정 - 순수 비트연산
    *(volatile uint32_t*)BSW_GPIO_OUT_W1TC_REG = (1U << led_gpio);
    BSW_LOGI(TAG, "GPIO %d LOW 설정", led_gpio);
    
    // 2. 매크로를 사용한 빠른 제어
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "2. 매크로 기반 고속 제어:");
    
    for (int i = 0; i < 10; i++) {
        GPIO_BIT_SET(led_gpio);          // HIGH
        bsw_delay_ms(100);  // 100ms 대기
        GPIO_BIT_CLEAR(led_gpio);        // LOW  
        bsw_delay_ms(100);  // 100ms 대기
    }
    
    // 3. 멀티 GPIO 비트연산 제어
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "3. 멀티 GPIO 동시 제어:");
    
    // GPIO 2, 4, 5를 모두 출력으로 설정
    uint32_t multi_gpio_mask = (1U << 2) | (1U << 4) | (1U << 5);
    
    // 출력 활성화
    *(volatile uint32_t*)BSW_GPIO_ENABLE_W1TS_REG = multi_gpio_mask;
    
    // 모든 GPIO를 동시에 HIGH로 설정
    bsw_gpio_multi_set_high(multi_gpio_mask);
    BSW_LOGI(TAG, "GPIO 2,4,5 모두 HIGH 설정");
    bsw_delay_ms(500);
    
    // 모든 GPIO를 동시에 LOW로 설정
    bsw_gpio_multi_set_low(multi_gpio_mask);
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO 2,4,5 모두 LOW 설정");
    
    // 4. 입력 GPIO 읽기 (순수 비트연산)
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "4. 입력 GPIO 비트연산 읽기:");
    
    uint8_t input_gpio = 0;  // GPIO 0을 입력으로 사용
    
    // IO_MUX를 사용하여 풀업 설정 (순수 비트연산)
    bsw_gpio_configure_iomux(input_gpio, 0, true, false);  // GPIO 기능, 풀업 활성화
    
    // 입력 상태 읽기
    uint32_t input_state = GPIO_BIT_READ(input_gpio);
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO %d 입력 상태: %s", input_gpio, input_state ? "HIGH" : "LOW");
    
    // 5. IO_MUX 레지스터 직접 제어
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "5. IO_MUX 직접 비트연산 제어:");
    
    // GPIO 2의 드라이브 강도를 최대로 설정
    bsw_gpio_set_drive_strength(led_gpio, 3);  // 최대 드라이브 강도
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO %d 드라이브 강도 최대로 설정", led_gpio);
    
    // 빠른 슬루 레이트 설정
    bsw_gpio_set_slew_rate(led_gpio, true);
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO %d 빠른 슬루 레이트 설정", led_gpio);
    
    // 6. 원시 레지스터 조작 데모
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "6. 원시 레지스터 비트 조작:");
    
    // 현재 GPIO OUT 레지스터 값 읽기
    uint32_t current_out = bsw_gpio_raw_read_reg(GPIO_OUT_REG);
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "현재 GPIO OUT 레지스터 값: 0x%08lX", current_out);
    
    // 특정 비트만 설정
    bsw_gpio_raw_set_bits(GPIO_OUT_REG, (1U << led_gpio));
    BSW_LOGI(TAG, "GPIO %d 비트 설정 완료", led_gpio);
    
    bsw_delay_ms(500);
    
    // 특정 비트만 클리어
    bsw_gpio_raw_clear_bits(GPIO_OUT_REG, (1U << led_gpio));
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO %d 비트 클리어 완료", led_gpio);
    
    // 7. 인라인 함수를 통한 최적화된 제어
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "7. 인라인 함수 최적화 제어:");
    
    // 초고속 GPIO 제어 (컴파일러 최적화로 인해 매우 빠름)
    for (int i = 0; i < 1000000; i++) {
        bsw_gpio_fast_set_high(led_gpio);
        bsw_gpio_fast_set_low(led_gpio);
    }
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "100만 회 초고속 GPIO 토글 완료!");
    
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "=== 순수 비트연산 GPIO 제어 데모 완료 ===");
}

/**
 * @brief 레지스터 값 비교 및 분석 함수
 */
void analyze_register_differences(void) {
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "=== 레지스터 제어 방식 비교 ===");
    
    uint8_t test_gpio = 2;
    
    // 방법 1: ESP-IDF 구조체 사용
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "방법 1 - ESP-IDF 구조체:");
    GPIO.out_w1ts.val = (1U << test_gpio);
    uint32_t struct_val = GPIO.out.val;
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "구조체 방식 OUT 레지스터: 0x%08lX", struct_val);
    
    // 방법 2: 순수 레지스터 주소 접근
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "방법 2 - 순수 레지스터 주소:");
    *(volatile uint32_t*)GPIO_OUT_W1TC_REG = (1U << test_gpio);
    uint32_t raw_val = *(volatile uint32_t*)GPIO_OUT_REG;
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "직접 주소 방식 OUT 레지스터: 0x%08lX", raw_val);
    
    // 방법 3: 비트 마스크 조작
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "방법 3 - 비트 마스크 조작:");
    uint32_t current = *(volatile uint32_t*)GPIO_OUT_REG;
    *(volatile uint32_t*)GPIO_OUT_REG = current | (1U << test_gpio);  // 비트 설정
    uint32_t mask_val = *(volatile uint32_t*)GPIO_OUT_REG;
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "비트 마스크 방식 OUT 레지스터: 0x%08lX", mask_val);
    
    // 레지스터 주소 정보 출력
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "레지스터 주소 정보:");
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO_BASE_ADDR: 0x%08lX", GPIO_BASE_ADDR);
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "BSW_GPIO_OUT_REG: 0x%08lX", BSW_GPIO_OUT_REG);
    BSW_LOGI(TAG, "BSW_GPIO_OUT_W1TS_REG: 0x%08lX", BSW_GPIO_OUT_W1TS_REG);
    BSW_LOGI(TAG, "BSW_GPIO_OUT_W1TC_REG: 0x%08lX", BSW_GPIO_OUT_W1TC_REG);
}

/**
 * @brief 성능 테스트 함수
 */
void performance_test_bitwise_gpio(void) {
    BSW_LOGI(TAG, "=== GPIO 제어 성능 테스트 ===");
    
    uint8_t test_gpio = 2;
    uint32_t iterations = 100000;
    
    // 테스트 1: ESP-IDF 구조체 방식
    uint32_t start_time = bsw_get_time_ms();
    for (uint32_t i = 0; i < iterations; i++) {
        GPIO.out_w1ts.val = (1U << test_gpio);
        GPIO.out_w1tc.val = (1U << test_gpio);
    }
    uint32_t struct_time = bsw_get_time_ms() - start_time;
    
    // 테스트 2: 순수 레지스터 주소 방식
    start_time = bsw_get_time_ms();
    for (uint32_t i = 0; i < iterations; i++) {
        *(volatile uint32_t*)GPIO_OUT_W1TS_REG = (1U << test_gpio);
        *(volatile uint32_t*)GPIO_OUT_W1TC_REG = (1U << test_gpio);
    }
    uint32_t raw_time = bsw_get_time_ms() - start_time;
    
    // 테스트 3: 매크로 방식
    start_time = bsw_get_time_ms();
    for (uint32_t i = 0; i < iterations; i++) {
        GPIO_BIT_SET(test_gpio);
        GPIO_BIT_CLEAR(test_gpio);
    }
    uint32_t macro_time = bsw_get_time_ms() - start_time;
    
    // 테스트 4: 인라인 함수 방식
    start_time = bsw_get_time_ms();
    for (uint32_t i = 0; i < iterations; i++) {
        bsw_gpio_fast_set_high(test_gpio);
        bsw_gpio_fast_set_low(test_gpio);
    }
    uint32_t inline_time = bsw_get_time_ms() - start_time;
    
    // 결과 출력
    BSW_LOGI(TAG, "성능 테스트 결과 (%lu 회 반복):", iterations);
    BSW_LOGI(TAG, "ESP-IDF 구조체: %lu ms", struct_time);
    BSW_LOGI(TAG, "순수 레지스터:  %lu ms", raw_time);
    BSW_LOGI(TAG, "매크로 방식:   %lu ms", macro_time);
    BSW_LOGI(TAG, "인라인 함수:   %lu ms", inline_time);
    
    // 속도 비교 (구조체 방식 기준)
    BSW_LOGI(TAG, "속도 개선 비율 (구조체 방식 대비):");
    if (struct_time > 0) {
        BSW_LOGI(TAG, "순수 레지스터: %.2fx 빠름", (float)struct_time / raw_time);
        BSW_LOGI(TAG, "매크로 방식:  %.2fx 빠름", (float)struct_time / macro_time);
        BSW_LOGI(TAG, "인라인 함수:  %.2fx 빠름", (float)struct_time / inline_time);
    }
}