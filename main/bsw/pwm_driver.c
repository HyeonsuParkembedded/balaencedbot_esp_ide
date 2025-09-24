/**
 * @file pwm_driver.c
 * @brief PWM 신호 생성 드라이버 구현 파일
 * 
 * ESP32-C6의 LEDC(LED Control) 컨트롤러를 이용한 PWM 신호 생성 구현입니다.
 * 모터 속도 제어와 서보 모터 위치 제어에 사용됩니다.
 * 
 * 구현 특징:
 * - 10비트 해상도 (0-1023) - 더 정밀한 제어
 * - 5kHz 주파수 (모터 제어에 최적)
 * - 저속 모드 사용 (정밀한 제어)
 * - ESP32-C6 호환 클록 소스
 * - 싱글톤 타이머 초기화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0 (ESP32-C6 최적화)
 */

#include "pwm_driver.h"
#ifndef NATIVE_BUILD
#include "driver/ledc.h"
#include "esp_log.h"
#endif

#ifndef NATIVE_BUILD
static const char* PWM_TAG = "PWM_DRIVER"; ///< ESP-IDF 로깅 태그

// ESP-IDF LEDC 채널 매핑
static const ledc_channel_t ledc_channel_map[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
    LEDC_CHANNEL_4, LEDC_CHANNEL_5  // ESP32-C6는 6개 채널만 지원
};
#else
#define PWM_TAG "PWM_DRIVER" ///< 네이티브 빌드용 로깅 태그
#endif

static bool pwm_timer_initialized = false; ///< PWM 타이머 초기화 상태 플래그

/**
 * @brief PWM 드라이버 전역 초기화 구현
 * 
 * LEDC 타이머를 설정하여 PWM 신호 생성의 기반을 마련합니다.
 * 싱글톤 패턴으로 구현되어 중복 초기화를 방지합니다.
 * 
 * LEDC 타이머 설정:
 * - 속도 모드: LEDC_LOW_SPEED_MODE (정밀 제어)
 * - 타이머 번호: LEDC_TIMER_0
 * - 해상도: LEDC_TIMER_8_BIT (0-255)
 * - 주파수: 5kHz (모터 제어 최적)
 * - 클록: LEDC_AUTO_CLK (자동 선택)
 * 
 * @return esp_err_t 초기화 결과
 */
esp_err_t pwm_driver_init(void) {
    if (pwm_timer_initialized) {
        return ESP_OK; // 이미 초기화됨
    }

#ifndef NATIVE_BUILD
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,    // 10비트 해상도 (0-1023)로 정밀도 향상
        .freq_hz = 5000,                         // 5kHz 주파수
        .clk_cfg = LEDC_AUTO_CLK                 // ESP32-C6에서 자동 클록 소스 선택
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(PWM_TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(PWM_TAG, "PWM timer initialized successfully on ESP32-C6 (10-bit, 5kHz)");
#endif

    pwm_timer_initialized = true;
    return ESP_OK;
}

/**
 * @brief PWM 채널 초기화 구현
 * 
 * 지정된 GPIO 핀과 LEDC 채널을 연결하여 PWM 출력을 설정합니다.
 * 
 * LEDC 채널 설정:
 * - 속도 모드: LEDC_LOW_SPEED_MODE
 * - 타이머: LEDC_TIMER_0 (전역 초기화된 타이머)
 * - 인터럽트: 비활성화
 * - 초기 듀티: 0 (출력 OFF)
 * - H 포인트: 0 (위상 시프트 없음)
 * 
 * @param gpio PWM 출력할 GPIO 핀
 * @param channel 사용할 LEDC 채널
 * @return esp_err_t 채널 설정 결과
 */
esp_err_t pwm_channel_init(gpio_num_t gpio, pwm_channel_t channel) {
#ifndef NATIVE_BUILD
    // 추상화된 채널을 ESP-IDF LEDC 채널로 변환
    if (channel >= PWM_CHANNEL_MAX) {
        ESP_LOGE(PWM_TAG, "Invalid PWM channel: %d", channel);
        return ESP_FAIL;
    }
    
    ledc_channel_t ledc_ch = ledc_channel_map[channel];
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ledc_ch,
        .timer_sel = LEDC_TIMER_0,      // 전역 초기화된 타이머 사용
        .intr_type = LEDC_INTR_DISABLE, // 인터럽트 비활성화
        .gpio_num = gpio,
        .duty = 0,                      // 초기 듀티 0 (OFF)
        .hpoint = 0                     // 위상 시프트 없음
    };
    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(PWM_TAG, "Failed to configure LEDC channel %d on GPIO %d: %s", 
                 ledc_ch, gpio, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(PWM_TAG, "PWM channel %d configured on GPIO %d", channel, gpio);
#endif
    return ESP_OK;
}

/**
 * @brief PWM 듀티 사이클 설정 구현
 * 
 * 지정된 채널의 PWM 듀티 사이클을 설정하고 즉시 적용합니다.
 * 
 * 동작 과정:
 * 1. 듀티 값 설정 (ledc_set_duty)
 * 2. 하드웨어에 즉시 적용 (ledc_update_duty)
 * 
 * @param channel 제어할 LEDC 채널
 * @param duty 듀티 사이클 값 (0-1023, 10비트)
 * @return esp_err_t 듀티 설정 결과
 */
esp_err_t pwm_set_duty(pwm_channel_t channel, uint32_t duty) {
#ifndef NATIVE_BUILD
    // 채널 유효성 검사
    if (channel >= PWM_CHANNEL_MAX) {
        ESP_LOGE(PWM_TAG, "Invalid PWM channel: %d", channel);
        return ESP_FAIL;
    }
    
    // 추상화된 채널을 ESP-IDF LEDC 채널로 변환
    ledc_channel_t ledc_ch = ledc_channel_map[channel];
    
    // 10비트 해상도에서 최대값 확인 (0-1023)
    if (duty > 1023) {
        ESP_LOGW(PWM_TAG, "Duty value %lu exceeds 10-bit maximum (1023), clamping", duty);
        duty = 1023;
    }
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_ch, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(PWM_TAG, "Failed to set duty %lu on channel %d: %s", 
                 duty, channel, esp_err_to_name(ret));
        return ret;
    }
    
    // 하드웨어에 즉시 적용
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_ch);
    if (ret != ESP_OK) {
        ESP_LOGE(PWM_TAG, "Failed to update duty on channel %d: %s", 
                 channel, esp_err_to_name(ret));
    }
    
    return ret;
#else
    // 네이티브 빌드: 모의 동작
    return ESP_OK;
#endif
}