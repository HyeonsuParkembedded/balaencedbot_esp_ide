/**
 * @file pwm_driver.c
 * @brief PWM 신호 생성 드라이버 구현 파일
 * 
 * ESP32-S3의 LEDC(LED Control) 컨트롤러를 이용한 PWM 신호 생성 구현입니다.
 * 모터 속도 제어와 서보 모터 위치 제어에 사용됩니다.
 * 
 * 구현 특징:
 * - 8비트 해상도 (0-255)
 * - 5kHz 주파수 (모터 제어에 최적)
 * - 저속 모드 사용 (정밀한 제어)
 * - 싱글톤 타이머 초기화
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "pwm_driver.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#endif

#ifndef NATIVE_BUILD
static const char* PWM_TAG = "PWM_DRIVER"; ///< ESP-IDF 로깅 태그
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
        .duty_resolution = LEDC_TIMER_8_BIT, // 8비트 해상도 (0-255)
        .freq_hz = 5000,                     // 5kHz 주파수
        .clk_cfg = LEDC_AUTO_CLK             // 클록 자동 선택
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(PWM_TAG, "Failed to configure LEDC timer");
        return ret;
    }
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
esp_err_t pwm_channel_init(gpio_num_t gpio, ledc_channel_t channel) {
#ifndef NATIVE_BUILD
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0,    // 전역 초기화된 타이머 사용
        .intr_type = LEDC_INTR_DISABLE, // 인터럽트 비활성화
        .gpio_num = gpio,
        .duty = 0,                    // 초기 듀티 0 (OFF)
        .hpoint = 0                   // 위상 시프트 없음
    };
    esp_err_t ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(PWM_TAG, "Failed to configure LEDC channel");
        return ret;
    }
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
 * @param duty 듀티 사이클 값 (0-255, 8비트)
 * @return esp_err_t 듀티 설정 결과
 */
esp_err_t pwm_set_duty(ledc_channel_t channel, uint32_t duty) {
#ifndef NATIVE_BUILD
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (ret != ESP_OK) {
        return ret;
    }
    // 하드웨어에 즉시 적용
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
#else
    // 네이티브 빌드: 모의 동작
    return ESP_OK;
#endif
}