/**
 * @file encoder_sensor.c
 * @brief 로터리 엔코더 센서 드라이버 구현 파일
 * 
 * Quadrature 엔코더의 A/B 상 신호를 인터럽트로 처리하여
 * 모터의 회전 방향과 위치를 실시간으로 추적합니다.
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "encoder_sensor.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#include <math.h>

#ifndef NATIVE_BUILD
static const char* ENCODER_TAG = "ENCODER_SENSOR"; ///< ESP-IDF 로깅 태그
#else
#define ENCODER_TAG "ENCODER_SENSOR" ///< 네이티브 빌드용 로깅 태그
#endif

/**
 * @brief 엔코더 인터럽트 서비스 루틴
 * 
 * GPIO 인터럽트에서 호출되어 A/B 상 신호를 읽고
 * Quadrature decoding을 수행하여 회전 방향을 판단합니다.
 * 
 * @param arg 엔코더 센서 구조체 포인터
 */
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    encoder_sensor_t* encoder = (encoder_sensor_t*)arg;

#ifndef NATIVE_BUILD
    int msb = gpio_get_level(encoder->encoder_pin_a);
    int lsb = gpio_get_level(encoder->encoder_pin_b);
#else
    // Mock for native build
    static int mock_counter = 0;
    int msb = (mock_counter >> 1) & 1;
    int lsb = mock_counter & 1;
    mock_counter++;
#endif

    int encoded = (msb << 1) | lsb;
    int sum = (encoder->last_encoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoder->encoder_count++;
    }
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        encoder->encoder_count--;
    }

    encoder->last_encoded = encoded;
}

/**
 * @brief 로터리 엔코더 센서를 초기화하고 인터럽트 설정
 * 
 * Quadrature 엔코더의 A/B 채널 GPIO를 설정하고 엣지 인터럽트를 활성화합니다.
 * 엔코더 파라미터(PPR, 휠 직경)를 설정하고 초기 상태를 초기화합니다.
 * 
 * 초기화 과정:
 * 1. GPIO 핀을 입력 모드로 설정 (풀업 저항 활성화)
 * 2. 양쪽 엣지 인터럽트 설정
 * 3. ISR 서비스 설치 및 핸들러 등록
 * 4. 엔코더 파라미터 및 상태 변수 초기화
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @param pin_a 엔코더 A 채널 GPIO 핀 번호
 * @param pin_b 엔코더 B 채널 GPIO 핀 번호
 * @param pulses_per_rev 엔코더의 회전당 펄스 수 (PPR)
 * @param wheel_diam 연결된 휠의 직경 (cm 단위)
 * @return ESP_OK 성공, ESP_FAIL GPIO 설정 또는 ISR 등록 실패
 */
esp_err_t encoder_sensor_init(encoder_sensor_t* encoder,
                             gpio_num_t pin_a, gpio_num_t pin_b,
                             int pulses_per_rev, float wheel_diam) {
    encoder->encoder_pin_a = pin_a;
    encoder->encoder_pin_b = pin_b;
    encoder->ppr = pulses_per_rev;
    encoder->wheel_diameter = wheel_diam;
    encoder->encoder_count = 0;
    encoder->last_encoded = 0;
    encoder->last_time = 0;
    encoder->last_position = 0;
    encoder->current_speed = 0.0f;

#ifndef NATIVE_BUILD
    // Configure encoder pins
    gpio_config_t encoder_config = {
        .pin_bit_mask = (1ULL << pin_a) | (1ULL << pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t ret = gpio_config(&encoder_config);
    if (ret != ESP_OK) {
        ESP_LOGE(ENCODER_TAG, "Failed to configure encoder GPIO");
        return ret;
    }

    // Install interrupt service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(ENCODER_TAG, "Failed to install ISR service");
        return ret;
    }

    // Add ISR handlers
    ret = gpio_isr_handler_add(pin_a, encoder_isr_handler, encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(ENCODER_TAG, "Failed to add ISR handler for encoder A");
        return ret;
    }

    ret = gpio_isr_handler_add(pin_b, encoder_isr_handler, encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(ENCODER_TAG, "Failed to add ISR handler for encoder B");
        return ret;
    }

    encoder->last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(ENCODER_TAG, "Encoder sensor initialized");
#endif

    return ESP_OK;
}

/**
 * @brief 엔코더 카운터와 속도를 리셋
 * 
 * 엔코더의 위치 카운터, 이전 위치, 속도 정보를 모두 0으로 초기화합니다.
 * 시간 기준점도 현재 시각으로 재설정합니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 */
void encoder_sensor_reset(encoder_sensor_t* encoder) {
    encoder->encoder_count = 0;
    encoder->last_position = 0;
    encoder->current_speed = 0.0f;
#ifndef NATIVE_BUILD
    encoder->last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
#endif
}

/**
 * @brief 현재 엔코더 위치(펄스 카운트) 반환
 * 
 * 엔코더 초기화 이후 누적된 총 펄스 카운트를 반환합니다.
 * 시계방향 회전은 양수, 반시계방향 회전은 음수로 계산됩니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return 누적 펄스 카운트 (양수: 시계방향, 음수: 반시계방향)
 */
int32_t encoder_sensor_get_position(const encoder_sensor_t* encoder) {
    return encoder->encoder_count;
}

/**
 * @brief 엔코더로부터 계산된 이동 거리 반환
 * 
 * 펄스 카운트를 회전수로 변환하고, 휠 둘레를 곱하여 실제 이동 거리를 계산합니다.
 * 계산 공식: 거리 = (펄스수 / PPR) × π × 휠직경
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return 총 이동 거리 (cm 단위, 양수: 전진, 음수: 후진)
 */
float encoder_sensor_get_distance(const encoder_sensor_t* encoder) {
    float revolutions = (float)encoder->encoder_count / encoder->ppr;
    return revolutions * M_PI * encoder->wheel_diameter;
}

/**
 * @brief 현재 계산된 이동 속도 반환
 * 
 * encoder_sensor_update_speed() 함수에서 계산된 현재 속도를 반환합니다.
 * 속도는 100ms 주기로 업데이트됩니다.
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return 현재 이동 속도 (cm/s 단위)
 */
float encoder_sensor_get_speed(const encoder_sensor_t* encoder) {
    return encoder->current_speed;
}

/**
 * @brief 엔코더 속도를 계산하고 업데이트
 * 
 * 이전 위치와 현재 위치의 차이를 시간 간격으로 나누어 속도를 계산합니다.
 * 100ms마다 속도를 업데이트하여 안정적인 속도 측정을 제공합니다.
 * 
 * 속도 계산 과정:
 * 1. 현재 시간과 이전 업데이트 시간의 차이 계산
 * 2. 100ms 이상 경과 시에만 속도 업데이트
 * 3. 위치 차이를 거리로 변환
 * 4. 거리를 시간으로 나누어 속도 계산
 * 
 * @param encoder 엔코더 센서 구조체 포인터
 * @return ESP_OK 성공
 */
esp_err_t encoder_sensor_update_speed(encoder_sensor_t* encoder) {
#ifndef NATIVE_BUILD
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t time_diff = current_time - encoder->last_time;

    if (time_diff >= 100) { // Update every 100ms
        int32_t position_diff = encoder->encoder_count - encoder->last_position;
        float distance_diff = (float)position_diff / encoder->ppr * M_PI * encoder->wheel_diameter;
        encoder->current_speed = (distance_diff / (time_diff / 1000.0f)) * 100.0f; // cm/s

        encoder->last_time = current_time;
        encoder->last_position = encoder->encoder_count;
    }
#endif
    return ESP_OK;
}