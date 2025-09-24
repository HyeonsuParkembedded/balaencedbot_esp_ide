/**
 * @file servo_standup.c
 * @brief 서보 모터 기립 보조 시스템 구현
 * 
 * PWM을 사용하여 서보 모터를 제어하고, 상태 머신을 통해
 * 체계적인 기립 시퀀스를 실행합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#include "servo_standup.h"
#include "../bsw/pwm_driver.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#endif

#ifndef NATIVE_BUILD
static const char* SERVO_TAG = "SERVO_STANDUP";  ///< 로깅 태그
#else
#define SERVO_TAG "SERVO_STANDUP"  ///< 네이티브 빌드용 로깅 태그
#endif

// Servo PWM constants
#define SERVO_MIN_PULSEWIDTH_US 500   ///< 최소 펄스 폭 (마이크로초)
#define SERVO_MAX_PULSEWIDTH_US 2500  ///< 최대 펄스 폭 (마이크로초)
#define SERVO_MAX_DEGREE        180   ///< 최대 각도 (도)
#define SERVO_FREQ              50    ///< 서보 주파수 (Hz)

/**
 * @brief 서보 각도를 펄스 폭으로 변환
 * 
 * 지정된 각도에 해당하는 PWM 펄스 폭을 계산합니다.
 * 
 * @param degree_of_rotation 회전 각도 (0-180도)
 * @return uint32_t 펄스 폭 (마이크로초)
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH_US + (((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

/**
 * @brief 서보 모터 각도 설정
 * 
 * PWM 듀티 사이클을 조정하여 서보 모터를 지정된 각도로 이동시킵니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param angle 목표 각도 (0-180도)
 */
static void servo_set_angle(servo_standup_t* servo, int angle) {
    // 각도 범위 제한
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 펄스 폭 계산 및 듀티 사이클 변환
    uint32_t pulse_width = servo_per_degree_init(angle);
    uint32_t duty = (pulse_width * ((1 << LEDC_TIMER_14_BIT) - 1)) / (1000000 / SERVO_FREQ);
    
    // PWM 듀티 설정 및 업데이트
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo->servo_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo->servo_channel);
    
    servo->current_angle = angle;
}

/**
 * @brief 서보 기립 시스템 초기화 구현
 * 
 * PWM 드라이버를 초기화하고 서보 채널을 설정합니다.
 * 초기 상태를 격납 위치로 설정합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param pin 서보 제어 핀
 * @param channel PWM 채널
 * @param extend_angle 확장 각도
 * @param retract_angle 격납 각도
 * @return esp_err_t 초기화 결과
 */
esp_err_t servo_standup_init(servo_standup_t* servo, gpio_num_t pin, ledc_channel_t channel, int extend_angle, int retract_angle) {
    // 구조체 멤버 초기화
    servo->servo_pin = pin;
    servo->servo_channel = channel;
    servo->extended_angle = extend_angle;
    servo->retracted_angle = retract_angle;
    servo->current_angle = retract_angle;
    servo->state = STANDUP_IDLE;
    servo->state_start_time = 0;
    servo->extend_duration = 1000;   // 확장: 1초
    servo->push_duration = 2000;     // 밀기: 2초
    servo->retract_duration = 1000;  // 격납: 1초
    servo->standup_requested = false;
    servo->standup_in_progress = false;

    // Configure LEDC timer for servo
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(SERVO_TAG, "Failed to configure LEDC timer");
        return ret;
    }

    // Configure LEDC channel for servo
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = pin,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(SERVO_TAG, "Failed to configure LEDC channel");
        return ret;
    }

    // Set initial position
    servo_set_angle(servo, retract_angle);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(SERVO_TAG, "Servo standup initialized");
    return ESP_OK;
}

/**
 * @brief 기립 동작 요청 구현
 * 
 * 기립 시퀀스를 시작하도록 요청합니다.
 * 이미 진행 중인 경우 요청을 무시합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 */
void servo_standup_request_standup(servo_standup_t* servo) {
    if (!servo->standup_in_progress) {
        servo->standup_requested = true;
    }
}

/**
 * @brief 서보 기립 상태 업데이트 구현
 * 
 * 기립 상태 머신을 실행하여 각 단계를 순차적으로 처리합니다.
 * 확장 → 밀기 → 격납 → 완료 순서로 진행됩니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 */
void servo_standup_update(servo_standup_t* servo) {
    // 기립 요청이 있고 진행 중이 아닌 경우 시작
    if (servo->standup_requested && !servo->standup_in_progress) {
        servo->standup_requested = false;
        servo->standup_in_progress = true;
        servo->state = STANDUP_EXTENDING;
        servo->state_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGI(SERVO_TAG, "Starting standup sequence");
    }
    
    if (!servo->standup_in_progress) return;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed_time = current_time - servo->state_start_time;
    
    switch (servo->state) {
        case STANDUP_EXTENDING:
            if (elapsed_time == 0 || servo->current_angle != servo->extended_angle) {
                servo_set_angle(servo, servo->extended_angle);
                ESP_LOGI(SERVO_TAG, "Extending servo to %d degrees", servo->extended_angle);
            }
            if (elapsed_time >= servo->extend_duration) {
                servo->state = STANDUP_PUSHING;
                servo->state_start_time = current_time;
                ESP_LOGI(SERVO_TAG, "Holding position for push");
            }
            break;
            
        case STANDUP_PUSHING:
            if (elapsed_time >= servo->push_duration) {
                servo->state = STANDUP_RETRACTING;
                servo->state_start_time = current_time;
                servo_set_angle(servo, servo->retracted_angle);
                ESP_LOGI(SERVO_TAG, "Retracting servo to %d degrees", servo->retracted_angle);
            }
            break;
            
        case STANDUP_RETRACTING:
            if (elapsed_time >= servo->retract_duration) {
                servo->state = STANDUP_COMPLETE;
                servo->state_start_time = current_time;
                ESP_LOGI(SERVO_TAG, "Standup sequence complete");
            }
            break;
            
        case STANDUP_COMPLETE:
            if (elapsed_time >= 500) { // Wait 500ms before allowing next standup
                servo->state = STANDUP_IDLE;
                servo->standup_in_progress = false;
                ESP_LOGI(SERVO_TAG, "Ready for next standup");
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief 기립 동작 진행 상태 확인 구현
 * 
 * 현재 기립 동작이 진행 중인지 확인합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @return bool 진행 상태
 */
bool servo_standup_is_standing_up(const servo_standup_t* servo) {
    return servo->standup_in_progress;
}

/**
 * @brief 기립 완료 상태 확인 구현
 * 
 * 기립 시퀀스가 완료 상태에 있는지 확인합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @return bool 완료 상태
 */
bool servo_standup_is_complete(const servo_standup_t* servo) {
    return servo->state == STANDUP_COMPLETE;
}

/**
 * @brief 기립 시스템 리셋 구현
 * 
 * 기립 상태를 초기 상태로 되돌리고 모든 플래그를 초기화합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 */
void servo_standup_reset(servo_standup_t* servo) {
    servo->state = STANDUP_IDLE;
    servo->standup_in_progress = false;
    servo->standup_requested = false;
    servo_set_angle(servo, servo->retracted_angle);
    ESP_LOGI(SERVO_TAG, "Servo standup reset");
}

/**
 * @brief 기립 동작 타이밍 설정 구현
 * 
 * 각 기립 단계의 지속 시간을 동적으로 설정합니다.
 * 로봇의 크기나 환경에 따라 최적화된 타이밍을 적용할 수 있습니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param extend 확장 지속 시간 (밀리초)
 * @param push 밀기 지속 시간 (밀리초)
 * @param retract 격납 지속 시간 (밀리초)
 */
void servo_standup_set_timings(servo_standup_t* servo, uint32_t extend, uint32_t push, uint32_t retract) {
    servo->extend_duration = extend;
    servo->push_duration = push;
    servo->retract_duration = retract;
}

/**
 * @brief 서보 각도 설정 구현
 * 
 * 확장 및 격납 시의 서보 각도를 동적으로 설정합니다.
 * 기립이 진행 중이 아닐 때만 즉시 격납 위치로 이동합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param extend 확장 각도 (0-180도)
 * @param retract 격납 각도 (0-180도)
 */
void servo_standup_set_angles(servo_standup_t* servo, int extend, int retract) {
    servo->extended_angle = extend;
    servo->retracted_angle = retract;
    // 기립이 진행 중이 아닐 때만 즉시 격납 위치로 이동
    if (!servo->standup_in_progress) {
        servo_set_angle(servo, retract);
    }
}

/**
 * @brief 현재 기립 상태 가져오기 구현
 * 
 * 서보 기립 시스템의 현재 상태를 반환합니다.
 * 디버깅이나 상태 모니터링에 사용됩니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @return standup_state_t 현재 기립 상태
 */
standup_state_t servo_standup_get_state(const servo_standup_t* servo) {
    return servo->state;
}