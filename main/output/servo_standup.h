/**
 * @file servo_standup.h
 * @brief 서보 모터 기립 보조 시스템 인터페이스
 * 
 * 밸런싱 로봇이 넘어졌을 때 다시 일어설 수 있도록 도와주는
 * 서보 모터 기반 기립 보조 메커니즘을 제어합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#ifndef SERVO_STANDUP_H
#define SERVO_STANDUP_H

#ifndef NATIVE_BUILD
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#endif
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 기립 상태 열거형
 * 
 * 서보 모터의 기립 동작 단계를 나타냅니다.
 */
typedef enum {
    STANDUP_IDLE,        ///< 대기 상태 (서보 격납됨)
    STANDUP_EXTENDING,   ///< 서보 팔 확장 중
    STANDUP_PUSHING,     ///< 바닥을 밀어 기립 중
    STANDUP_RETRACTING,  ///< 서보 팔 격납 중
    STANDUP_COMPLETE     ///< 기립 완료
} standup_state_t;

/**
 * @brief 서보 기립 제어 구조체
 * 
 * 서보 모터의 기립 동작에 필요한 모든 상태와 설정을 관리합니다.
 */
typedef struct {
    gpio_num_t servo_pin;         ///< 서보 제어 핀
    ledc_channel_t servo_channel; ///< PWM 채널
    int extended_angle;           ///< 확장 시 서보 각도 (도)
    int retracted_angle;          ///< 격납 시 서보 각도 (도)
    int current_angle;            ///< 현재 서보 각도 (도)
    standup_state_t state;        ///< 현재 기립 상태
    uint32_t state_start_time;    ///< 현재 상태 시작 시간 (ms)
    uint32_t extend_duration;     ///< 확장 동작 지속 시간 (ms)
    uint32_t push_duration;       ///< 밀기 동작 지속 시간 (ms)
    uint32_t retract_duration;    ///< 격납 동작 지속 시간 (ms)
    bool standup_requested;       ///< 기립 요청 플래그
    bool standup_in_progress;     ///< 기립 진행 중 플래그
} servo_standup_t;

/**
 * @brief 서보 기립 시스템 초기화
 * 
 * 서보 모터의 PWM 제어를 초기화하고 초기 각도를 설정합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param pin 서보 제어 핀
 * @param channel PWM 채널
 * @param extend_angle 확장 각도 (도)
 * @param retract_angle 격납 각도 (도)
 * @return esp_err_t 초기화 결과
 * @retval ESP_OK 성공
 * @retval ESP_FAIL 실패
 */
esp_err_t servo_standup_init(servo_standup_t* servo, gpio_num_t pin, ledc_channel_t channel, int extend_angle, int retract_angle);

/**
 * @brief 기립 동작 요청
 * 
 * 기립 시퀀스를 시작합니다. 이미 진행 중인 경우 무시됩니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 */
void servo_standup_request_standup(servo_standup_t* servo);

/**
 * @brief 서보 기립 상태 업데이트
 * 
 * 기립 시퀀스의 각 단계를 처리하고 상태를 갱신합니다.
 * 주기적으로 호출되어야 합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 */
void servo_standup_update(servo_standup_t* servo);

/**
 * @brief 기립 동작 진행 상태 확인
 * 
 * 현재 기립 동작이 진행 중인지 확인합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @return bool 진행 상태
 * @retval true 기립 진행 중
 * @retval false 기립 중이 아님
 */
bool servo_standup_is_standing_up(const servo_standup_t* servo);

/**
 * @brief 기립 완료 상태 확인
 * 
 * 기립 시퀀스가 완료되었는지 확인합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @return bool 완료 상태
 * @retval true 기립 완료
 * @retval false 기립 미완료
 */
bool servo_standup_is_complete(const servo_standup_t* servo);

/**
 * @brief 기립 시스템 리셋
 * 
 * 기립 상태를 초기 상태로 되돌립니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 */
void servo_standup_reset(servo_standup_t* servo);

/**
 * @brief 기립 동작 타이밍 설정
 * 
 * 각 기립 단계의 지속 시간을 설정합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param extend 확장 지속 시간 (ms)
 * @param push 밀기 지속 시간 (ms)
 * @param retract 격납 지속 시간 (ms)
 */
void servo_standup_set_timings(servo_standup_t* servo, uint32_t extend, uint32_t push, uint32_t retract);

/**
 * @brief 서보 각도 설정
 * 
 * 확장 및 격납 시의 서보 각도를 설정합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @param extend 확장 각도 (도)
 * @param retract 격납 각도 (도)
 */
void servo_standup_set_angles(servo_standup_t* servo, int extend, int retract);

/**
 * @brief 현재 기립 상태 가져오기
 * 
 * 현재 기립 동작의 상태를 반환합니다.
 * 
 * @param servo 서보 기립 구조체 포인터
 * @return standup_state_t 현재 상태
 */
standup_state_t servo_standup_get_state(const servo_standup_t* servo);

#ifdef __cplusplus
}
#endif

#endif