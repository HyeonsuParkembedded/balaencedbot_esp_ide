/**
 * @file pid_controller.h
 * @brief PID 제어기 헤더 파일
 * 
 * 비례-적분-미분(PID) 제어 알고리즘을 구현합니다.
 * 단일 PID 제어기와 밸런싱 전용 이중 PID 제어기를 제공합니다.
 * 
 * 주요 기능:
 * - 기본 PID 제어 알고리즘
 * - 출력 포화 제한
 * - 적분 와인드업 방지
 * - 이중 루프 밸런싱 제어 (각도 + 속도)
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup PID_STRUCTS PID 제어기 구조체
 * @brief PID 제어기 데이터 구조체 정의
 * @{
 */

/**
 * @struct pid_controller_t
 * @brief 기본 PID 제어기 구조체
 * 
 * 단일 PID 제어 루프의 모든 상태와 파라미터를 관리합니다.
 */
typedef struct {
    float kp, ki, kd;        ///< PID 게인 (비례, 적분, 미분)
    float setpoint;          ///< 목표값 (설정점)
    float integral;          ///< 적분 누적값
    float previous_error;    ///< 이전 오차 (미분 계산용)
    float output;            ///< 현재 출력값
    float output_min;        ///< 출력 최솟값 제한
    float output_max;        ///< 출력 최댓값 제한
    bool first_run;          ///< 첫 실행 플래그 (미분 점프 방지)
} pid_controller_t;

/**
 * @struct balance_pid_t
 * @brief 밸런싱 전용 이중 PID 제어기 구조체
 * 
 * 각도 제어와 속도 제어를 결합한 캐스케이드 PID 시스템입니다.
 * 외부 루프는 각도를 제어하고, 내부 루프는 속도를 제어합니다.
 */
typedef struct {
    pid_controller_t pitch_pid;     ///< 피치 각도 제어 PID
    pid_controller_t velocity_pid;  ///< 속도 제어 PID
    float target_velocity;          ///< 목표 이동 속도 (cm/s)
    float max_tilt_angle;          ///< 최대 허용 기울기 각도 (degree)
} balance_pid_t;

/** @} */ // PID_STRUCTS

/**
 * @defgroup PID_BASIC_API 기본 PID 제어기 API
 * @brief 단일 PID 제어기 함수들
 * @{
 */

/**
 * @brief PID 제어기 초기화
 * 
 * PID 제어기의 게인을 설정하고 내부 상태를 초기화합니다.
 * 
 * @param pid PID 제어기 구조체 포인터
 * @param Kp 비례 게인 (Proportional gain)
 * @param Ki 적분 게인 (Integral gain)
 * @param Kd 미분 게인 (Derivative gain)
 */
void pid_controller_init(pid_controller_t* pid, float Kp, float Ki, float Kd);

/**
 * @brief PID 게인 재설정
 * 
 * 런타임에 PID 게인을 변경합니다. 튜닝 시 유용합니다.
 * 
 * @param pid PID 제어기 구조체 포인터
 * @param Kp 새로운 비례 게인
 * @param Ki 새로운 적분 게인
 * @param Kd 새로운 미분 게인
 */
void pid_controller_set_tunings(pid_controller_t* pid, float Kp, float Ki, float Kd);

/**
 * @brief 목표값 설정
 * 
 * PID 제어기의 설정점(목표값)을 변경합니다.
 * 
 * @param pid PID 제어기 구조체 포인터
 * @param sp 새로운 설정점
 */
void pid_controller_set_setpoint(pid_controller_t* pid, float sp);

/**
 * @brief 출력 제한 설정
 * 
 * PID 출력값의 상한과 하한을 설정하여 포화를 방지합니다.
 * 적분 와인드업도 자동으로 방지됩니다.
 * 
 * @param pid PID 제어기 구조체 포인터
 * @param min 출력 최솟값
 * @param max 출력 최댓값
 */
void pid_controller_set_output_limits(pid_controller_t* pid, float min, float max);

/**
 * @brief PID 제어 계산
 * 
 * 현재 입력값과 목표값의 차이를 기반으로 PID 출력을 계산합니다.
 * 
 * @param pid PID 제어기 구조체 포인터
 * @param input 현재 프로세스 변수 (측정값)
 * @param dt 샘플링 시간 간격 (초)
 * @return float 계산된 제어 출력
 */
float pid_controller_compute(pid_controller_t* pid, float input, float dt);

/**
 * @brief PID 제어기 리셋
 * 
 * 적분 누적값과 이전 오차를 초기화합니다.
 * 시스템 재시작이나 급격한 설정점 변경 시 사용합니다.
 * 
 * @param pid PID 제어기 구조체 포인터
 */
void pid_controller_reset(pid_controller_t* pid);

/** @} */ // PID_BASIC_API

/**
 * @defgroup BALANCE_PID_API 밸런싱 PID 제어기 API
 * @brief 이중 루프 밸런싱 제어 함수들
 * @{
 */

/**
 * @brief 밸런싱 PID 시스템 초기화
 * 
 * 각도 제어와 속도 제어를 위한 이중 PID 루프를 초기화합니다.
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 */
void balance_pid_init(balance_pid_t* balance_pid);

/**
 * @brief 각도 제어 PID 게인 설정
 * 
 * 피치 각도 제어를 위한 PID 게인을 설정합니다.
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 * @param Kp 각도 제어 비례 게인
 * @param Ki 각도 제어 적분 게인
 * @param Kd 각도 제어 미분 게인
 */
void balance_pid_set_balance_tunings(balance_pid_t* balance_pid, float Kp, float Ki, float Kd);

/**
 * @brief 속도 제어 PID 게인 설정
 * 
 * 로봇 이동 속도 제어를 위한 PID 게인을 설정합니다.
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 * @param Kp 속도 제어 비례 게인
 * @param Ki 속도 제어 적분 게인
 * @param Kd 속도 제어 미분 게인
 */
void balance_pid_set_velocity_tunings(balance_pid_t* balance_pid, float Kp, float Ki, float Kd);

/**
 * @brief 목표 이동 속도 설정
 * 
 * 로봇이 유지해야 할 목표 이동 속도를 설정합니다.
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 * @param velocity 목표 속도 (cm/s, 양수: 전진, 음수: 후진)
 */
void balance_pid_set_target_velocity(balance_pid_t* balance_pid, float velocity);

/**
 * @brief 최대 기울기 각도 제한 설정
 * 
 * 안전을 위한 최대 허용 기울기 각도를 설정합니다.
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 * @param angle 최대 기울기 각도 (degree)
 */
void balance_pid_set_max_tilt_angle(balance_pid_t* balance_pid, float angle);

/**
 * @brief 밸런싱 제어 계산
 * 
 * 이중 루프 제어를 수행하여 모터 출력을 계산합니다.
 * 
 * 제어 구조:
 * 1. 속도 오차로부터 목표 기울기 각도 계산
 * 2. 각도 오차로부터 모터 출력 계산
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 * @param current_angle 현재 피치 각도 (degree)
 * @param gyro_rate 현재 각속도 (degree/s)
 * @param current_velocity 현재 이동 속도 (cm/s)
 * @param dt 샘플링 시간 간격 (초)
 * @return float 계산된 모터 출력 (-255 ~ 255)
 */
float balance_pid_compute_balance(balance_pid_t* balance_pid, float current_angle, float gyro_rate, float current_velocity, float dt);

/**
 * @brief 밸런싱 PID 시스템 리셋
 * 
 * 모든 PID 제어기의 상태를 초기화합니다.
 * 
 * @param balance_pid 밸런싱 PID 구조체 포인터
 */
void balance_pid_reset(balance_pid_t* balance_pid);

/** @} */ // BALANCE_PID_API

#ifdef __cplusplus
}
#endif

#endif