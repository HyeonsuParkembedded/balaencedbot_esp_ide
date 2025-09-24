/**
 * @file pid_controller.c
 * @brief PID 제어기 구현 파일
 * 
 * 표준 PID 알고리즘과 밸런싱 로봇을 위한 이중 루프 제어를 구현합니다.
 * 적분 와인드업 방지와 미분 킥 방지 기능이 포함되어 있습니다.
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "pid_controller.h"
#include <string.h>

/**
 * @brief PID 제어기 초기화 구현
 * 
 * 모든 PID 파라미터와 상태 변수를 기본값으로 설정합니다.
 * 출력 제한은 모터 제어에 적합한 -255~255로 설정됩니다.
 */
void pid_controller_init(pid_controller_t* pid, float Kp, float Ki, float Kd) {
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->output_min = -255.0f;
    pid->output_max = 255.0f;
    pid->first_run = true;
}

/**
 * @brief PID 게인 재설정 구현
 * 
 * 런타임에 PID 게인을 변경할 때 사용합니다.
 * 기존 적분값과 이전 오차는 유지됩니다.
 */
void pid_controller_set_tunings(pid_controller_t* pid, float Kp, float Ki, float Kd) {
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
}

/**
 * @brief 목표값 설정 구현
 * 
 * PID 제어기의 새로운 설정점을 지정합니다.
 */
void pid_controller_set_setpoint(pid_controller_t* pid, float sp) {
    pid->setpoint = sp;
}

/**
 * @brief 출력 제한 설정 구현
 * 
 * PID 출력값의 상한/하한을 설정하고, 현재 출력과 적분값이
 * 제한 범위를 벗어나면 즉시 클램핑합니다.
 */
void pid_controller_set_output_limits(pid_controller_t* pid, float min, float max) {
    pid->output_min = min;
    pid->output_max = max;
    
    // 현재 출력값 클램핑
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < pid->output_min) pid->output = pid->output_min;
    
    // 적분 와인드업 방지를 위한 적분값 클램핑
    if (pid->integral > pid->output_max) pid->integral = pid->output_max;
    else if (pid->integral < pid->output_min) pid->integral = pid->output_min;
}

/**
 * @brief PID 제어 계산 구현
 * 
 * 표준 PID 알고리즘을 구현합니다:
 * Output = Kp*error + Ki*integral + Kd*derivative
 * 
 * 특수 기능:
 * - 첫 실행 시 미분 킥 방지
 * - 적분 와인드업 방지
 * - 출력 포화 제한
 */
float pid_controller_compute(pid_controller_t* pid, float input, float dt) {
    if (pid->first_run) {
        pid->previous_error = pid->setpoint - input;
        pid->first_run = false;
        return 0.0f; // 첫 실행 시 미분 킥 방지
    }

    if (dt <= 0.0f) return pid->output; // 잘못된 시간 간격 처리
    
    float error = pid->setpoint - input;
    
    // 적분 계산 및 와인드업 방지
    pid->integral += error * dt;
    if (pid->integral > pid->output_max) pid->integral = pid->output_max;
    else if (pid->integral < pid->output_min) pid->integral = pid->output_min;

    // 미분 계산
    float derivative = (error - pid->previous_error) / dt;
    
    // PID 출력 계산
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 출력 제한
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < pid->output_min) pid->output = pid->output_min;
    
    pid->previous_error = error;

    return pid->output;
}

/**
 * @brief PID 제어기 리셋 구현
 * 
 * 적분값, 이전 오차, 출력값을 초기화하고
 * 첫 실행 플래그를 설정합니다.
 */
void pid_controller_reset(pid_controller_t* pid) {
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->first_run = true;
}

/**
 * @brief 밸런싱 PID 시스템 초기화 구현
 * 
 * 각도 제어와 속도 제어를 위한 두 개의 PID 제어기를 설정합니다.
 * 각도 PID는 밸런싱용, 속도 PID는 이동 제어용으로 사용됩니다.
 */
void balance_pid_init(balance_pid_t* balance_pid) {
    pid_controller_init(&balance_pid->pitch_pid, 50.0f, 0.0f, 2.0f);     // 각도 제어용
    pid_controller_init(&balance_pid->velocity_pid, 1.0f, 0.1f, 0.0f); // 속도 제어용
    
    balance_pid->target_velocity = 0.0f;  // 정지 상태로 시작
    balance_pid->max_tilt_angle = 45.0f;  // 안전 각도 제한
    
    // 출력 제한 설정
    pid_controller_set_output_limits(&balance_pid->pitch_pid, -255.0f, 255.0f);   // 모터 출력 범위
    pid_controller_set_output_limits(&balance_pid->velocity_pid, -10.0f, 10.0f);  // 각도 조정 범위
}

/**
 * @brief 각도 제어 PID 게인 설정 구현
 */
void balance_pid_set_balance_tunings(balance_pid_t* balance_pid, float Kp, float Ki, float Kd) {
    pid_controller_set_tunings(&balance_pid->pitch_pid, Kp, Ki, Kd);
}

/**
 * @brief 속도 제어 PID 게인 설정 구현
 */
void balance_pid_set_velocity_tunings(balance_pid_t* balance_pid, float Kp, float Ki, float Kd) {
    pid_controller_set_tunings(&balance_pid->velocity_pid, Kp, Ki, Kd);
}

/**
 * @brief 목표 이동 속도 설정 구현
 */
void balance_pid_set_target_velocity(balance_pid_t* balance_pid, float velocity) {
    balance_pid->target_velocity = velocity;
    pid_controller_set_setpoint(&balance_pid->velocity_pid, velocity);
}

/**
 * @brief 최대 기울기 각도 제한 설정 구현
 */
void balance_pid_set_max_tilt_angle(balance_pid_t* balance_pid, float angle) {
    balance_pid->max_tilt_angle = angle;
}

/**
 * @brief 밸런싱 제어 계산 구현
 * 
 * 이중 루프 제어 구조:
 * 1. 속도 PID: 현재 속도와 목표 속도의 차이로 목표 기울기 각도 계산
 * 2. 각도 PID: 목표 기울기와 현재 각도의 차이로 모터 출력 계산
 * 
 * 안전 기능: 최대 기울기 각도 초과 시 모터 정지
 */
float balance_pid_compute_balance(balance_pid_t* balance_pid, float current_angle, float gyro_rate, float current_velocity, float dt) {
    // 안전 검사: 로봇이 넘어졌는지 확인
    if ((current_angle > 0 ? current_angle : -current_angle) > balance_pid->max_tilt_angle) {
        return 0.0f; // 로봇이 넘어짐, 모터 정지
    }

    // 1단계: 속도 제어 - 목표 기울기 각도 계산
    float velocity_adjustment = pid_controller_compute(&balance_pid->velocity_pid, current_velocity, dt);

    // 2단계: 각도 제어 - 모터 출력 계산
    pid_controller_set_setpoint(&balance_pid->pitch_pid, velocity_adjustment);
    float motor_output = pid_controller_compute(&balance_pid->pitch_pid, current_angle, dt);

    return motor_output;
}

/**
 * @brief 밸런싱 PID 시스템 리셋 구현
 * 
 * 모든 PID 제어기의 상태를 초기화합니다.
 */
void balance_pid_reset(balance_pid_t* balance_pid) {
    pid_controller_reset(&balance_pid->pitch_pid);
    pid_controller_reset(&balance_pid->velocity_pid);
}