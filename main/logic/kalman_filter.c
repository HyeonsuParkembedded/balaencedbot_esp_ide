/**
 * @file kalman_filter.c
 * @brief 칼만 필터 알고리즘 구현 파일
 * 
 * 1차원 칼만 필터의 예측 및 업데이트 단계를 구현합니다.
 * IMU 센서 융합을 위한 최적화된 알고리즘으로, 실시간 처리에 적합합니다.
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "kalman_filter.h"

/**
 * @brief 칼만 필터 초기화 구현
 * 
 * 필터의 모든 상태 변수와 공분산 행렬을 기본값으로 설정합니다.
 * 노이즈 파라미터는 일반적인 IMU 센서에 적합한 값으로 설정됩니다.
 */
void kalman_filter_init(kalman_filter_t* kf) {
    kf->Q_angle = 0.001f;   // 각도 프로세스 노이즈
    kf->Q_bias = 0.003f;    // 바이어스 프로세스 노이즈
    kf->R_measure = 0.03f;  // 측정 노이즈

    kf->angle = 0.0f;       // 초기 각도
    kf->bias = 0.0f;        // 초기 바이어스

    // 오차 공분산 행렬 초기화
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

/**
 * @brief 칼만 필터 초기 각도 설정 구현
 * 
 * 필터의 각도 상태를 주어진 값으로 설정합니다.
 * 보통 시스템 시작 시 가속도계 각도로 초기화합니다.
 */
void kalman_filter_set_angle(kalman_filter_t* kf, float angle) {
    kf->angle = angle;
}

/**
 * @brief 칼만 필터 메인 알고리즘 구현
 * 
 * 두 단계 과정으로 구성된 칼만 필터의 핵심 알고리즘입니다:
 * 
 * 1. 예측 단계 (Prediction):
 *    - 자이로스코프 데이터로 상태 예측
 *    - 프로세스 노이즈를 고려한 공분산 예측
 * 
 * 2. 업데이트 단계 (Update):
 *    - 가속도계 측정값으로 상태 보정
 *    - 칼만 게인 계산 및 상태 업데이트
 */
float kalman_filter_get_angle(kalman_filter_t* kf, float new_angle, float new_rate, float dt) {
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    kf->S = kf->P[0][0] + kf->R_measure;
    kf->K[0] = kf->P[0][0] / kf->S;
    kf->K[1] = kf->P[1][0] / kf->S;

    kf->y = new_angle - kf->angle;

    kf->angle += kf->K[0] * kf->y;
    kf->bias += kf->K[1] * kf->y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= kf->K[0] * P00_temp;
    kf->P[0][1] -= kf->K[0] * P01_temp;
    kf->P[1][0] -= kf->K[1] * P00_temp;
    kf->P[1][1] -= kf->K[1] * P01_temp;

    return kf->angle;
}

void kalman_filter_set_qangle(kalman_filter_t* kf, float Q_angle) {
    kf->Q_angle = Q_angle;
}

void kalman_filter_set_qbias(kalman_filter_t* kf, float Q_bias) {
    kf->Q_bias = Q_bias;
}

void kalman_filter_set_rmeasure(kalman_filter_t* kf, float R_measure) {
    kf->R_measure = R_measure;
}