/**
 * @file kalman_filter.h
 * @brief 칼만 필터 알고리즘 헤더 파일
 * 
 * 1차원 칼만 필터를 구현하여 IMU 센서의 각도 추정에 사용됩니다.
 * 가속도계와 자이로스코프 데이터를 융합하여 드리프트 없는 정확한 각도를 계산합니다.
 * 
 * 주요 기능:
 * - 센서 융합 (가속도계 + 자이로스코프)
 * - 노이즈 필터링
 * - 바이어스 추정 및 보정
 * - 실시간 각도 추정
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct kalman_filter_t
 * @brief 칼만 필터 상태 구조체
 * 
 * 칼만 필터의 내부 상태와 노이즈 파라미터를 저장합니다.
 * 상태 벡터는 [각도, 바이어스]로 구성됩니다.
 */
typedef struct {
    float Q_angle;      ///< 각도 프로세스 노이즈 분산
    float Q_bias;       ///< 바이어스 프로세스 노이즈 분산
    float R_measure;    ///< 측정 노이즈 분산
    float angle;        ///< 추정된 각도 (degree)
    float bias;         ///< 추정된 자이로 바이어스 (degree/s)
    float rate;         ///< 바이어스 보정된 각속도 (degree/s)
    float P[2][2];      ///< 오차 공분산 행렬 (2x2)
    float K[2];         ///< 칼만 게인 벡터 (2x1)
    float y;            ///< 혁신(innovation) - 측정값과 예측값의 차이
    float S;            ///< 혁신 공분산
} kalman_filter_t;

/**
 * @defgroup KALMAN_FILTER_API 칼만 필터 API
 * @brief 1차원 칼만 필터 제어 함수들
 * @{
 */

/**
 * @brief 칼만 필터 초기화
 * 
 * 칼만 필터의 상태와 공분산 행렬을 초기값으로 설정합니다.
 * 
 * 초기 설정값:
 * - Q_angle: 0.001 (각도 프로세스 노이즈)
 * - Q_bias: 0.003 (바이어스 프로세스 노이즈)  
 * - R_measure: 0.03 (측정 노이즈)
 * - 초기 각도: 0도
 * - 초기 바이어스: 0 degree/s
 * 
 * @param kf 칼만 필터 구조체 포인터
 */
void kalman_filter_init(kalman_filter_t* kf);

/**
 * @brief 칼만 필터 각도 초기값 설정
 * 
 * 필터의 초기 각도를 설정합니다. 보통 가속도계로부터 계산된 각도를 사용합니다.
 * 
 * @param kf 칼만 필터 구조체 포인터
 * @param angle 초기 각도 (degree)
 */
void kalman_filter_set_angle(kalman_filter_t* kf, float angle);

/**
 * @brief 칼만 필터 업데이트 및 각도 추정
 * 
 * 새로운 측정값으로 칼만 필터를 업데이트하고 추정된 각도를 반환합니다.
 * 
 * 알고리즘 단계:
 * 1. 예측 단계 (자이로스코프 데이터 사용)
 * 2. 업데이트 단계 (가속도계 데이터 사용)
 * 3. 칼만 게인 계산
 * 4. 상태 및 공분산 업데이트
 * 
 * @param kf 칼만 필터 구조체 포인터
 * @param new_angle 가속도계로부터 계산된 새로운 각도 (degree)
 * @param new_rate 자이로스코프 각속도 (degree/s)
 * @param dt 샘플링 시간 간격 (s)
 * @return float 추정된 각도 (degree)
 */
float kalman_filter_get_angle(kalman_filter_t* kf, float new_angle, float new_rate, float dt);

/**
 * @brief 각도 프로세스 노이즈 설정
 * 
 * 각도 상태의 프로세스 노이즈 분산을 설정합니다.
 * 값이 클수록 자이로스코프를 더 신뢰하고, 작을수록 가속도계를 더 신뢰합니다.
 * 
 * @param kf 칼만 필터 구조체 포인터
 * @param Q_angle 각도 프로세스 노이즈 분산 (일반적으로 0.001)
 */
void kalman_filter_set_qangle(kalman_filter_t* kf, float Q_angle);

/**
 * @brief 바이어스 프로세스 노이즈 설정
 * 
 * 자이로 바이어스의 프로세스 노이즈 분산을 설정합니다.
 * 값이 클수록 바이어스 변화에 빠르게 적응합니다.
 * 
 * @param kf 칼만 필터 구조체 포인터
 * @param Q_bias 바이어스 프로세스 노이즈 분산 (일반적으로 0.003)
 */
void kalman_filter_set_qbias(kalman_filter_t* kf, float Q_bias);

/**
 * @brief 측정 노이즈 설정
 * 
 * 가속도계 측정값의 노이즈 분산을 설정합니다.
 * 값이 클수록 가속도계 측정값을 덜 신뢰합니다.
 * 
 * @param kf 칼만 필터 구조체 포인터
 * @param R_measure 측정 노이즈 분산 (일반적으로 0.03)
 */
void kalman_filter_set_rmeasure(kalman_filter_t* kf, float R_measure);

/** @} */ // KALMAN_FILTER_API

#ifdef __cplusplus
}
#endif

#endif