/**
 * @file config_manager.h
 * @brief 설정 값 관리 시스템 헤더 파일
 * 
 * NVS(Non-Volatile Storage)를 사용하여 PID 게인, 센서 노이즈 파라미터 등을
 * 저장하고 BLE를 통해 실시간으로 튜닝할 수 있는 기능을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-08
 * @version 1.0
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CONFIG_KEYS 설정 키 정의
 * @brief NVS에 저장되는 설정 키 문자열
 * @{
 */
#define CONFIG_KEY_BALANCE_KP       "bal_kp"        ///< 밸런스 PID Kp
#define CONFIG_KEY_BALANCE_KI       "bal_ki"        ///< 밸런스 PID Ki
#define CONFIG_KEY_BALANCE_KD       "bal_kd"        ///< 밸런스 PID Kd
#define CONFIG_KEY_VELOCITY_KP      "vel_kp"        ///< 속도 PID Kp
#define CONFIG_KEY_VELOCITY_KI      "vel_ki"        ///< 속도 PID Ki
#define CONFIG_KEY_VELOCITY_KD      "vel_kd"        ///< 속도 PID Kd
#define CONFIG_KEY_KALMAN_Q_ANGLE   "kal_q_ang"     ///< 칼만 각도 프로세스 노이즈
#define CONFIG_KEY_KALMAN_Q_BIAS    "kal_q_bias"    ///< 칼만 바이어스 프로세스 노이즈
#define CONFIG_KEY_KALMAN_R_MEASURE "kal_r_meas"    ///< 칼만 측정 노이즈
#define CONFIG_KEY_MAX_TILT_ANGLE   "max_tilt"      ///< 최대 기울기 각도
#define CONFIG_KEY_FALLEN_THRESHOLD "fall_thresh"   ///< 넘어짐 판정 임계값
/** @} */

/**
 * @struct tuning_params_t
 * @brief 런타임 튜닝 가능한 파라미터 구조체
 */
typedef struct {
    // PID 게인 값
    float balance_kp;           ///< 밸런스 PID 비례 게인
    float balance_ki;           ///< 밸런스 PID 적분 게인
    float balance_kd;           ///< 밸런스 PID 미분 게인
    float velocity_kp;          ///< 속도 PID 비례 게인
    float velocity_ki;          ///< 속도 PID 적분 게인
    float velocity_kd;          ///< 속도 PID 미분 게인
    
    // 칼만 필터 파라미터
    float kalman_q_angle;       ///< 각도 프로세스 노이즈
    float kalman_q_bias;        ///< 바이어스 프로세스 노이즈
    float kalman_r_measure;     ///< 측정 노이즈
    
    // 제어 파라미터
    float max_tilt_angle;       ///< 최대 허용 기울기 각도
    float fallen_threshold;     ///< 넘어짐 판정 임계값
} tuning_params_t;

/**
 * @enum config_param_id_t
 * @brief 설정 파라미터 ID 열거형
 */
typedef enum {
    CONFIG_PARAM_BALANCE_KP = 0,
    CONFIG_PARAM_BALANCE_KI,
    CONFIG_PARAM_BALANCE_KD,
    CONFIG_PARAM_VELOCITY_KP,
    CONFIG_PARAM_VELOCITY_KI,
    CONFIG_PARAM_VELOCITY_KD,
    CONFIG_PARAM_KALMAN_Q_ANGLE,
    CONFIG_PARAM_KALMAN_Q_BIAS,
    CONFIG_PARAM_KALMAN_R_MEASURE,
    CONFIG_PARAM_MAX_TILT_ANGLE,
    CONFIG_PARAM_FALLEN_THRESHOLD,
    CONFIG_PARAM_COUNT  ///< 총 파라미터 개수
} config_param_id_t;

/**
 * @defgroup CONFIG_MANAGER_API 설정 관리자 API
 * @brief 설정 값 저장/로드/튜닝 함수들
 * @{
 */

/**
 * @brief 설정 관리자 초기화
 * 
 * NVS를 초기화하고 저장된 설정을 로드합니다.
 * 저장된 설정이 없으면 기본값을 사용합니다.
 * 
 * @return esp_err_t 초기화 결과
 */
esp_err_t config_manager_init(void);

/**
 * @brief 현재 튜닝 파라미터 가져오기
 * 
 * 현재 메모리에 있는 튜닝 파라미터를 반환합니다.
 * 
 * @return const tuning_params_t* 튜닝 파라미터 구조체 포인터
 */
const tuning_params_t* config_manager_get_params(void);

/**
 * @brief 특정 파라미터 값 설정
 * 
 * 메모리의 파라미터 값을 변경합니다.
 * save_to_nvs가 true면 NVS에도 저장합니다.
 * 
 * @param param_id 파라미터 ID
 * @param value 새로운 값
 * @param save_to_nvs NVS 저장 여부
 * @return esp_err_t 설정 결과
 */
esp_err_t config_manager_set_param(config_param_id_t param_id, float value, bool save_to_nvs);

/**
 * @brief 특정 파라미터 값 가져오기
 * 
 * 지정된 파라미터의 현재 값을 반환합니다.
 * 
 * @param param_id 파라미터 ID
 * @return float 파라미터 값 (오류 시 0.0f)
 */
float config_manager_get_param(config_param_id_t param_id);

/**
 * @brief 모든 설정을 NVS에 저장
 * 
 * 현재 메모리의 모든 튜닝 파라미터를 NVS에 저장합니다.
 * 
 * @return esp_err_t 저장 결과
 */
esp_err_t config_manager_save_all(void);

/**
 * @brief NVS에서 모든 설정 로드
 * 
 * NVS에서 저장된 설정을 읽어와 메모리에 로드합니다.
 * 
 * @return esp_err_t 로드 결과
 */
esp_err_t config_manager_load_all(void);

/**
 * @brief 기본값으로 초기화
 * 
 * 모든 파라미터를 기본값으로 설정합니다.
 * save_to_nvs가 true면 NVS에도 저장합니다.
 * 
 * @param save_to_nvs NVS 저장 여부
 * @return esp_err_t 초기화 결과
 */
esp_err_t config_manager_reset_defaults(bool save_to_nvs);

/**
 * @brief 파라미터 이름 가져오기
 * 
 * 파라미터 ID에 해당하는 사람이 읽기 쉬운 이름을 반환합니다.
 * 
 * @param param_id 파라미터 ID
 * @return const char* 파라미터 이름 문자열
 */
const char* config_manager_get_param_name(config_param_id_t param_id);

/**
 * @brief BLE 명령으로 파라미터 튜닝
 * 
 * BLE를 통해 수신된 명령을 파싱하여 파라미터를 설정합니다.
 * 명령 형식: "SET <param_id> <value>"
 * 
 * @param command BLE 명령 문자열
 * @return esp_err_t 처리 결과
 */
esp_err_t config_manager_handle_ble_command(const char* command);

/**
 * @brief 현재 파라미터 상태 문자열 생성
 * 
 * 모든 파라미터의 현재 값을 문자열로 생성합니다.
 * BLE 상태 전송에 사용됩니다.
 * 
 * @param buffer 출력 버퍼
 * @param buffer_size 버퍼 크기
 * @return esp_err_t 생성 결과
 */
esp_err_t config_manager_get_status_string(char* buffer, size_t buffer_size);

/** @} */ // CONFIG_MANAGER_API

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H