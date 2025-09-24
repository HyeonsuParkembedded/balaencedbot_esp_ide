/**
 * @file config.h
 * @brief BalanceBot 하드웨어 및 시스템 설정 파일
 * 
 * 이 파일은 BalanceBot 프로젝트의 모든 하드웨어 핀 할당, 제어 시스템 파라미터,
 * 태스크 설정, 통신 설정 등을 정의합니다.
 * 
 * ESP32-S3-DevKitC-1 보드에 최적화되어 있으며, 다음과 같은 구성요소를 지원합니다:
 * - MPU6050 IMU 센서 (I2C)
 * - GPS 모듈 (UART)
 * - 좌우 모터와 엔코더
 * - 서보 모터 (기립 보조)
 * - BLE 통신
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifndef NATIVE_BUILD
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#else
// Native build - mock ESP32 types
#ifndef GPIO_NUM_T_DEFINED
#define GPIO_NUM_T_DEFINED
typedef enum {
    GPIO_NUM_0 = 0,
    GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5,
    GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10,
    GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17,
    GPIO_NUM_18, GPIO_NUM_20, GPIO_NUM_21
} gpio_num_t;
#endif

#ifndef LEDC_CHANNEL_T_DEFINED
#define LEDC_CHANNEL_T_DEFINED
typedef enum {
    I2C_NUM_0 = 0,
    UART_NUM_2 = 2,
    LEDC_CHANNEL_0 = 0,
    LEDC_CHANNEL_1 = 1,
    LEDC_CHANNEL_2 = 2,
    ADC1_CHANNEL_0 = 0
} ledc_channel_t;
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup HARDWARE_CONFIG 하드웨어 핀 설정
 * @brief ESP32-S3-DevKitC-1 보드의 하드웨어 핀 할당 정의
 * @{
 */

// ========================================
// Hardware Pin Configuration
// ========================================

/**
 * @defgroup MPU6050_CONFIG MPU6050 IMU 센서 설정
 * @brief 6축 관성 센서 I2C 인터페이스 설정
 * @{
 */
#define CONFIG_MPU6050_SDA_PIN          GPIO_NUM_8   ///< I2C SDA 핀 (ESP32-S3 표준)
#define CONFIG_MPU6050_SCL_PIN          GPIO_NUM_9   ///< I2C SCL 핀 (ESP32-S3 표준)
#define CONFIG_MPU6050_I2C_PORT         I2C_NUM_0    ///< I2C 포트 번호
/** @} */

/**
 * @defgroup GPS_CONFIG GPS 모듈 설정
 * @brief UART 기반 GPS 모듈 인터페이스 설정
 * @{
 */
#define CONFIG_GPS_RX_PIN               GPIO_NUM_18  ///< GPS UART RX 핀
#define CONFIG_GPS_TX_PIN               GPIO_NUM_17  ///< GPS UART TX 핀
#define CONFIG_GPS_UART_PORT            UART_NUM_2   ///< UART 포트 번호
#define CONFIG_GPS_BAUDRATE             9600         ///< GPS 통신 속도 (bps)
/** @} */

/**
 * @defgroup LEFT_MOTOR_CONFIG 좌측 모터 설정
 * @brief 좌측 모터 제어 핀 및 PWM 채널 설정
 * @{
 */
#define CONFIG_LEFT_MOTOR_A_PIN         GPIO_NUM_1   ///< 좌측 모터 A 방향 핀
#define CONFIG_LEFT_MOTOR_B_PIN         GPIO_NUM_2   ///< 좌측 모터 B 방향 핀
#define CONFIG_LEFT_MOTOR_EN_PIN        GPIO_NUM_3   ///< 좌측 모터 PWM 활성화 핀
#define CONFIG_LEFT_MOTOR_CHANNEL       LEDC_CHANNEL_0 ///< 좌측 모터 PWM 채널
/** @} */

/**
 * @defgroup LEFT_ENCODER_CONFIG 좌측 엔코더 설정
 * @brief 좌측 모터 회전 인코더 핀 설정
 * @{
 */
#define CONFIG_LEFT_ENC_A_PIN           GPIO_NUM_4   ///< 좌측 엔코더 A상 핀
#define CONFIG_LEFT_ENC_B_PIN           GPIO_NUM_5   ///< 좌측 엔코더 B상 핀
/** @} */

/**
 * @defgroup RIGHT_MOTOR_CONFIG 우측 모터 설정
 * @brief 우측 모터 제어 핀 및 PWM 채널 설정
 * @{
 */
#define CONFIG_RIGHT_MOTOR_A_PIN        GPIO_NUM_6   ///< 우측 모터 A 방향 핀
#define CONFIG_RIGHT_MOTOR_B_PIN        GPIO_NUM_7   ///< 우측 모터 B 방향 핀
#define CONFIG_RIGHT_MOTOR_EN_PIN       GPIO_NUM_15  ///< 우측 모터 PWM 활성화 핀
#define CONFIG_RIGHT_MOTOR_CHANNEL      LEDC_CHANNEL_1 ///< 우측 모터 PWM 채널
/** @} */

/**
 * @defgroup RIGHT_ENCODER_CONFIG 우측 엔코더 설정
 * @brief 우측 모터 회전 인코더 핀 설정
 * @{
 */
#define CONFIG_RIGHT_ENC_A_PIN          GPIO_NUM_16  ///< 우측 엔코더 A상 핀
#define CONFIG_RIGHT_ENC_B_PIN          GPIO_NUM_21  ///< 우측 엔코더 B상 핀
/** @} */

/**
 * @defgroup SERVO_CONFIG 서보 모터 설정
 * @brief 기립 보조용 서보 모터 제어 설정
 * @{
 */
#define CONFIG_SERVO_PIN                GPIO_NUM_10  ///< 서보 모터 PWM 신호 핀
#define CONFIG_SERVO_CHANNEL            LEDC_CHANNEL_2 ///< 서보 모터 PWM 채널
/** @} */

/**
 * @defgroup BATTERY_CONFIG 배터리 전압 측정 설정
 * @brief 배터리 전압 모니터링용 ADC 핀 설정
 * @{
 */
#define CONFIG_BATTERY_ADC_PIN          GPIO_NUM_1   ///< 배터리 전압 측정 ADC 핀 (ADC1_CH0)
#define CONFIG_BATTERY_ADC_CHANNEL      ADC1_CHANNEL_0 ///< ADC 채널 번호
#define CONFIG_BATTERY_R1_KOHM          10.0f        ///< 전압분배 상단 저항 (kΩ)
#define CONFIG_BATTERY_R2_KOHM          3.3f         ///< 전압분배 하단 저항 (kΩ)
#define CONFIG_BATTERY_MAX_VOLTAGE      8.4f         ///< 배터리 최대 전압 (V) - 2S 리튬 완충
#define CONFIG_BATTERY_MIN_VOLTAGE      6.0f         ///< 배터리 최소 전압 (V) - 2S 리튬 방전 컷오프
#define CONFIG_BATTERY_LOW_THRESHOLD    6.8f         ///< 저전압 경고 임계값 (V)
#define CONFIG_BATTERY_CRITICAL_THRESHOLD 6.4f       ///< 위험 전압 임계값 (V)
/** @} */

/** @} */ // HARDWARE_CONFIG

/**
 * @defgroup CONTROL_CONFIG 제어 시스템 파라미터
 * @brief PID 제어기, 칼만 필터, 로봇 물리 파라미터 설정
 * @{
 */

// ========================================
// Control System Parameters
// ========================================

/**
 * @defgroup PID_CONFIG PID 제어기 설정
 * @brief 밸런싱 제어를 위한 PID 게인 및 출력 제한 설정
 * @{
 */
#define CONFIG_BALANCE_PID_KP           50.0f        ///< 비례 게인 (Proportional)
#define CONFIG_BALANCE_PID_KI           0.5f         ///< 적분 게인 (Integral)
#define CONFIG_BALANCE_PID_KD           2.0f         ///< 미분 게인 (Derivative)
#define CONFIG_PID_OUTPUT_MIN           -255.0f      ///< PID 출력 최솟값
#define CONFIG_PID_OUTPUT_MAX           255.0f       ///< PID 출력 최댓값
/** @} */

/**
 * @defgroup KALMAN_CONFIG 칼만 필터 설정
 * @brief 센서 융합을 위한 칼만 필터 노이즈 파라미터
 * @{
 */
#define CONFIG_KALMAN_Q_ANGLE           0.001f       ///< 각도 프로세스 노이즈
#define CONFIG_KALMAN_Q_BIAS            0.003f       ///< 바이어스 프로세스 노이즈
#define CONFIG_KALMAN_R_MEASURE         0.03f        ///< 측정 노이즈
/** @} */

/**
 * @defgroup ROBOT_PHYSICAL_CONFIG 로봇 물리 파라미터
 * @brief 로봇의 물리적 특성 정의
 * @{
 */
#define CONFIG_WHEEL_DIAMETER_CM        6.5f         ///< 바퀴 직경 (cm)
#define CONFIG_ENCODER_PPR              360          ///< 엔코더 펄스/회전
/** @} */

/**
 * @defgroup STATE_MACHINE_CONFIG 상태 머신 임계값
 * @brief 로봇 상태 전환을 위한 임계값 설정
 * @{
 */
#define CONFIG_FALLEN_ANGLE_THRESHOLD   45.0f        ///< 넘어짐 판정 각도 (degree)
#define CONFIG_BALANCE_ANGLE_TARGET     0.0f         ///< 밸런스 목표 각도 (degree)
#define CONFIG_STANDUP_ANGLE_TOLERANCE  5.0f         ///< 기립 완료 허용 오차 (degree)
#define CONFIG_RECOVERY_TIMEOUT_MS      30000        ///< 복구 시도 타임아웃 (ms)
/** @} */

/**
 * @defgroup SERVO_PHYSICAL_CONFIG 서보 각도 설정
 * @brief 서보 모터의 동작 각도 정의
 * @{
 */
#define CONFIG_SERVO_EXTENDED_ANGLE     90           ///< 서보 확장 각도 (degree)
#define CONFIG_SERVO_RETRACTED_ANGLE    0            ///< 서보 수축 각도 (degree)
/** @} */

/** @} */ // CONTROL_CONFIG

/**
 * @defgroup TASK_CONFIG 태스크 설정
 * @brief FreeRTOS 태스크 스택 크기, 우선순위, 업데이트 주기 설정
 * @{
 */

// ========================================
// Task Configuration
// ========================================

/**
 * @defgroup TASK_STACK_CONFIG 태스크 스택 크기 설정
 * @brief 각 태스크별 스택 메모리 할당 크기
 * @{
 */
#define CONFIG_SENSOR_TASK_STACK        4096         ///< 센서 읽기 태스크 스택 크기 (bytes)
#define CONFIG_BALANCE_TASK_STACK       4096         ///< 밸런싱 제어 태스크 스택 크기 (bytes)
#define CONFIG_STATUS_TASK_STACK        4096         ///< 상태 모니터링 태스크 스택 크기 (bytes)
/** @} */

/**
 * @defgroup TASK_PRIORITY_CONFIG 태스크 우선순위 설정
 * @brief FreeRTOS 태스크 우선순위 (높을수록 우선순위 높음)
 * @{
 */
#define CONFIG_SENSOR_TASK_PRIORITY     5            ///< 센서 태스크 우선순위 (최고)
#define CONFIG_BALANCE_TASK_PRIORITY    4            ///< 밸런싱 태스크 우선순위 (높음)
#define CONFIG_STATUS_TASK_PRIORITY     3            ///< 상태 태스크 우선순위 (중간)
/** @} */

/**
 * @defgroup TASK_UPDATE_CONFIG 태스크 업데이트 주기 설정
 * @brief 각 태스크의 실행 주기 (밀리초)
 * @{
 */
#define CONFIG_SENSOR_UPDATE_RATE       20           ///< 센서 업데이트 주기 (ms) - 50Hz
#define CONFIG_BALANCE_UPDATE_RATE      20           ///< 밸런싱 업데이트 주기 (ms) - 50Hz
#define CONFIG_STATUS_UPDATE_RATE       1000         ///< 상태 업데이트 주기 (ms) - 1Hz
/** @} */

/** @} */ // TASK_CONFIG

/**
 * @defgroup COMM_CONFIG 통신 설정
 * @brief BLE 통신 및 메시지 버퍼 설정
 * @{
 */

// ========================================
// Communication Configuration
// ========================================

#define CONFIG_BLE_DEVICE_NAME          "BalanceBot" ///< BLE 광고 디바이스 이름
#define CONFIG_STATUS_BUFFER_SIZE       128          ///< 상태 메시지 버퍼 크기 (bytes)

/** @} */ // COMM_CONFIG

/**
 * @defgroup SYSTEM_CONFIG 시스템 설정
 * @brief 시스템 레벨 설정 및 오류 복구 파라미터
 * @{
 */

// ========================================
// System Configuration
// ========================================

#define CONFIG_WATCHDOG_TIMEOUT_MS      5000         ///< 워치독 타임아웃 (ms)
#define CONFIG_MAX_INIT_RETRIES         3            ///< 초기화 최대 재시도 횟수
#define CONFIG_ERROR_RECOVERY_DELAY     1000         ///< 오류 복구 지연 시간 (ms)

/** @} */ // SYSTEM_CONFIG

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H