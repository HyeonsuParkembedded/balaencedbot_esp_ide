/**
 * @file main.c
 * @brief BalanceBot 메인 제어 프로그램
 * 
 * 이 파일은 BalanceBot의 메인 제어 로직을 포함합니다.
 * 
 * 주요 기능:
 * - FreeRTOS 태스크 기반 멀티태스킹 구조
 * - 센서 데이터 읽기 및 칼만 필터링
 * - PID 제어 기반 밸런싱 알고리즘
 * - BLE 무선 통신 및 원격 제어
 * - 서보 기반 기립 보조 시스템
 * - 안전한 상태 머신 관리
 * 
 * 태스크 구조:
 * - sensor_task: 센서 데이터 수집 및 필터링(50Hz)
 * - balance_task: PID 제어 및 모터 제어 (50Hz)
 * - status_task: 상태 모니터링 및 BLE 통신 (1Hz)
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "config.h"
#include "bsw/system_services.h"
#include "bsw/i2c_driver.h"

#include "input/imu_sensor.h"
#include "logic/kalman_filter.h"
#include "input/gps_sensor.h"
#include "input/encoder_sensor.h"
#include "input/battery_sensor.h"
#include "output/motor_control.h"
#include "output/ble_controller.h"
#include "logic/pid_controller.h"
#include "output/servo_standup.h"
#include "system/error_recovery.h"
#include "system/config_manager.h"

// Pin definitions are now in config.h

static const char* TAG = "BALANCE_ROBOT"; ///< ESP-IDF 로깅 태그

/**
 * @enum robot_state_t
 * @brief 로봇 상태 머신 정의
 * 
 * 로봇의 현재 동작 상태를 나타내는 열거형입니다.
 * 각 상태는 로봇의 특정 동작 모드를 나타냅니다.
 */
typedef enum {
    ROBOT_STATE_INIT,        ///< 초기화 상태
    ROBOT_STATE_IDLE,        ///< 대기 상태 (모터 정지)
    ROBOT_STATE_BALANCING,   ///< 밸런싱 제어 상태
    ROBOT_STATE_STANDING_UP, ///< 기립 보조 동작 상태
    ROBOT_STATE_FALLEN,      ///< 넘어진 상태
    ROBOT_STATE_ERROR        ///< 오류 상태
} robot_state_t;

static robot_state_t current_state = ROBOT_STATE_INIT; ///< 현재 로봇 상태
static SemaphoreHandle_t state_mutex = NULL;           ///< 상태 변경 보호용 뮤텍스

/**
 * @defgroup ROBOT_COMPONENTS 로봇 구성 요소
 * @brief 로봇의 하드웨어 및 소프트웨어 구성 요소 인스턴스
 * @{
 */
static imu_sensor_t imu;                ///< IMU 센서 (MPU6050)
static kalman_filter_t kalman_pitch;    ///< pitch 각도용 칼만 필터
static gps_sensor_t gps;                ///< GPS 센서
static encoder_sensor_t left_encoder;   ///< 좌측 바퀴 엔코더
static motor_control_t left_motor;      ///< 좌측 모터 제어
static encoder_sensor_t right_encoder;  ///< 우측 바퀴 엔코더
static motor_control_t right_motor;     ///< 우측 모터 제어
static ble_controller_t ble_controller; ///< BLE 무선 통신 컨트롤러
static balance_pid_t balance_pid;       ///< 밸런싱용 이중 루프 PID 제어기
static servo_standup_t servo_standup;   ///< 기립 보조용 서보 모터
static battery_sensor_t battery_sensor; ///< 배터리 전압 센서
/** @} */

/**
 * @defgroup SHARED_DATA 공유 데이터
 * @brief 태스크간 공유되는 로봇 상태 데이터(뮤텍스로 보호)
 * @{
 */
static float filtered_angle = 0.0f;     ///< 칼만 필터링된 pitch 각도 (degree)
static float robot_velocity = 0.0f;     ///< 로봇 이동 속도 (cm/s)
static bool balancing_enabled = true;   ///< 밸런싱 제어 활성화 플래그
/** @} */

static SemaphoreHandle_t data_mutex = NULL; ///< 공유 데이터 보호용 뮤텍스

/**
 * @defgroup TASK_HANDLES FreeRTOS 태스크 핸들
 * @brief 생성된 태스크들의 핸들
 * @{
 */
static TaskHandle_t balance_task_handle = NULL; ///< 밸런싱 제어 태스크 핸들
static TaskHandle_t sensor_task_handle = NULL;  ///< 센서 읽기 태스크 핸들
static TaskHandle_t status_task_handle = NULL;  ///< 상태 모니터링 태스크 핸들
/** @} */

/**
 * @defgroup FUNCTION_PROTOTYPES 함수 프로토타입
 * @brief 메인 파일 내부 함수들의 프로토타입 선언
 * @{
 */

/**
 * @brief 로봇 하드웨어 및 소프트웨어 구성 요소 초기화
 * 
 * 모든 센서, 액추에이터, 필터, 제어기를 초기화하고
 * 오류 복구 시스템을 설정합니다.
 */
static void initialize_robot(void);

/**
 * @brief 밸런싱 제어 태스크
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 50Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - 상태 머신 업데이트
 * - PID 제어 계산
 * - 모터 제어 명령 적용
 */
static void balance_task(void *pvParameters);

/**
 * @brief 센서 데이터 수집 태스크
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 50Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - IMU 센서 데이터 읽기
 * - 칼만 필터링
 * - GPS 데이터 업데이트
 * - 엔코더 속도 계산
 */
static void sensor_task(void *pvParameters);

/**
 * @brief 상태 모니터링 및 통신 태스크
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 1Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - BLE 상태 메시지 전송
 * - 시리얼 디버그 출력
 * - 시스템 상태 로깅
 */
static void status_task(void *pvParameters);

/**
 * @brief PID 출력과 원격 명령을 기반으로 모터 제어
 * @param motor_output PID 제어기 출력값(-255 ~ 255)
 * @param cmd 원격 제어 명령 구조체
 * 
 * PID 출력값에 조향 명령을 적용하여 좌우 모터 속도를 계산하고
 * 모터 제어 모듈에 명령을 전달합니다.
 */
static void update_motors(float motor_output, remote_command_t cmd);

/**
 * @brief 원격 제어 명령 처리
 * 
 * BLE로 수신된 원격 제어 명령을 분석하고 해당 동작을 실행합니다.
 * - 기립 명령 처리
 * - 밸런싱 활성화/비활성화
 */
static void handle_remote_commands(void);

/** @} */ // FUNCTION_PROTOTYPES

/**
 * @defgroup THREAD_SAFE_ACCESS 스레드 안전 데이터 접근 함수
 * @brief 뮤텍스를 사용한 공유 데이터 안전 접근 함수들
 * @{
 */

/**
 * @brief 칼만 필터링된 각도 값을 안전하게 읽기
 * @return float 현재 pitch 각도 (degree)
 */
static float get_filtered_angle(void);

/**
 * @brief 칼만 필터링된 각도 값을 안전하게 설정
 * @param angle 설정할 pitch 각도 (degree)
 */
static void set_filtered_angle(float angle);

/**
 * @brief 로봇 이동 속도를 안전하게 읽기
 * @return float 현재 이동 속도 (cm/s)
 */
static float get_robot_velocity(void);

/**
 * @brief 로봇 이동 속도를 안전하게 설정
 * @param velocity 설정할 이동 속도 (cm/s)
 */
static void set_robot_velocity(float velocity);

/**
 * @brief 밸런싱 활성화 상태를 안전하게 읽기
 * @return bool 밸런싱 활성화 여부 (true: 활성, false: 비활성)
 */
static bool get_balancing_enabled(void);

/**
 * @brief 밸런싱 활성화 상태를 안전하게 설정
 * @param enabled 밸런싱 활성화 여부 (true: 활성, false: 비활성)
 */
static void set_balancing_enabled(bool enabled);

/** @} */ // THREAD_SAFE_ACCESS

/**
 * @defgroup STATE_MACHINE 상태 머신 관리 함수
 * @brief 로봇 상태 머신의 상태 관리 및 전환 로직
 * @{
 */

/**
 * @brief 현재 로봇 상태를 안전하게 읽기
 * @return robot_state_t 현재 로봇 상태
 */
static robot_state_t get_robot_state(void);

/**
 * @brief 로봇 상태를 안전하게 변경
 * @param new_state 새로운 로봇 상태
 * 
 * 상태 변경을 로그로 출력하고 뮤텍스로 보호합니다.
 */
static void set_robot_state(robot_state_t new_state);

/**
 * @brief 로봇 상태를 문자열로 변환
 * @param state 변환할 로봇 상태
 * @return const char* 상태를 나타내는 문자열
 */
static const char* get_state_name(robot_state_t state);

/**
 * @brief 상태 머신 업데이트 및 상태 전환 처리
 * 
 * 현재 센서 데이터와 원격 명령을 기반으로 상태 전환 조건을 확인하고
 * 필요시 상태를 변경합니다.
 */
static void state_machine_update(void);

/** @} */ // STATE_MACHINE

/**
 * @brief ESP-IDF 애플리케이션 메인 함수
 * 
 * 시스템 초기화 및 태스크 생성, 메인 루프를 담당합니다
 * 
 * 초기화 순서:
 * 1. 뮤텍스 생성
 * 2. NVS 플래시 초기화
 * 3. 로봇 구성 요소 초기화
 * 4. FreeRTOS 태스크 생성
 * 5. 메인 루프 시작
 * 
 * 메인 루프에서는 다음 작업을 수행합니다:
 * - 센서 데이터 태스크 업데이트
 * - BLE 통신 업데이트
 * - 원격 명령 처리
 */
void app_main(void) {
    BSW_LOGI(TAG, "Balance Robot Starting...");

    // Create mutexes for data protection
    data_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL || state_mutex == NULL) {
        BSW_LOGE(TAG, "Failed to create mutexes!");
        set_robot_state(ROBOT_STATE_ERROR);
        esp_restart();
    }
    BSW_LOGI(TAG, "Mutexes created");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize robot components
    initialize_robot();

    // Set initial state to idle after successful initialization
    set_robot_state(ROBOT_STATE_IDLE);
    BSW_LOGI(TAG, "Robot initialized successfully!");
    
    // Create tasks
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &sensor_task_handle);
    xTaskCreate(balance_task, "balance_task", 4096, NULL, 4, &balance_task_handle);
    xTaskCreate(status_task, "status_task", 4096, NULL, 3, &status_task_handle);
    
    BSW_LOGI(TAG, "Tasks created, starting main loop...");
    
    // Main loop
    while (1) {
        // Update servo standup mechanism
        servo_standup_update(&servo_standup);
        
        // Update BLE
        ble_controller_update(&ble_controller);
        
        // Handle remote commands
        handle_remote_commands();
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief IMU 센서 초기화 래퍼 함수
 * 
 * config.h에 정의된 설정값을 사용하여 IMU 센서를 초기화하는 래퍼 함수입니다.
 * 오류 복구 시스템에 등록됩니다.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_imu_wrapper(void) {
    return imu_sensor_init(&imu, BSW_I2C_PORT_0, CONFIG_MPU6050_SDA_PIN, CONFIG_MPU6050_SCL_PIN);
}

/**
 * @brief 좌측 엔코더 초기화 래퍼 함수
 * 
 * config.h에 정의된 설정값을 사용하여 좌측 바퀴 엔코더를 초기화하는 래퍼 함수입니다.
 * CONFIG_ENABLE_LEFT_ENCODER가 정의되어 있을 때만 초기화됩니다.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_left_encoder_wrapper(void) {
#ifdef CONFIG_ENABLE_LEFT_ENCODER
    return encoder_sensor_init(&left_encoder, CONFIG_LEFT_ENC_A_PIN, CONFIG_LEFT_ENC_B_PIN, CONFIG_ENCODER_PPR, CONFIG_WHEEL_DIAMETER_CM);
#else
    BSW_LOGI(TAG, "Left encoder disabled in config.h");
    return ESP_OK;  // 비활성화된 것도 성공으로 처리
#endif
}

/**
 * @brief 우측 엔코더 초기화 래퍼 함수
 * 
 * config.h에 정의된 설정값을 사용하여 우측 바퀴 엔코더를 초기화하는 래퍼 함수입니다.
 * CONFIG_ENABLE_RIGHT_ENCODER가 정의되어 있을 때만 초기화됩니다.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_right_encoder_wrapper(void) {
#ifdef CONFIG_ENABLE_RIGHT_ENCODER
    return encoder_sensor_init(&right_encoder, CONFIG_RIGHT_ENC_A_PIN, CONFIG_RIGHT_ENC_B_PIN, CONFIG_ENCODER_PPR, CONFIG_WHEEL_DIAMETER_CM);
#else
    BSW_LOGI(TAG, "Right encoder disabled in config.h");
    return ESP_OK;  // 비활성화된 것도 성공으로 처리
#endif
}

/**
 * @brief GPS 센서 초기화 래퍼 함수
 * 
 * config.h에 정의된 설정값을 사용하여 GPS 센서를 초기화하는 래퍼 함수입니다.
 * 
 * NOTE: Temporarily disabled for testing without GPS hardware
 *       GPS UART RX polling task causes watchdog timeout without actual GPS module
 *       Also fixed argument order bug in gps_sensor.c: uart_driver_init(port, baudrate, tx, rx)
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
#if 0  // Disabled for hardware-less testing
static esp_err_t init_gps_wrapper(void) {
    return gps_sensor_init(&gps, CONFIG_GPS_UART_PORT, CONFIG_GPS_BAUDRATE, CONFIG_GPS_TX_PIN, CONFIG_GPS_RX_PIN);
}
#endif

/**
 * @brief 배터리 센서 초기화 래퍼 함수 (BSW ADC 사용)
 * 
 * config.h에 정의된 설정값을 사용하여 배터리 센서를 초기화하는 래퍼 함수입니다.
 * BSW ADC 드라이버를 사용하여 직접 레지스터 제어로 성능 최적화.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_battery_wrapper(void) {
    return battery_sensor_init(&battery_sensor, 
                              BSW_ADC_UNIT_1,                    // ADC1 사용
                              BSW_ADC_CHANNEL_3,                  // CH3 (GPIO3)
                              CONFIG_BATTERY_ADC_PIN,             // GPIO3
                              CONFIG_BATTERY_R1_KOHM, 
                              CONFIG_BATTERY_R2_KOHM);
}

/**
 * @brief BLE 컨트롤러 초기화 래퍼 함수
 * 
 * config.h에 정의된 설정값을 사용하여 BLE 컨트롤러를 초기화하는 래퍼 함수입니다.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_ble_wrapper(void) {
    return ble_controller_init(&ble_controller, CONFIG_BLE_DEVICE_NAME);
}

/**
 * @brief 서보 기립 시스템 초기화 래퍼 함수
 * 
 * config.h에 정의된 설정값을 사용하여 서보 기립 시스템을 초기화하는 래퍼 함수입니다.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_servo_wrapper(void) {
    return servo_standup_init(&servo_standup, CONFIG_SERVO_PIN, CONFIG_SERVO_CHANNEL, CONFIG_SERVO_EXTENDED_ANGLE, CONFIG_SERVO_RETRACTED_ANGLE);
}

/**
 * @brief 설정 관리자 초기화 래퍼 함수
 * 
 * NVS 기반 설정 관리자를 초기화하는 래퍼 함수입니다.
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_config_manager_wrapper(void) {
    return config_manager_init();
}

/**
 * @brief 로봇 하드웨어 및 소프트웨어 구성 요소 초기화
 * 
 * 모든 센서, 액추에이터, 필터, 제어기를 초기화하고
 * 오류 복구 시스템을 설정합니다.
 * 
 * 초기화 구성 요소:
 * - 오류 복구 시스템
 * - IMU 센서 (MPU6050)
 * - 좌우 엔코더 센서
 * - GPS 센서
 * - BLE 컨트롤러
 * - 서보 기립 시스템
 * - 칼만 필터
 * - 좌우 모터 제어기
 * - PID 제어기
 */
static void initialize_robot(void) {
    // Initialize BSW GPIO driver first (required by all other drivers)
    esp_err_t gpio_ret = bsw_gpio_init();
    if (gpio_ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize GPIO driver!");
        enter_safe_mode();
        return;
    }
    BSW_LOGI(TAG, "GPIO driver initialized");
    
    // Initialize error recovery system
    error_recovery_init();
    
    // Define component configurations
    component_info_t components[] = {
        {"Config_Manager", init_config_manager_wrapper, COMPONENT_CRITICAL, false, 0},
        {"IMU_Sensor", init_imu_wrapper, COMPONENT_OPTIONAL, false, 0},  // ⚠️ OPTIONAL로 변경 (하드웨어 미연결 시 테스트용)
        {"Left_Encoder", init_left_encoder_wrapper, COMPONENT_OPTIONAL, false, 0},  // ⚠️ OPTIONAL로 변경
        {"Right_Encoder", init_right_encoder_wrapper, COMPONENT_OPTIONAL, false, 0},  // ⚠️ OPTIONAL로 변경
        // GPS disabled for testing without hardware to prevent UART RX polling watchdog timeout
        // {"GPS_Sensor", init_gps_wrapper, COMPONENT_OPTIONAL, false, 0},
        {"Battery_Sensor", init_battery_wrapper, COMPONENT_IMPORTANT, false, 0},
        {"BLE_Controller", init_ble_wrapper, COMPONENT_IMPORTANT, false, 0},
        {"Servo_Standup", init_servo_wrapper, COMPONENT_IMPORTANT, false, 0}
    };
    
    int num_components = sizeof(components) / sizeof(components[0]);
    
    // Initialize each component with retry logic
    for (int i = 0; i < num_components; i++) {
        initialize_component_with_retry(&components[i]);
    }
    
    // Initialize Kalman filter with tuned parameters
    const tuning_params_t* params = config_manager_get_params();
    kalman_filter_init(&kalman_pitch);
    kalman_filter_set_angle(&kalman_pitch, 0.0f);
    // Apply tuned noise parameters
    kalman_pitch.Q_angle = params ? params->kalman_q_angle : CONFIG_KALMAN_Q_ANGLE;
    kalman_pitch.Q_bias = params ? params->kalman_q_bias : CONFIG_KALMAN_Q_BIAS;
    kalman_pitch.R_measure = params ? params->kalman_r_measure : CONFIG_KALMAN_R_MEASURE;
    BSW_LOGI(TAG, "Kalman filter initialized with tuned parameters");
    
    // Initialize motors (these are always critical)
    esp_err_t ret = motor_control_init(&left_motor, CONFIG_LEFT_MOTOR_A_PIN, CONFIG_LEFT_MOTOR_B_PIN, CONFIG_LEFT_MOTOR_EN_PIN, CONFIG_LEFT_MOTOR_CHANNEL);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize left motor!");
        enter_safe_mode();
        return;
    }
    BSW_LOGI(TAG, "Left motor initialized");
    
    ret = motor_control_init(&right_motor, CONFIG_RIGHT_MOTOR_A_PIN, CONFIG_RIGHT_MOTOR_B_PIN, CONFIG_RIGHT_MOTOR_EN_PIN, CONFIG_RIGHT_MOTOR_CHANNEL);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize right motor!");
        enter_safe_mode();
        return;
    }
    BSW_LOGI(TAG, "Right motor initialized");
    
    // Initialize PID controllers with tuned parameters
    balance_pid_init(&balance_pid);
    if (params) {
        balance_pid_set_balance_tunings(&balance_pid, params->balance_kp, params->balance_ki, params->balance_kd);
        balance_pid_set_velocity_tunings(&balance_pid, params->velocity_kp, params->velocity_ki, params->velocity_kd);
        balance_pid_set_max_tilt_angle(&balance_pid, params->max_tilt_angle);
    } else {
        // Fallback to config.h defaults
        balance_pid_set_balance_tunings(&balance_pid, CONFIG_BALANCE_PID_KP, CONFIG_BALANCE_PID_KI, CONFIG_BALANCE_PID_KD);
        balance_pid_set_velocity_tunings(&balance_pid, 1.0f, 0.1f, 0.0f);
        balance_pid_set_max_tilt_angle(&balance_pid, CONFIG_FALLEN_ANGLE_THRESHOLD);
    }
    balance_pid_set_target_velocity(&balance_pid, 0.0f);  // 정지 상태로 시작
    BSW_LOGI(TAG, "Balance PID controllers initialized with tuned parameters");
    
    // Log system health after initialization
    log_system_health();
}

/**
 * @brief 센서 데이터 수집 태스크
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 50Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - IMU 센서 데이터 읽기 및 칼만 필터링
 * - GPS 데이터 업데이트
 * - 엔코더 속도 계산
 * - 로봇 전체 이동 속도 계산 (좌우 바퀴 평균)
 * 
 * 이 태스크는 높은 우선순위(5)로 실행되어 정확한 센서 데이터 수집을 보장합니다.
 */
static void sensor_task(void *pvParameters) {
    BSW_LOGI(TAG, "Sensor task started");
    
    while (1) {
        // Update IMU
        esp_err_t ret = imu_sensor_update(&imu);
        if (ret == ESP_OK) {
            // Apply Kalman filter to pitch angle
            float dt = 0.01f; // 100Hz update rate
            set_filtered_angle(kalman_filter_get_angle(&kalman_pitch, 
                                                   imu_sensor_get_pitch(&imu),
                                                   imu_sensor_get_gyro_y(&imu), 
                                                   dt));
        }
        
        // Update GPS
        gps_sensor_update(&gps);
        
        // Update motor speeds (only if enabled)
        float left_speed = 0.0f, right_speed = 0.0f;
        
#ifdef CONFIG_ENABLE_LEFT_ENCODER
        encoder_sensor_update_speed(&left_encoder);
        left_speed = encoder_sensor_get_speed(&left_encoder);
#endif

#ifdef CONFIG_ENABLE_RIGHT_ENCODER
        encoder_sensor_update_speed(&right_encoder);
        right_speed = encoder_sensor_get_speed(&right_encoder);
#endif
        
        // Calculate robot velocity (average of enabled encoders)
#if defined(CONFIG_ENABLE_LEFT_ENCODER) && defined(CONFIG_ENABLE_RIGHT_ENCODER)
        set_robot_velocity((left_speed + right_speed) / 2.0f);
#elif defined(CONFIG_ENABLE_LEFT_ENCODER)
        set_robot_velocity(left_speed);
#elif defined(CONFIG_ENABLE_RIGHT_ENCODER)
        set_robot_velocity(right_speed);
#else
        set_robot_velocity(0.0f);  // 엔코더가 모두 비활성화된 경우
#endif
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

/**
 * @brief 밸런싱 제어 태스크
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 50Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - 상태 머신 업데이트 및 상태 전환 처리
 * - 현재 상태에 따른 제어 로직 실행
 * - PID 제어 계산 (밸런싱 상태에서)
 * - 모터 제어 명령 적용
 * 
 * 상태별 동작:
 * - IDLE: 모터 정지, PID 리셋
 * - BALANCING: PID 제어 기반 밸런싱
 * - STANDING_UP: 모터 정지, 서보 동작
 * - FALLEN/ERROR: 비상 정지
 */
static void balance_task(void *pvParameters) {
    BSW_LOGI(TAG, "Balance task started");

    while (1) {
        // Update state machine first
        state_machine_update();

        remote_command_t cmd = ble_controller_get_command(&ble_controller);
        robot_state_t state = get_robot_state();

        // Handle different robot states
        switch (state) {
        case ROBOT_STATE_IDLE:
            // Stop motors and reset PID
            motor_control_stop(&left_motor);
            motor_control_stop(&right_motor);
            balance_pid_reset(&balance_pid);
            break;

        case ROBOT_STATE_BALANCING:
            // Balance using dual-loop PID (angle + velocity control)
            // Compute balance control (dt = 20ms = 0.02s for 50Hz update rate)
            float dt = CONFIG_BALANCE_UPDATE_RATE / 1000.0f;
            
            // Get current sensor values
            float current_angle = get_filtered_angle();
            float current_velocity = get_robot_velocity();
            
            // Get angular velocity from IMU
            if (imu_sensor_update(&imu) == ESP_OK) {
                float gyro_rate = imu_sensor_get_gyro_y(&imu);  // Pitch angular velocity
                
                // Set target velocity based on remote command
                balance_pid_set_target_velocity(&balance_pid, cmd.direction * 10.0f);  // Scale command
                
                // Compute dual-loop balance control output
                float motor_output = balance_pid_compute_balance(&balance_pid, current_angle, gyro_rate, current_velocity, dt);
                
                // Apply motor commands
                update_motors(motor_output, cmd);
            } else {
                // Fallback: stop motors if sensor fails
                motor_control_stop(&left_motor);
                motor_control_stop(&right_motor);
            }
            break;

        case ROBOT_STATE_STANDING_UP:
            // Motors stopped during standup
            motor_control_stop(&left_motor);
            motor_control_stop(&right_motor);
            balance_pid_reset(&balance_pid);
            break;

        case ROBOT_STATE_FALLEN:
        case ROBOT_STATE_ERROR:
        default:
            // Emergency stop
            motor_control_stop(&left_motor);
            motor_control_stop(&right_motor);
            balance_pid_reset(&balance_pid);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz control loop
    }
}

/**
 * @brief 상태 모니터링 및 통신 태스크
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 1Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - BLE 연결 시 구조화된 상태 데이터 전송
 * - 시리얼 콘솔에 디버그 정보 출력
 * - GPS 수신 상태 및 좌표 정보 로깅
 * - 서보 기립 시스템 상태 모니터링
 * 
 * 전송되는 상태 정보:
 * - 기울어짐 각도, 이동 속도, 배터리 전압
 * - GPS 위치 정보 (수신 가능한 경우)
 * - 서보 동작 상태
 */
static void status_task(void *pvParameters) {
    BSW_LOGI(TAG, "Status task started");
    
    while (1) {
        // Send BLE status
        if (ble_controller_is_connected(&ble_controller)) {
            char status[128];
            snprintf(status, sizeof(status), "Angle:%.2f Vel:%.1f GPS:%s", 
                    get_filtered_angle(), get_robot_velocity(), 
                    gps_sensor_has_fix(&gps) ? "OK" : "NO");
            
            if (gps_sensor_has_fix(&gps)) {
                char gps_info[64];
                snprintf(gps_info, sizeof(gps_info), " Lat:%.6f Lon:%.6f", 
                        gps_sensor_get_latitude(&gps), gps_sensor_get_longitude(&gps));
                strncat(status, gps_info, sizeof(status) - strlen(status) - 1);
            }
            
            // Send structured status data instead of string
            float angle = get_filtered_angle();
            float velocity = get_robot_velocity();
            float battery_voltage = battery_sensor_read_voltage(&battery_sensor);
            ble_controller_send_status(&ble_controller, angle, velocity, battery_voltage);
        }
        
        // Print debug info to serial
        float battery_voltage = battery_sensor_read_voltage(&battery_sensor);
        int battery_percent = battery_sensor_get_percentage(&battery_sensor);
        
        BSW_LOGI(TAG, "Angle: %.2f | Velocity: %.2f | Battery: %.1fV(%d%%) | GPS: %s", 
                get_filtered_angle(), get_robot_velocity(), battery_voltage, battery_percent,
                gps_sensor_has_fix(&gps) ? "Valid" : "Invalid");
        
        if (gps_sensor_has_fix(&gps)) {
            BSW_LOGI(TAG, "GPS - Lat: %.6f | Lon: %.6f | Sats: %d", 
                    gps_sensor_get_latitude(&gps), 
                    gps_sensor_get_longitude(&gps),
                    gps_sensor_get_satellites(&gps));
        }
        
        BSW_LOGI(TAG, "Standup: %s", servo_standup_is_standing_up(&servo_standup) ? "Active" : "Idle");
        
        // Print tuning parameters occasionally (every 10 seconds)
        static int param_log_counter = 0;
        if (++param_log_counter >= 10) {
            param_log_counter = 0;
            char param_status[256];
            if (config_manager_get_status_string(param_status, sizeof(param_status)) == ESP_OK) {
                BSW_LOGI(TAG, "Tuning: %s", param_status);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz status updates
    }
}

/**
 * @brief PID 출력과 원격 명령을 기반으로 모터 제어
 * @param motor_output PID 제어기 출력값(-255 ~ 255)
 * @param cmd 원격 제어 명령 구조체
 * 
 * PID 출력값에 조향 명령을 적용하여 좌우 모터 속도를 계산하고
 * 모터 제어 모듈에 명령을 전달합니다.
 * 
 * 제어 로직:
 * 1. 회전 명령에 따른 조향 보정값 계산
 * 2. 좌측 모터 = PID 출력 - 조향 보정
 * 3. 우측 모터 = PID 출력 + 조향 보정
 * 4. 모터 속도 제한 (-255 ~ 255)
 */
static void update_motors(float motor_output, remote_command_t cmd) {
    // Apply turn adjustment
    float turn_adjustment = cmd.turn * 0.5f; // Scale turn command
    
    float left_motor_speed = motor_output - turn_adjustment;
    float right_motor_speed = motor_output + turn_adjustment;
    
    // Constrain motor speeds
    if (left_motor_speed > 255.0f) left_motor_speed = 255.0f;
    if (left_motor_speed < -255.0f) left_motor_speed = -255.0f;
    if (right_motor_speed > 255.0f) right_motor_speed = 255.0f;
    if (right_motor_speed < -255.0f) right_motor_speed = -255.0f;
    
    // Apply to motors
    motor_control_set_speed(&left_motor, (int)left_motor_speed);
    motor_control_set_speed(&right_motor, (int)right_motor_speed);
}

/**
 * @brief 원격 제어 명령 처리
 * 
 * BLE로 수신된 원격 제어 명령을 분석하고 해당 동작을 실행합니다.
 * - 기립 명령 처리: 서보 기립 시스템 활성화 및 상태 전송
 * - 밸런싱 활성화/비활성화: 밸런싱 제어 플래그 업데이트
 * - 설정 명령 처리: config_manager를 통한 파라미터 튜닝
 * 
 * 명령 처리 로직:
 * 1. BLE 컨트롤러에서 최신 명령 수신
 * 2. 텍스트 명령이 있으면 config_manager로 전달
 * 3. 기립 명령 확인 및 서보 동작 요청
 * 4. 밸런싱 상태 업데이트
 * 5. 필요시 상태 정보 BLE 전송
 */
static void handle_remote_commands(void) {
    remote_command_t cmd = ble_controller_get_command(&ble_controller);
    
    // Handle text commands for parameter tuning
    if (ble_controller_has_text_command(&ble_controller)) {
        const char* text_command = ble_controller_get_text_command(&ble_controller);
        if (text_command != NULL) {
            BSW_LOGI(TAG, "Processing text command: %s", text_command);
            esp_err_t ret = config_manager_handle_ble_command(text_command);
            if (ret == ESP_OK) {
                BSW_LOGI(TAG, "Text command processed successfully");
                // Parameter update successful - reload parameters for active components
                const tuning_params_t* params = config_manager_get_params();
                if (params) {
                    // Update PID parameters in real-time
                    balance_pid_set_balance_tunings(&balance_pid, params->balance_kp, params->balance_ki, params->balance_kd);
                    balance_pid_set_velocity_tunings(&balance_pid, params->velocity_kp, params->velocity_ki, params->velocity_kd);
                    balance_pid_set_max_tilt_angle(&balance_pid, params->max_tilt_angle);
                    
                    // Update Kalman filter parameters in real-time
                    kalman_pitch.Q_angle = params->kalman_q_angle;
                    kalman_pitch.Q_bias = params->kalman_q_bias;
                    kalman_pitch.R_measure = params->kalman_r_measure;
                    
                    BSW_LOGI(TAG, "Real-time parameter update completed");
                }
            } else {
                BSW_LOGE(TAG, "Failed to process text command: %d", ret);
            }
        }
    }
    
    // Handle standup command
    if (cmd.standup && !servo_standup_is_standing_up(&servo_standup)) {
        servo_standup_request_standup(&servo_standup);
        // Send status with standup indication via system_status field
        float angle = get_filtered_angle();
        float velocity = get_robot_velocity(); 
        float battery_voltage = battery_sensor_read_voltage(&battery_sensor);
        ble_controller_send_status(&ble_controller, angle, velocity, battery_voltage);
    }

    // Update balancing state
    set_balancing_enabled(cmd.balance);
}

/**
 * @brief 칼만 필터링된 각도 값을 안전하게 읽기
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 현재 pitch 각도를 읽습니다.
 * 
 * @return float 현재 pitch 각도 (degree)
 */
static float get_filtered_angle(void) {
    float angle = 0.0f;
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        angle = filtered_angle;
        xSemaphoreGive(data_mutex);
    }
    return angle;
}

/**
 * @brief 칼만 필터링된 각도 값을 안전하게 설정
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 pitch 각도를 업데이트합니다.
 * 
 * @param angle 설정할 pitch 각도 (degree)
 */
static void set_filtered_angle(float angle) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        filtered_angle = angle;
        xSemaphoreGive(data_mutex);
    }
}

/**
 * @brief 로봇 이동 속도를 안전하게 읽기
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 현재 이동 속도를 읽습니다.
 * 
 * @return float 현재 이동 속도 (cm/s)
 */
static float get_robot_velocity(void) {
    float velocity = 0.0f;
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        velocity = robot_velocity;
        xSemaphoreGive(data_mutex);
    }
    return velocity;
}

/**
 * @brief 로봇 이동 속도를 안전하게 설정
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 이동 속도를 업데이트합니다.
 * 
 * @param velocity 설정할 이동 속도 (cm/s)
 */
static void set_robot_velocity(float velocity) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        robot_velocity = velocity;
        xSemaphoreGive(data_mutex);
    }
}

/**
 * @brief 밸런싱 활성화 상태를 안전하게 읽기
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 밸런싱 활성화 상태를 읽습니다.
 * 
 * @return bool 밸런싱 활성화 여부 (true: 활성, false: 비활성)
 */
static bool __attribute__((unused)) get_balancing_enabled(void) {
    bool enabled = false;
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        enabled = balancing_enabled;
        xSemaphoreGive(data_mutex);
    }
    return enabled;
}

/**
 * @brief 밸런싱 활성화 상태를 안전하게 설정
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 밸런싱 활성화 상태를 업데이트합니다.
 * 
 * @param enabled 밸런싱 활성화 여부 (true: 활성, false: 비활성)
 */
static void set_balancing_enabled(bool enabled) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        balancing_enabled = enabled;
        xSemaphoreGive(data_mutex);
    }
}

/**
 * @brief 현재 로봇 상태를 안전하게 읽기
 * 
 * 뮤텍스를 사용하여 스레드 안전하게 현재 로봇 상태를 읽습니다.
 * 
 * @return robot_state_t 현재 로봇 상태
 */
static robot_state_t get_robot_state(void) {
    robot_state_t state = ROBOT_STATE_ERROR;
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_state;
        xSemaphoreGive(state_mutex);
    }
    return state;
}

/**
 * @brief 로봇 상태를 안전하게 변경
 * @param new_state 새로운 로봇 상태
 * 
 * 상태 변경을 로그로 출력하고 뮤텍스로 보호합니다.
 * 동일한 상태로의 변경은 로그를 출력하지 않습니다.
 */
static void set_robot_state(robot_state_t new_state) {
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        if (current_state != new_state) {
            BSW_LOGI(TAG, "State change: %s -> %s",
                    get_state_name(current_state), get_state_name(new_state));
            current_state = new_state;
        }
        xSemaphoreGive(state_mutex);
    }
}

/**
 * @brief 로봇 상태를 문자열로 변환
 * @param state 변환할 로봇 상태
 * @return const char* 상태를 나타내는 문자열
 * 
 * 로깅 및 디버깅 목적으로 상태 열거값을 읽기 쉬운 문자열로 변환합니다.
 */
static const char* get_state_name(robot_state_t state) {
    switch (state) {
        case ROBOT_STATE_INIT: return "INIT";
        case ROBOT_STATE_IDLE: return "IDLE";
        case ROBOT_STATE_BALANCING: return "BALANCING";
        case ROBOT_STATE_STANDING_UP: return "STANDING_UP";
        case ROBOT_STATE_FALLEN: return "FALLEN";
        case ROBOT_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 상태 머신 업데이트 및 상태 전환 처리
 * 
 * 현재 센서 데이터와 원격 명령을 기반으로 상태 전환 조건을 확인하고
 * 필요시 상태를 변경합니다.
 * 
 * 상태 전환 조건:
 * - IDLE → BALANCING: 밸런싱 명령 수신 (기립 중이 아닐 때)
 * - IDLE → STANDING_UP: 기립 명령 수신
 * - BALANCING → IDLE: 밸런싱 명령 해제
 * - BALANCING → STANDING_UP: 기립 명령 수신
 * - any → FALLEN: 기울어짐 각도가 임계값 초과
 * - FALLEN → STANDING_UP: 기립 명령 수신 (복구 시도)
 * - STANDING_UP → IDLE: 기립 완료 또는 실패
 */
static void state_machine_update(void) {
    robot_state_t current = get_robot_state();
    float angle = get_filtered_angle();
    remote_command_t cmd = ble_controller_get_command(&ble_controller);

    // State transitions based on conditions
    switch (current) {
    case ROBOT_STATE_IDLE:
        if (cmd.balance && !servo_standup_is_standing_up(&servo_standup)) {
            set_robot_state(ROBOT_STATE_BALANCING);
        } else if (cmd.standup) {
            set_robot_state(ROBOT_STATE_STANDING_UP);
        }
        // Check if fallen (angle too large)
        if (fabsf(angle) > CONFIG_FALLEN_ANGLE_THRESHOLD) {
            set_robot_state(ROBOT_STATE_FALLEN);
        }
        break;

    case ROBOT_STATE_BALANCING:
        if (!cmd.balance) {
            set_robot_state(ROBOT_STATE_IDLE);
        } else if (cmd.standup) {
            set_robot_state(ROBOT_STATE_STANDING_UP);
        } else if (fabsf(angle) > CONFIG_FALLEN_ANGLE_THRESHOLD) {
            set_robot_state(ROBOT_STATE_FALLEN);
        }
        break;

    case ROBOT_STATE_STANDING_UP:
        if (servo_standup_is_complete(&servo_standup)) {
            set_robot_state(ROBOT_STATE_IDLE);
        } else if (!servo_standup_is_standing_up(&servo_standup)) {
            // Standup failed or cancelled
            set_robot_state(ROBOT_STATE_IDLE);
        }
        break;

    case ROBOT_STATE_FALLEN:
        // Can only recover through standup
        if (cmd.standup) {
            set_robot_state(ROBOT_STATE_STANDING_UP);
        }
        break;

    case ROBOT_STATE_ERROR:
        // Manual recovery required - could add auto-recovery logic here
        break;

    default:
        set_robot_state(ROBOT_STATE_ERROR);
        break;
    }
}
