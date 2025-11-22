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
 * - sensor_task: 센서 데이터 수집 및 필터링(100Hz)
 * - balance_task: PID 제어 및 모터 제어 (100Hz)
 * - status_task: 상태 모니터링 및 BLE 통신 (1Hz)
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "config.h"
#include "bsw/system_services.h"
#include "bsw/i2c_driver.h"
#include "bsw/uart_driver.h"
#include "bsw/gpio_driver.h"
#include "bsw/adc_driver.h"
#include "bsw/pwm_driver.h"

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
#include "input/button_driver.h"
#include "output/rgb_led.h"
#include "esp_timer.h"
#include "esp_task_wdt.h" // Watchdog Timer

// Pin definitions are now in config.h

static const char* TAG = "BALANCE_ROBOT"; ///< ESP-IDF 로깅 태그

/**
 * @struct telemetry_data_t
 * @brief 고속 로깅용 데이터 구조체
 */
typedef struct __attribute__((packed)) {
    float angle;
    float velocity;
    float motor_output;
} telemetry_data_t;

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

typedef enum { MODE_REMOTE_CONTROL, MODE_GPS_FOLLOWING } robot_mode_t;
static robot_mode_t robot_mode = MODE_REMOTE_CONTROL;
static button_t mode_btn;

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
static TaskHandle_t control_task_handle = NULL; ///< 통합 제어 태스크 핸들
static TaskHandle_t status_task_handle = NULL;  ///< 상태 모니터링 태스크 핸들
/** @} */

// Navigation State
static double est_lat = 0.0, est_lon = 0.0;
static bool is_pos_initialized = false;
static float robot_heading = 0.0f; // 0=North, 90=East
static double target_lat = 0.0, target_lon = 0.0;

// Helper functions for navigation
static float normalize_angle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

static double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // Earth radius in meters
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

static double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double y = sin(dLon) * cos(lat2 * M_PI / 180.0);
    double x = cos(lat1 * M_PI / 180.0) * sin(lat2 * M_PI / 180.0) -
               sin(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * cos(dLon);
    return atan2(y, x) * 180.0 / M_PI;
}

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
 * @brief 통합 제어 태스크 (센서 + 밸런싱)
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 100Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - 센서 데이터 수집 (IMU, GPS, Encoder)
 * - 칼만 필터링
 * - 상태 머신 업데이트
 * - PID 제어 및 모터 출력
 */
static void control_task(void *pvParameters);

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
static void apply_stored_pitch_offset(const tuning_params_t* params);
static void enter_imu_error_state(const char* reason);
static void verify_control_polarity(float pitch, float motor_output);
static void imu_pitch_polarity_probe(float pitch);
static void set_filtered_angle(float angle);
static void set_robot_state(robot_state_t new_state);



/**
 * @brief 자이로스코프 오프셋 캘리브레이션
 * 
 * 로봇이 정지 상태일 때 자이로스코프의 초기 오프셋을 측정하여 설정합니다.
 * 초기화 시 호출되어야 합니다.
 */
static esp_err_t calibrate_pitch_offset_and_store(void) {
    if (!imu_sensor_is_initialized(&imu)) {
        BSW_LOGE(TAG, "Cannot calibrate pitch offset: IMU not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    BSW_LOGI(TAG, "Pitch calibration started. Keep the robot perfectly upright and still for a few seconds...");

    set_robot_state(ROBOT_STATE_IDLE);
    motor_control_stop(&left_motor);
    motor_control_stop(&right_motor);
    balance_pid_reset(&balance_pid);
    vTaskDelay(pdMS_TO_TICKS(200));

    float sum_pitch = 0.0f;
    for (int i = 0; i < CONFIG_PITCH_CALIBRATION_SAMPLES; i++) {
        esp_err_t ret = imu_sensor_update(&imu);
        if (ret != ESP_OK) {
            BSW_LOGE(TAG, "IMU read failed during calibration (%d)", ret);
            return ret;
        }
        sum_pitch += imu_sensor_get_pitch(&imu);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    float offset = sum_pitch / CONFIG_PITCH_CALIBRATION_SAMPLES;
    imu_sensor_set_pitch_offset(&imu, offset);
    set_filtered_angle(0.0f);

    esp_err_t store_ret = config_manager_set_param(CONFIG_PARAM_PITCH_OFFSET, offset, true);
    if (store_ret != ESP_OK) {
        BSW_LOGW(TAG, "Failed to persist pitch offset (err=%d)", store_ret);
    }

    BSW_LOGI(TAG, "Pitch calibration complete. Stored offset: %.2f°", offset);
    return ESP_OK;
}

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
    
    // Initialize Task Watchdog Timer (TWDT)
    const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000, // Keep 10s to prevent boot loop during heavy init
        .trigger_panic = true,
    };
    
    // [Fix for IDF v5.5.1] Try reconfigure FIRST to avoid "TWDT already initialized" error
    esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_config);
    
    if (wdt_err == ESP_ERR_INVALID_STATE) {
        // Not initialized yet, so initialize it
        BSW_LOGI(TAG, "TWDT not active, initializing...");
        ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));
    } else {
        // Reconfigured successfully (or other error caught by CHECK)
        ESP_ERROR_CHECK(wdt_err);
        BSW_LOGI(TAG, "Task Watchdog Timer reconfigured to 10s");
    }
    BSW_LOGI(TAG, "Task Watchdog Timer initialized (10s)");

    // Register main task to WDT immediately so we can reset it during init
    esp_task_wdt_add(NULL);

    // Initialize robot components
    BSW_LOGI(TAG, "Initializing robot components...");
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow logs to flush
    
    // Initialize BSW GPIO driver first (required by all other drivers)
    esp_err_t gpio_ret = bsw_gpio_init();
    if (gpio_ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize GPIO driver!");
        set_robot_state(ROBOT_STATE_ERROR);
        esp_restart();
    }
    BSW_LOGI(TAG, "GPIO driver initialized");

    initialize_robot();

    // Set initial state to idle after successful initialization
    set_robot_state(ROBOT_STATE_IDLE);
    BSW_LOGI(TAG, "Robot initialized successfully!");
    
    // Create tasks
    // Priority 5 (High) for control task to ensure real-time performance
    xTaskCreate(control_task, "control_task", 8192, NULL, 5, &control_task_handle);
    xTaskCreate(status_task, "status_task", 4096, NULL, 3, &status_task_handle);
    
    BSW_LOGI(TAG, "Tasks created, starting main loop...");
    
    // Main loop
    while (1) {
        esp_task_wdt_reset();

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
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
static esp_err_t init_gps_wrapper(void) {
    return gps_sensor_init(&gps, BSW_UART_PORT_1, (bsw_gpio_num_t)CONFIG_GPS_TX_PIN, (bsw_gpio_num_t)CONFIG_GPS_RX_PIN, CONFIG_GPS_BAUDRATE);
}

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
    return servo_standup_init(&servo_standup, (bsw_gpio_num_t)CONFIG_SERVO_PIN, (pwm_channel_t)CONFIG_SERVO_CHANNEL, CONFIG_SERVO_EXTENDED_ANGLE, CONFIG_SERVO_RETRACTED_ANGLE);
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
    // GPIO driver is already initialized in app_main()
    
    // Initialize PWM driver (required by Servo and Motors)
    esp_err_t pwm_ret = pwm_driver_init();
    if (pwm_ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize PWM driver!");
        enter_safe_mode();
        return;
    }
    BSW_LOGI(TAG, "PWM driver initialized");
    
    // Initialize error recovery system
    error_recovery_init();
    
    // Define component configurations
    component_info_t components[] = {
        {"Config_Manager", init_config_manager_wrapper, COMPONENT_CRITICAL, false, 0},
        {"IMU_Sensor", init_imu_wrapper, COMPONENT_OPTIONAL, false, 0},  // ⚠️ OPTIONAL로 변경 (하드웨어 미연결 시 테스트용)
        {"Left_Encoder", init_left_encoder_wrapper, COMPONENT_OPTIONAL, false, 0},  // ⚠️ OPTIONAL로 변경
        {"Right_Encoder", init_right_encoder_wrapper, COMPONENT_OPTIONAL, false, 0},  // ⚠️ OPTIONAL로 변경
        {"GPS_Sensor", init_gps_wrapper, COMPONENT_OPTIONAL, false, 0},
        {"Battery_Sensor", init_battery_wrapper, COMPONENT_IMPORTANT, false, 0},
        {"BLE_Controller", init_ble_wrapper, COMPONENT_IMPORTANT, false, 0}
        // Servo_Standup moved to after motor initialization to avoid PWM timer conflict
    };
    
    int num_components = sizeof(components) / sizeof(components[0]);
    
    // Initialize each component with retry logic
    for (int i = 0; i < num_components; i++) {
        esp_task_wdt_reset(); // Feed WDT before each component init
        
        // [Power Stability] Add delay between component initializations to reduce inrush current
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        BSW_LOGI(TAG, "Starting init for %s...", components[i].name);
        bool init_result = initialize_component_with_retry(&components[i]);
        BSW_LOGI(TAG, "Finished init for %s: %s", components[i].name, init_result ? "SUCCESS" : "FAILED");
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
    
    apply_stored_pitch_offset(params);
    
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
    
    // Initialize Servo Standup (Moved here to avoid PWM timer conflict with motors)
    // Motors (High Freq) must be initialized before Servo (Low Freq) on ESP32-C6
    component_info_t servo_component = {"Servo_Standup", init_servo_wrapper, COMPONENT_IMPORTANT, false, 0};
    BSW_LOGI(TAG, "Starting init for Servo_Standup...");
    bool servo_init_result = initialize_component_with_retry(&servo_component);
    BSW_LOGI(TAG, "Finished init for Servo_Standup: %s", servo_init_result ? "SUCCESS" : "FAILED");

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
    
    // Initialize UI (Button & LED)
    button_init(&mode_btn, CONFIG_BUTTON_PIN);
    rgb_led_init(CONFIG_LED_R_PIN, CONFIG_LED_G_PIN, CONFIG_LED_B_PIN);
    rgb_led_set_color(LED_COLOR_BLUE); // Default: Remote Control Mode
    BSW_LOGI(TAG, "UI initialized");

    // Log system health after initialization
    log_system_health();
}

/**
 * @brief 통합 제어 태스크 (센서 + 밸런싱)
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용안함)
 * 
 * 100Hz 주기로 실행되며 다음 작업을 수행합니다:
 * - 센서 데이터 수집 (IMU, GPS, Encoder)
 * - 칼만 필터링
 * - 상태 머신 업데이트
 * - PID 제어 및 모터 출력
 */
static void control_task(void *pvParameters) {
    BSW_LOGI(TAG, "Control task started");
    
    // Register this task with WDT
    esp_task_wdt_add(NULL);
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms period (100Hz)
    
    // Initialize wake time
    xLastWakeTime = xTaskGetTickCount();
    
    static int64_t last_time_us = 0;
    static uint8_t telemetry_counter = 0;
    static uint8_t imu_failure_count = 0;

    while (1) {
        // 1. Ensure periodic execution (more precise than vTaskDelay)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Feed WDT
        esp_task_wdt_reset();

        // 2. Calculate actual dt
        int64_t current_time_us = esp_timer_get_time();
        if (last_time_us == 0) {
            last_time_us = current_time_us;
            continue;
        }
        float dt = (float)(current_time_us - last_time_us) / 1000000.0f;
        last_time_us = current_time_us;

        // --- Sensor Update Section ---
        
        // IMU Update
       esp_err_t imu_status = imu_sensor_update(&imu);
       if (imu_status == ESP_OK) {
           imu_failure_count = 0;
           float pitch = imu_sensor_get_pitch(&imu);
           float gyro_y = imu_sensor_get_gyro_y(&imu);

           imu_pitch_polarity_probe(pitch);
             
           // Kalman Filter Update with actual dt
           float filtered = kalman_filter_get_angle(&kalman_pitch, pitch, gyro_y, dt);
           set_filtered_angle(filtered);
       } else {
           imu_failure_count++;
           BSW_LOGW(TAG, "IMU update failed (%d/%d) Error: %d", imu_failure_count, CONFIG_IMU_MAX_CONSECUTIVE_FAILURES, imu_status);
             if (imu_failure_count >= CONFIG_IMU_MAX_CONSECUTIVE_FAILURES) {
                 enter_imu_error_state("IMU communication lost");
                 imu_failure_count = 0;
                 
                 // Attempt recovery
                 BSW_LOGW(TAG, "Attempting IMU recovery...");
                 esp_err_t recovery_ret = imu_sensor_init(&imu, BSW_I2C_PORT_0, CONFIG_MPU6050_SDA_PIN, CONFIG_MPU6050_SCL_PIN);
                 if (recovery_ret == ESP_OK) {
                     BSW_LOGI(TAG, "IMU recovery successful!");
                     // Reset state to IDLE if recovery succeeds
                     set_robot_state(ROBOT_STATE_IDLE);
                 } else {
                     BSW_LOGE(TAG, "IMU recovery failed: %d", recovery_ret);
                 }
             }
           continue;
       }

        // GPS Update
        gps_sensor_update(&gps);
        if (gps_sensor_has_fix(&gps)) {
             est_lat = gps_sensor_get_latitude(&gps);
             est_lon = gps_sensor_get_longitude(&gps);
             is_pos_initialized = true;
             
             float gps_speed_ms = gps_sensor_get_speed(&gps) / 3.6f; // km/h -> m/s
             if (gps_speed_ms > CONFIG_GPS_HEADING_MIN_SPEED_MS) {
                 robot_heading = gps_sensor_get_course(&gps);
             }
        }

        // Encoder Update
        float left_speed = 0.0f;
        float right_speed = 0.0f;

#ifdef CONFIG_ENABLE_LEFT_ENCODER
        encoder_sensor_update_speed(&left_encoder);
        left_speed = encoder_sensor_get_speed(&left_encoder);
#endif

#ifdef CONFIG_ENABLE_RIGHT_ENCODER
        encoder_sensor_update_speed(&right_encoder);
        right_speed = encoder_sensor_get_speed(&right_encoder);
#endif
        
        // Calculate robot velocity
#if defined(CONFIG_ENABLE_LEFT_ENCODER) && defined(CONFIG_ENABLE_RIGHT_ENCODER)
        set_robot_velocity((left_speed + right_speed) / 2.0f);
#elif defined(CONFIG_ENABLE_LEFT_ENCODER)
        set_robot_velocity(left_speed);
#elif defined(CONFIG_ENABLE_RIGHT_ENCODER)
        set_robot_velocity(right_speed);
#else
        set_robot_velocity(0.0f);
#endif

        // --- Control Logic Section ---

        // Button Event Handling
        button_event_t evt = button_get_event(&mode_btn);
        if (evt == BUTTON_EVENT_CLICK) {
            robot_mode = (robot_mode == MODE_REMOTE_CONTROL) ? MODE_GPS_FOLLOWING : MODE_REMOTE_CONTROL;
            rgb_led_set_color(robot_mode == MODE_REMOTE_CONTROL ? LED_COLOR_BLUE : LED_COLOR_GREEN);
            BSW_LOGI(TAG, "Mode switched to: %s", robot_mode == MODE_REMOTE_CONTROL ? "REMOTE" : "GPS");
        } else if (evt == BUTTON_EVENT_LONG_PRESS) {
            if (!servo_standup_is_standing_up(&servo_standup)) {
                servo_standup_request_standup(&servo_standup);
                set_robot_state(ROBOT_STATE_STANDING_UP);
                rgb_led_set_color(LED_COLOR_YELLOW);
                BSW_LOGI(TAG, "Standup requested via button");
            }
        }

        // Standup completion check
        // Start balancing as soon as the servo starts retracting (robot is upright)
        if ((servo_standup_is_complete(&servo_standup) || servo_standup_is_retracting(&servo_standup)) && 
            get_robot_state() == ROBOT_STATE_STANDING_UP) {
            set_robot_state(ROBOT_STATE_BALANCING);
            rgb_led_set_color(robot_mode == MODE_REMOTE_CONTROL ? LED_COLOR_BLUE : LED_COLOR_GREEN);
        }

        // State Machine Update
        state_machine_update();

        remote_command_t cmd = ble_controller_get_command(&ble_controller);
        robot_state_t state = get_robot_state();
        float current_motor_output = 0.0f;

        switch (state) {
        case ROBOT_STATE_IDLE:
            motor_control_stop(&left_motor);
            motor_control_stop(&right_motor);
            balance_pid_reset(&balance_pid);
            break;

        case ROBOT_STATE_BALANCING: {
            float current_angle = get_filtered_angle();
            float current_velocity = get_robot_velocity();
            float gyro_rate = imu_sensor_get_gyro_y(&imu); // Already updated above

            float target_speed = 0.0f;
            float target_turn = 0.0f;

            if (robot_mode == MODE_REMOTE_CONTROL) {
                target_speed = cmd.direction * 10.0f;
                target_turn = cmd.turn * 0.5f;
            } else if (robot_mode == MODE_GPS_FOLLOWING) {
                if (is_pos_initialized && (target_lat != 0.0 || target_lon != 0.0)) {
                    double dist = calculate_distance(est_lat, est_lon, target_lat, target_lon);
                    double bearing = calculate_bearing(est_lat, est_lon, target_lat, target_lon);
                    float heading_error = normalize_angle(bearing - robot_heading);

                    if (dist < 2.0) {
                        rgb_led_set_color(LED_COLOR_YELLOW);
                        target_speed = 0.0f;
                        target_turn = 0.0f;
                    } else {
                        target_turn = heading_error * 2.0f; 
                        if (target_turn > 30.0f) target_turn = 30.0f;
                        if (target_turn < -30.0f) target_turn = -30.0f;

                        if (fabs(heading_error) < 45.0f) {
                            target_speed = 15.0f;
                        } else {
                            target_speed = 0.0f;
                        }
                    }
                }
            }
            
            balance_pid_set_target_velocity(&balance_pid, target_speed);
            
            // Compute dual-loop balance control output with ACTUAL dt
            float motor_output = balance_pid_compute_balance(&balance_pid, current_angle, gyro_rate, current_velocity, dt);
            current_motor_output = motor_output;

            verify_control_polarity(current_angle, motor_output);
            
            remote_command_t temp_cmd = {0};
            temp_cmd.turn = (int8_t)target_turn; 
            update_motors(motor_output, temp_cmd);
            break;
        }

        case ROBOT_STATE_STANDING_UP:
            motor_control_stop(&left_motor);
            motor_control_stop(&right_motor);
            balance_pid_reset(&balance_pid);
            break;

        case ROBOT_STATE_FALLEN:
        case ROBOT_STATE_ERROR:
        default:
            motor_control_stop(&left_motor);
            motor_control_stop(&right_motor);
            balance_pid_reset(&balance_pid);
            break;
        }

        // Telemetry (20Hz = 100Hz / 5)
        if (++telemetry_counter >= 5) {
            telemetry_counter = 0;
            if (ble_controller_is_connected(&ble_controller)) {
                telemetry_data_t data;
                data.angle = get_filtered_angle();
                data.velocity = get_robot_velocity();
                data.motor_output = current_motor_output;
                ble_controller_send_raw(&ble_controller, (uint8_t*)&data, sizeof(data));
            }
        }
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
    
    // Get current battery voltage for compensation / cutoff
    static float filtered_battery_voltage = 0.0f;
    static bool battery_filter_initialized = false;

    float battery_voltage_raw = battery_sensor_read_voltage(&battery_sensor);

    if (battery_voltage_raw <= CONFIG_BATTERY_CRITICAL_THRESHOLD) {
        motor_control_stop(&left_motor);
        motor_control_stop(&right_motor);
        balance_pid_reset(&balance_pid);
        set_robot_state(ROBOT_STATE_IDLE);
        BSW_LOGW(TAG, "Battery cutoff active (%.2fV <= %.2fV). Motors disabled.",
                 battery_voltage_raw, CONFIG_BATTERY_CRITICAL_THRESHOLD);
        return;
    }

    float compensated_voltage = battery_voltage_raw;
    if (battery_voltage_raw > 0.0f) {
        if (!battery_filter_initialized) {
            filtered_battery_voltage = battery_voltage_raw;
            battery_filter_initialized = true;
        } else {
            filtered_battery_voltage += CONFIG_BATTERY_VOLTAGE_LPF_ALPHA * (battery_voltage_raw - filtered_battery_voltage);
        }
        compensated_voltage = filtered_battery_voltage;
    }

    // Apply to motors with voltage compensation
    motor_control_set_speed_compensated(&left_motor, (int)left_motor_speed, compensated_voltage);
    motor_control_set_speed_compensated(&right_motor, (int)right_motor_speed, compensated_voltage);
}

static void apply_stored_pitch_offset(const tuning_params_t* params) {
    float offset = (params != NULL) ? params->pitch_offset_deg : 0.0f;
    imu_sensor_set_pitch_offset(&imu, offset);
    BSW_LOGI(TAG, "Pitch offset loaded from NVS: %.2f°", offset);
}

static void enter_imu_error_state(const char* reason) {
    motor_control_stop(&left_motor);
    motor_control_stop(&right_motor);
    balance_pid_reset(&balance_pid);
    set_robot_state(ROBOT_STATE_ERROR);
    rgb_led_set_color(LED_COLOR_RED);
    BSW_LOGE(TAG, "IMU FAILSAFE engaged: %s", reason ? reason : "Unknown");
}

static void verify_control_polarity(float pitch, float motor_output) {
    static bool polarity_checked = false;
    if (polarity_checked) return;

    if (fabsf(pitch) < 2.0f || fabsf(motor_output) < 15.0f) {
        return; // wait for meaningful motion
    }

    bool same_sign = (pitch > 0.0f && motor_output > 0.0f) ||
                     (pitch < 0.0f && motor_output < 0.0f);

    if (same_sign) {
        BSW_LOGI(TAG, "Control polarity OK: pitch=%.2f°, cmd=%.2f (matching sign)", pitch, motor_output);
    } else {
        BSW_LOGE(TAG, "Control polarity mismatch: pitch=%.2f°, cmd=%.2f. Check PID sign or swap motor leads!", pitch, motor_output);
    }

    polarity_checked = true;
}

static void imu_pitch_polarity_probe(float pitch) {
    static bool pitch_polarity_logged = false;
    if (pitch_polarity_logged) return;

    if (fabsf(pitch) < 3.0f) {
        return; // need a deliberate tilt
    }

    BSW_LOGI(TAG,
             "IMU pitch polarity sample: %.2f°. Tilt the robot forward when capturing this log. Positive values must mean forward lean.",
             pitch);
    pitch_polarity_logged = true;
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
            bool command_handled = false;
            if (strcasecmp(text_command, "CAL_PITCH") == 0) {
                command_handled = true;
                esp_err_t cal_ret = calibrate_pitch_offset_and_store();
                if (cal_ret == ESP_OK) {
                    const tuning_params_t* params = config_manager_get_params();
                    apply_stored_pitch_offset(params);
                    BSW_LOGI(TAG, "Pitch offset calibration applied successfully");
                } else {
                    BSW_LOGE(TAG, "Pitch calibration failed (err=%d)", cal_ret);
                }
            }

            if (!command_handled) {
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

    // [추가됨] 앱 명령에 따라 주행 모드 전환
    // 앱에서 GPS 버튼을 켜면 -> GPS 모드, 끄면 -> 리모트 모드
    if (cmd.gps_mode) {
        if (robot_mode != MODE_GPS_FOLLOWING) {
            robot_mode = MODE_GPS_FOLLOWING;
            rgb_led_set_color(LED_COLOR_GREEN); // GPS 모드는 초록색
            BSW_LOGI(TAG, "Mode switched to GPS (via App)");
        }
    } else {
        if (robot_mode != MODE_REMOTE_CONTROL) {
            robot_mode = MODE_REMOTE_CONTROL;
            rgb_led_set_color(LED_COLOR_BLUE); // 일반 모드는 파란색
            BSW_LOGI(TAG, "Mode switched to Remote (via App)");
        }
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
        } else if (cmd.standup && !servo_standup_is_standing_up(&servo_standup)) {
            set_robot_state(ROBOT_STATE_STANDING_UP);
        } else if (fabsf(angle) > CONFIG_FALLEN_ANGLE_THRESHOLD) {
            set_robot_state(ROBOT_STATE_FALLEN);
        }
        break;

    case ROBOT_STATE_STANDING_UP:
        if (servo_standup_is_complete(&servo_standup)) {
            set_robot_state(ROBOT_STATE_BALANCING);
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
