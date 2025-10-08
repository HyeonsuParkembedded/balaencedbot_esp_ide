# BalanceBot ESP32-C6 개발자 사양서

**버전:** 1.0  
**날짜:** 2025-10-08  
**대상:** 임베디드 소프트웨어 개발자  

---

## 📋 목차

1. [시스템 아키텍처](#시스템-아키텍처)
2. [BLE 컨트롤러 API](#ble-컨트롤러-api)
3. [프로토콜 시스템](#프로토콜-시스템)
4. [설정 관리자](#설정-관리자)
5. [BSW 계층](#bsw-계층)
6. [통합 및 사용법](#통합-및-사용법)
7. [디버깅 가이드](#디버깅-가이드)
8. [테스트 및 검증](#테스트-및-검증)

---

## 시스템 아키텍처

### 전체 구조

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Application Layer                           │
├─────────────────────────────────────────────────────────────────────┤
│  main.c  │  BLE Controller  │  Config Manager  │  Protocol System  │
├─────────────────────────────────────────────────────────────────────┤
│                     Logic Layer (PID, Control)                      │
├─────────────────────────────────────────────────────────────────────┤
│              BSW (Board Support Wrapper) Layer                      │
│  BLE Driver  │  System Services  │  I2C Driver  │  PWM Driver  │
├─────────────────────────────────────────────────────────────────────┤
│                       ESP-IDF/Hardware                              │
└─────────────────────────────────────────────────────────────────────┘
```

### 모듈 간 의존성

- **BLE Controller** ← BSW BLE Driver, Protocol System
- **Config Manager** ← ESP-IDF NVS, BLE Controller  
- **Main Application** ← 모든 모듈
- **Protocol System** ← 독립적 (순수 C 라이브러리)

### 스레드 모델

- **Main Task**: 100Hz 제어 루프 (FreeRTOS 우선순위 5)
- **BLE Task**: NimBLE 스택 이벤트 처리 (우선순위 4)
- **Config Task**: NVS 저장 작업 (우선순위 3)

---

## BLE 컨트롤러 API

### 헤더 파일
```c
#include "output/ble_controller.h"
```

### 핵심 구조체

#### ble_controller_t
```c
typedef struct {
    bool device_connected;           ///< BLE 연결 상태
    remote_command_t current_command; ///< 현재 수신 명령
    char last_command[64];           ///< 마지막 텍스트 명령
    bool has_text_command;           ///< 처리 대기 텍스트 명령 존재
    ble_conn_handle_t conn_handle;   ///< BSW BLE 연결 핸들
    ble_service_handle_t service_handle; ///< GATT 서비스 핸들
    ble_char_handle_t command_char_handle; ///< 명령 특성 핸들
    ble_char_handle_t status_char_handle;  ///< 상태 특성 핸들
} ble_controller_t;
```

#### remote_command_t
```c
typedef struct {
    int direction;    ///< 방향 (-1:후진, 0:정지, 1:전진)
    int turn;         ///< 회전 (-100~100)
    int speed;        ///< 속도 (0~100)
    bool balance;     ///< 밸런싱 활성화
    bool standup;     ///< 기립 명령
} remote_command_t;
```

### 초기화 함수

#### ble_controller_init()
```c
esp_err_t ble_controller_init(ble_controller_t* ble, const char* device_name);
```

**매개변수:**
- `ble`: BLE 컨트롤러 구조체 포인터
- `device_name`: BLE 광고명 (예: "BalanceBot-C6")

**반환값:**
- `ESP_OK`: 초기화 성공
- `ESP_FAIL`: 초기화 실패

**사용 예제:**
```c
static ble_controller_t ble_ctrl;

esp_err_t init_ble_system() {
    esp_err_t ret = ble_controller_init(&ble_ctrl, "BalanceBot-C6");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE 초기화 실패: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE 시스템 초기화 완료");
    return ESP_OK;
}
```

### 런타임 함수

#### ble_controller_update()
```c
void ble_controller_update(ble_controller_t* ble);
```

**설명:** BLE 이벤트 처리 및 상태 업데이트. **주기적 호출 필수** (권장: 10ms마다)

**사용 예제:**
```c
void app_main_loop() {
    while (1) {
        // BLE 이벤트 처리
        ble_controller_update(&ble_ctrl);
        
        // 제어 로직 실행
        control_loop();
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms 주기
    }
}
```

#### ble_controller_get_command()
```c
remote_command_t ble_controller_get_command(const ble_controller_t* ble);
```

**설명:** 현재 유효한 원격 제어 명령 반환

**사용 예제:**
```c
void process_remote_commands() {
    if (ble_controller_is_connected(&ble_ctrl)) {
        remote_command_t cmd = ble_controller_get_command(&ble_ctrl);
        
        // 방향 제어
        if (cmd.direction == 1) {
            set_motor_forward(cmd.speed);
        } else if (cmd.direction == -1) {
            set_motor_backward(cmd.speed);
        } else {
            stop_motors();
        }
        
        // 회전 제어
        if (cmd.turn != 0) {
            set_motor_turn(cmd.turn);
        }
        
        // 밸런싱 제어
        enable_balance_mode(cmd.balance);
        
        // 기립 명령
        if (cmd.standup) {
            trigger_standup_sequence();
        }
    }
}
```

#### ble_controller_send_status()
```c
esp_err_t ble_controller_send_status(ble_controller_t* ble, float angle, 
                                    float velocity, float battery_voltage);
```

**매개변수:**
- `angle`: 로봇 기울기 각도 (도 단위, -90 ~ +90)
- `velocity`: 로봇 속도 (m/s 단위)
- `battery_voltage`: 배터리 전압 (V 단위, 3.0 ~ 4.2)

**사용 예제:**
```c
void send_robot_status() {
    float current_angle = get_robot_angle();
    float current_velocity = get_robot_velocity(); 
    float battery_voltage = get_battery_voltage();
    
    esp_err_t ret = ble_controller_send_status(&ble_ctrl, current_angle, 
                                              current_velocity, battery_voltage);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "상태 전송 실패");
    }
}
```

### 텍스트 명령 처리

#### ble_controller_has_text_command()
```c
bool ble_controller_has_text_command(const ble_controller_t* ble);
```

#### ble_controller_get_text_command()  
```c
const char* ble_controller_get_text_command(ble_controller_t* ble);
```

**사용 예제:**
```c
void handle_text_commands() {
    if (ble_controller_has_text_command(&ble_ctrl)) {
        const char* text_cmd = ble_controller_get_text_command(&ble_ctrl);
        if (text_cmd != NULL) {
            // Config Manager로 전달하여 파라미터 튜닝 처리
            esp_err_t ret = config_manager_handle_ble_command(text_cmd);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "텍스트 명령 처리 완료: %s", text_cmd);
            } else {
                ESP_LOGE(TAG, "텍스트 명령 처리 실패: %s", text_cmd);
            }
        }
    }
}
```

---

## 프로토콜 시스템

### 헤더 파일
```c
#include "system/protocol.h"
```

### 메시지 구조

#### 프로토콜 헤더 (8바이트)
```c
typedef struct __attribute__((packed)) {
    uint8_t start_marker;       ///< 시작 마커 (0xAA)
    uint8_t version;           ///< 프로토콜 버전 (0x01)
    uint8_t msg_type;          ///< 메시지 타입
    uint8_t payload_len;       ///< 페이로드 길이
    uint16_t sequence_num;     ///< 시퀀스 번호
    uint8_t flags;             ///< 제어 플래그
    uint8_t checksum;          ///< 체크섬
} protocol_header_t;
```

#### 이동 명령 페이로드
```c
typedef struct __attribute__((packed)) {
    int8_t direction;          ///< 이동 방향
    int8_t turn;              ///< 회전값
    uint8_t speed;            ///< 속도
    uint8_t flags;            ///< 명령 플래그
    uint32_t timestamp;       ///< 타임스탬프
} move_command_payload_t;
```

### 메시지 타입

```c
#define MSG_TYPE_MOVE_CMD       0x01  ///< 이동 명령
#define MSG_TYPE_STATUS_REQ     0x02  ///< 상태 요청  
#define MSG_TYPE_STATUS_RESP    0x03  ///< 상태 응답
#define MSG_TYPE_CONFIG_SET     0x04  ///< 설정 변경
#define MSG_TYPE_CONFIG_GET     0x05  ///< 설정 조회
#define MSG_TYPE_ERROR          0xFF  ///< 오류 메시지
```

### 핵심 함수

#### encode_message()
```c
int encode_message(const protocol_message_t* msg, uint8_t* buffer, int buffer_size);
```

#### decode_message()
```c
int decode_message(const uint8_t* buffer, int buffer_len, protocol_message_t* msg);
```

#### validate_message()
```c
bool validate_message(const protocol_message_t* msg);
```

**사용 예제:**
```c
void send_move_command(int8_t direction, int8_t turn, uint8_t speed) {
    protocol_message_t msg;
    uint8_t buffer[64];
    
    // 이동 명령 메시지 구성
    build_move_command(&msg, direction, turn, speed, 
                       CMD_FLAG_BALANCE, get_timestamp());
    
    // 메시지 인코딩
    int encoded_len = encode_message(&msg, buffer, sizeof(buffer));
    if (encoded_len > 0) {
        // BLE를 통해 전송
        ble_send_data(conn_handle, char_handle, buffer, encoded_len);
    }
}

void process_received_data(const uint8_t* data, size_t len) {
    protocol_message_t msg;
    
    // 메시지 디코딩
    int result = decode_message(data, len, &msg);
    if (result > 0 && validate_message(&msg)) {
        // 메시지 타입별 처리
        switch (msg.header.msg_type) {
            case MSG_TYPE_MOVE_CMD:
                handle_move_command(&msg.payload.move_cmd);
                break;
            case MSG_TYPE_STATUS_REQ:
                send_status_response();
                break;
            default:
                ESP_LOGW(TAG, "알 수 없는 메시지 타입: 0x%02X", msg.header.msg_type);
        }
    }
}
```

---

## 설정 관리자

### 헤더 파일
```c
#include "system/config_manager.h"
```

### 파라미터 ID 열거형

```c
typedef enum {
    CONFIG_PARAM_BALANCE_KP = 0,        ///< 밸런스 PID Kp
    CONFIG_PARAM_BALANCE_KI,            ///< 밸런스 PID Ki  
    CONFIG_PARAM_BALANCE_KD,            ///< 밸런스 PID Kd
    CONFIG_PARAM_VELOCITY_KP,           ///< 속도 PID Kp
    CONFIG_PARAM_VELOCITY_KI,           ///< 속도 PID Ki
    CONFIG_PARAM_VELOCITY_KD,           ///< 속도 PID Kd
    CONFIG_PARAM_KALMAN_Q_ANGLE,        ///< 칼만 각도 프로세스 노이즈
    CONFIG_PARAM_KALMAN_Q_BIAS,         ///< 칼만 바이어스 프로세스 노이즈  
    CONFIG_PARAM_KALMAN_R_MEASURE,      ///< 칼만 측정 노이즈
    CONFIG_PARAM_MAX_TILT_ANGLE,        ///< 최대 기울기 각도
    CONFIG_PARAM_FALLEN_THRESHOLD,      ///< 넘어짐 판정 임계값
    CONFIG_PARAM_COUNT                  ///< 총 파라미터 개수
} config_param_id_t;
```

### 핵심 함수

#### config_manager_init()
```c
esp_err_t config_manager_init(void);
```

**설명:** NVS 초기화 및 기본값 로드. **앱 시작 시 1회 호출 필수**

#### config_manager_set_param()
```c
esp_err_t config_manager_set_param(config_param_id_t param_id, float value, bool save_to_nvs);
```

**매개변수:**
- `param_id`: 파라미터 ID (enum 값)
- `value`: 설정할 값
- `save_to_nvs`: NVS에 즉시 저장할지 여부

#### config_manager_get_param()
```c
float config_manager_get_param(config_param_id_t param_id);
```

#### config_manager_handle_ble_command()
```c
esp_err_t config_manager_handle_ble_command(const char* command);
```

**지원 명령:**
- `"SET <param_id> <value>"`: 파라미터 설정
- `"GET <param_id>"`: 파라미터 조회  
- `"SAVE"`: 모든 설정 NVS 저장
- `"RESET"`: 기본값 복원

**사용 예제:**
```c
// 초기화
esp_err_t init_config_system() {
    esp_err_t ret = config_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config Manager 초기화 실패");
        return ret;
    }
    
    // 런타임 파라미터 구조체 가져오기
    tuning_params_t* params = config_manager_get_tuning_params();
    
    // PID 컨트롤러에 파라미터 적용
    pid_init(&balance_pid, params->balance_kp, params->balance_ki, params->balance_kd);
    pid_init(&velocity_pid, params->velocity_kp, params->velocity_ki, params->velocity_kd);
    
    return ESP_OK;
}

// 실시간 파라미터 업데이트
void update_balance_kp(float new_kp) {
    esp_err_t ret = config_manager_set_param(CONFIG_PARAM_BALANCE_KP, new_kp, false);
    if (ret == ESP_OK) {
        // PID 컨트롤러에 즉시 적용
        tuning_params_t* params = config_manager_get_tuning_params();
        pid_set_gains(&balance_pid, params->balance_kp, params->balance_ki, params->balance_kd);
        
        ESP_LOGI(TAG, "Balance Kp 업데이트: %.3f", new_kp);
    }
}

// BLE 텍스트 명령 처리
void process_ble_text_commands() {
    if (ble_controller_has_text_command(&ble_ctrl)) {
        const char* cmd = ble_controller_get_text_command(&ble_ctrl);
        esp_err_t ret = config_manager_handle_ble_command(cmd);
        
        if (ret == ESP_OK) {
            // 업데이트된 파라미터를 제어 시스템에 적용
            apply_updated_parameters();
        }
    }
}
```

---

## BSW 계층

### BLE 드라이버

#### 헤더 파일
```c
#include "bsw/ble_driver.h"
```

#### 핵심 타입
```c
typedef uint16_t ble_conn_handle_t;
typedef uint16_t ble_char_handle_t; 
typedef uint16_t ble_service_handle_t;

typedef void (*ble_event_callback_t)(const ble_event_t* event, void* user_data);
```

#### 주요 함수

```c
// 드라이버 초기화
bool ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data);

// 서비스 생성
bool ble_create_service(const bsw_ble_uuid_t* service_uuid, ble_service_handle_t* service_handle);

// 특성 추가
bool ble_add_characteristic(ble_service_handle_t service_handle,
                           const bsw_ble_uuid_t* char_uuid,
                           const ble_char_properties_t* properties,
                           ble_char_handle_t* char_handle);

// 데이터 전송
bool ble_send_data(ble_conn_handle_t conn_handle, ble_char_handle_t char_handle,
                  const uint8_t* data, size_t len);

// 광고 시작
bool ble_start_advertising(void);
```

### 시스템 서비스

#### 헤더 파일
```c
#include "bsw/system_services.h"  
```

#### 로깅 매크로
```c
#define BSW_LOGE(tag, format, ...)  ///< 에러 로그
#define BSW_LOGW(tag, format, ...)  ///< 경고 로그  
#define BSW_LOGI(tag, format, ...)  ///< 정보 로그
#define BSW_LOGD(tag, format, ...)  ///< 디버그 로그
#define BSW_LOGV(tag, format, ...)  ///< 상세 로그
```

---

## 통합 및 사용법

### 메인 애플리케이션 구조

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "output/ble_controller.h"
#include "system/config_manager.h"
#include "logic/pid_controller.h"
#include "input/imu_sensor.h"

static const char* TAG = "MAIN";

// 전역 객체
static ble_controller_t ble_ctrl;
static pid_controller_t balance_pid;
static pid_controller_t velocity_pid;

void app_main(void) {
    ESP_LOGI(TAG, "BalanceBot 시작");
    
    // 1. 시스템 초기화
    ESP_ERROR_CHECK(init_hardware());
    ESP_ERROR_CHECK(config_manager_init());
    ESP_ERROR_CHECK(ble_controller_init(&ble_ctrl, "BalanceBot-C6"));
    
    // 2. 제어 시스템 초기화
    init_control_systems();
    
    // 3. 메인 루프 시작
    xTaskCreate(main_control_task, "control", 4096, NULL, 5, NULL);
    xTaskCreate(ble_communication_task, "ble_comm", 4096, NULL, 4, NULL);
}

void main_control_task(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100Hz 제어
    
    while (1) {
        // 1. 센서 데이터 읽기
        float angle = read_robot_angle();
        float velocity = calculate_velocity();
        
        // 2. BLE 명령 처리
        handle_remote_commands();
        handle_text_commands();
        
        // 3. 제어 알고리즘 실행
        float balance_output = pid_compute(&balance_pid, 0.0, angle);
        float velocity_output = pid_compute(&velocity_pid, target_velocity, velocity);
        
        // 4. 모터 출력
        set_motor_outputs(balance_output, velocity_output);
        
        // 5. 다음 주기까지 대기
        vTaskDelayUntil(&last_wake_time, period);
    }
}

void ble_communication_task(void* parameters) {
    TickType_t last_status_time = 0;
    const TickType_t status_period = pdMS_TO_TICKS(50); // 20Hz 상태 전송
    
    while (1) {
        // BLE 이벤트 처리
        ble_controller_update(&ble_ctrl);
        
        // 주기적 상태 전송
        TickType_t current_time = xTaskGetTickCount();
        if (current_time - last_status_time >= status_period) {
            if (ble_controller_is_connected(&ble_ctrl)) {
                float angle = get_current_angle();
                float velocity = get_current_velocity();
                float battery = get_battery_voltage();
                
                ble_controller_send_status(&ble_ctrl, angle, velocity, battery);
            }
            last_status_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 5ms 주기
    }
}

void handle_remote_commands(void) {
    if (ble_controller_is_connected(&ble_ctrl)) {
        remote_command_t cmd = ble_controller_get_command(&ble_ctrl);
        
        // 이동 명령 처리
        target_velocity = cmd.direction * cmd.speed * MAX_VELOCITY / 100.0f;
        target_turn_rate = cmd.turn * MAX_TURN_RATE / 100.0f;
        
        // 밸런싱 모드
        balance_mode_enabled = cmd.balance;
        
        // 기립 명령
        if (cmd.standup && !standup_in_progress) {
            trigger_standup_sequence();
        }
    }
}

void handle_text_commands(void) {
    if (ble_controller_has_text_command(&ble_ctrl)) {
        const char* text_cmd = ble_controller_get_text_command(&ble_ctrl);
        if (text_cmd != NULL) {
            esp_err_t ret = config_manager_handle_ble_command(text_cmd);
            if (ret == ESP_OK) {
                // 업데이트된 파라미터 적용
                apply_updated_parameters();
                ESP_LOGI(TAG, "파라미터 업데이트 완료: %s", text_cmd);
            }
        }
    }
}

void apply_updated_parameters(void) {
    tuning_params_t* params = config_manager_get_tuning_params();
    
    // PID 게인 업데이트
    pid_set_gains(&balance_pid, params->balance_kp, params->balance_ki, params->balance_kd);
    pid_set_gains(&velocity_pid, params->velocity_kp, params->velocity_ki, params->velocity_kd);
    
    // 칼만 필터 노이즈 파라미터 업데이트
    kalman_set_noise_params(&kalman_filter, params->kalman_q_angle, 
                           params->kalman_q_bias, params->kalman_r_measure);
    
    // 안전 임계값 업데이트
    set_safety_thresholds(params->max_tilt_angle, params->fallen_threshold);
}
```

### CMakeLists.txt 설정

```cmake
# 메인 컴포넌트
set(COMPONENT_SRCS 
    "main.c"
    "output/ble_controller.c"
    "system/config_manager.c" 
    "system/protocol.c"
    "bsw/ble_driver.c"
    "bsw/system_services.c"
    "logic/pid_controller.c"
    "input/imu_sensor.c"
)

set(COMPONENT_ADD_INCLUDEDIRS 
    "."
    "output"
    "system" 
    "bsw"
    "logic"
    "input"
)

# NimBLE 컴포넌트 의존성
set(COMPONENT_REQUIRES 
    nvs_flash
    bt
    esp_nimble
)

register_component()
```

---

## 디버깅 가이드

### 로깅 레벨 설정

```c
// Component별 로깅 레벨 설정 (menuconfig에서도 가능)
esp_log_level_set("BLE_CONTROLLER", ESP_LOG_DEBUG);
esp_log_level_set("CONFIG_MANAGER", ESP_LOG_INFO);
esp_log_level_set("PROTOCOL", ESP_LOG_WARN);
```

### BLE 연결 문제

```c
void debug_ble_connection(void) {
    ESP_LOGI(TAG, "BLE 연결 상태: %s", 
             ble_controller_is_connected(&ble_ctrl) ? "연결됨" : "연결 안됨");
    
    // NimBLE 스택 상태 확인
    ESP_LOGI(TAG, "NimBLE 초기화 상태: %s", 
             nimble_port_get_state() == NIMBLE_PORT_STATE_STARTED ? "시작됨" : "중지됨");
    
    // 메모리 사용량 확인
    ESP_LOGI(TAG, "힙 메모리 여유: %d bytes", esp_get_free_heap_size());
}
```

### 프로토콜 패킷 분석

```c
void debug_protocol_packet(const uint8_t* data, size_t len) {
    ESP_LOGI(TAG, "수신 패킷 (%d bytes):", len);
    
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
    
    // 헤더 분석
    if (len >= sizeof(protocol_header_t)) {
        protocol_header_t* header = (protocol_header_t*)data;
        ESP_LOGI(TAG, "헤더: marker=0x%02X, ver=0x%02X, type=0x%02X, len=%d",
                 header->start_marker, header->version, header->msg_type, header->payload_len);
    }
}
```

### 성능 모니터링

```c
void monitor_system_performance(void) {
    static uint32_t last_print_time = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    if (current_time - last_print_time >= pdMS_TO_TICKS(5000)) { // 5초마다
        // CPU 사용률
        char task_list_buffer[1024];
        vTaskList(task_list_buffer);
        ESP_LOGI(TAG, "태스크 상태:\n%s", task_list_buffer);
        
        // 메모리 사용량
        ESP_LOGI(TAG, "힙 메모리: 여유=%d, 최소여유=%d", 
                 esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
        
        // BLE 통계
        ESP_LOGI(TAG, "BLE 연결: %s, 마지막 데이터: %s",
                 ble_controller_is_connected(&ble_ctrl) ? "OK" : "FAIL",
                 ble_ctrl.last_command);
        
        last_print_time = current_time;
    }
}
```

---

## 테스트 및 검증

### 단위 테스트 예제

```c
#include "unity.h"
#include "system/config_manager.h"

void setUp(void) {
    config_manager_init();
}

void tearDown(void) {
    // 정리 작업
}

void test_config_manager_set_get_param(void) {
    // Given
    float test_value = 25.5f;
    config_param_id_t param_id = CONFIG_PARAM_BALANCE_KP;
    
    // When
    esp_err_t set_result = config_manager_set_param(param_id, test_value, false);
    float get_result = config_manager_get_param(param_id);
    
    // Then
    TEST_ASSERT_EQUAL(ESP_OK, set_result);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, test_value, get_result);
}

void test_ble_text_command_parsing(void) {
    // Given
    const char* test_command = "SET 0 30.0";
    
    // When
    esp_err_t result = config_manager_handle_ble_command(test_command);
    float value = config_manager_get_param(CONFIG_PARAM_BALANCE_KP);
    
    // Then
    TEST_ASSERT_EQUAL(ESP_OK, result);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, value);
}

void test_protocol_message_encoding_decoding(void) {
    // Given
    protocol_message_t original_msg;
    build_move_command(&original_msg, 1, 50, 80, CMD_FLAG_BALANCE, 12345);
    
    uint8_t buffer[64];
    protocol_message_t decoded_msg;
    
    // When
    int encoded_len = encode_message(&original_msg, buffer, sizeof(buffer));
    int decoded_len = decode_message(buffer, encoded_len, &decoded_msg);
    
    // Then
    TEST_ASSERT_GREATER_THAN(0, encoded_len);
    TEST_ASSERT_GREATER_THAN(0, decoded_len);
    TEST_ASSERT_EQUAL(original_msg.header.msg_type, decoded_msg.header.msg_type);
    TEST_ASSERT_EQUAL(original_msg.payload.move_cmd.direction, decoded_msg.payload.move_cmd.direction);
    TEST_ASSERT_EQUAL(original_msg.payload.move_cmd.speed, decoded_msg.payload.move_cmd.speed);
}
```

### 통합 테스트 시나리오

```c
void integration_test_ble_parameter_tuning(void) {
    ESP_LOGI("TEST", "=== BLE 파라미터 튜닝 통합 테스트 시작 ===");
    
    // 1. BLE 연결 시뮬레이션
    simulate_ble_connection();
    TEST_ASSERT_TRUE(ble_controller_is_connected(&ble_ctrl));
    
    // 2. 텍스트 명령 전송 시뮬레이션
    simulate_ble_text_command("SET 0 35.0");  // Balance Kp 설정
    
    // 3. 명령 처리 확인
    handle_text_commands();
    float updated_kp = config_manager_get_param(CONFIG_PARAM_BALANCE_KP);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 35.0f, updated_kp);
    
    // 4. PID 컨트롤러에 적용 확인
    apply_updated_parameters();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 35.0f, balance_pid.kp);
    
    ESP_LOGI("TEST", "=== BLE 파라미터 튜닝 통합 테스트 완료 ===");
}
```

---

## 추가 리소스

### 관련 문서
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [NimBLE User Guide](https://mynewt.apache.org/latest/network/ble/ble_intro/)
- [FreeRTOS Reference Manual](https://www.freertos.org/Documentation/RTOS_book.html)

### 개발 도구
- **ESP-IDF Monitor**: `idf.py monitor` - 실시간 로그 확인
- **GDB**: `idf.py gdb` - 디버깅
- **Unity**: ESP-IDF 내장 단위 테스트 프레임워크

### 성능 벤치마크
- **BLE 처리량**: 최대 20KB/s (MTU 512, 15ms 간격)  
- **제어 주기**: 100Hz (10ms) 안정적 달성
- **메모리 사용**: 약 180KB RAM, 1.2MB Flash

---

**문의사항이나 기술 지원이 필요하시면 GitHub Issue로 등록해주세요.**