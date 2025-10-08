/**
 * @file ble_controller_mock.c
 * @brief POSIX 시뮬레이터용 BLE 컨트롤러 목 구현
 * 
 * BLE 통신을 표준 입력/출력으로 시뮬레이션합니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// Mock ESP types
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1

// Mock 로깅
#define ESP_LOGI(tag, format, ...) printf("[INFO][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("[WARN][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) printf("[ERROR][%s] " format "\n", tag, ##__VA_ARGS__)

static const char* TAG = "BLE_MOCK";

// 원격 제어 명령 구조체
typedef struct {
    int direction;
    int turn;
    int speed;
    bool balance;
    bool standup;
} remote_command_t;

// BLE 컨트롤러 상태 구조체
typedef struct {
    bool device_connected;
    remote_command_t current_command;
    char last_command[64];
    bool has_text_command;
} ble_controller_t;

// 전역 상태
static bool initialized = false;

// 초기화
esp_err_t ble_controller_init(ble_controller_t* ble, const char* device_name) {
    if (!ble) return ESP_FAIL;
    
    memset(ble, 0, sizeof(ble_controller_t));
    ble->device_connected = true; // 시뮬레이션에서는 항상 연결됨
    
    // 기본 명령 설정
    ble->current_command.direction = 0;
    ble->current_command.turn = 0;
    ble->current_command.speed = 0;
    ble->current_command.balance = true;
    ble->current_command.standup = false;
    
    // 표준 입력을 논블로킹 모드로 설정
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    
    initialized = true;
    ESP_LOGI(TAG, "BLE Controller Mock initialized (%s)", device_name);
    ESP_LOGI(TAG, "Commands: 'w'=forward, 's'=backward, 'a'=left, 'd'=right, ' '=stop");
    ESP_LOGI(TAG, "Text commands: 'SET 0 25.5', 'GET 0', 'SAVE', 'RESET'");
    
    return ESP_OK;
}

// BLE 업데이트 (키보드 입력 확인)
void ble_controller_update(ble_controller_t* ble) {
    if (!ble || !initialized) return;
    
    char input[256];
    ssize_t bytes_read = read(STDIN_FILENO, input, sizeof(input) - 1);
    
    if (bytes_read > 0) {
        input[bytes_read] = '\0';
        
        // 개행 문자 제거
        for (int i = 0; i < bytes_read; i++) {
            if (input[i] == '\n' || input[i] == '\r') {
                input[i] = '\0';
                break;
            }
        }
        
        // 텍스트 명령 처리 (SET, GET, SAVE, RESET)
        if (strncmp(input, "SET", 3) == 0 || strncmp(input, "GET", 3) == 0 || 
            strncmp(input, "SAVE", 4) == 0 || strncmp(input, "RESET", 5) == 0) {
            strncpy(ble->last_command, input, sizeof(ble->last_command) - 1);
            ble->has_text_command = true;
            ESP_LOGI(TAG, "Text command received: %s", input);
            return;
        }
        
        // 단일 키 명령 처리
        char key = input[0];
        switch (key) {
            case 'w': // 전진
                ble->current_command.direction = 1;
                ble->current_command.speed = 50;
                ble->current_command.turn = 0;
                ESP_LOGI(TAG, "Command: Forward");
                break;
            case 's': // 후진
                ble->current_command.direction = -1;
                ble->current_command.speed = 50;
                ble->current_command.turn = 0;
                ESP_LOGI(TAG, "Command: Backward");
                break;
            case 'a': // 좌회전
                ble->current_command.direction = 0;
                ble->current_command.speed = 30;
                ble->current_command.turn = -50;
                ESP_LOGI(TAG, "Command: Turn Left");
                break;
            case 'd': // 우회전
                ble->current_command.direction = 0;
                ble->current_command.speed = 30;
                ble->current_command.turn = 50;
                ESP_LOGI(TAG, "Command: Turn Right");
                break;
            case ' ': // 정지
                ble->current_command.direction = 0;
                ble->current_command.speed = 0;
                ble->current_command.turn = 0;
                ESP_LOGI(TAG, "Command: Stop");
                break;
            case 'b': // 밸런싱 토글
                ble->current_command.balance = !ble->current_command.balance;
                ESP_LOGI(TAG, "Balance: %s", ble->current_command.balance ? "ON" : "OFF");
                break;
            case 'u': // 기립
                ble->current_command.standup = true;
                ESP_LOGI(TAG, "Command: Stand Up");
                break;
            case 'q': // 종료
                ESP_LOGI(TAG, "Quit command received");
                exit(0);
                break;
            default:
                // 알 수 없는 명령 무시
                break;
        }
    }
}

// 현재 명령 가져오기
remote_command_t ble_controller_get_command(const ble_controller_t* ble) {
    if (ble) {
        return ble->current_command;
    }
    
    remote_command_t empty_cmd = {0};
    return empty_cmd;
}

// 연결 상태 확인
bool ble_controller_is_connected(const ble_controller_t* ble) {
    return ble ? ble->device_connected : false;
}

// 상태 전송 (콘솔에 출력)
esp_err_t ble_controller_send_status(ble_controller_t* ble, float angle, float velocity, float battery_voltage) {
    if (!ble || !initialized) return ESP_FAIL;
    
    // 상태 정보를 콘솔에 출력
    printf("[BLE_STATUS] Angle: %6.2f° | Velocity: %6.2f cm/s | Battery: %.2fV\n", 
           angle, velocity, battery_voltage);
    
    return ESP_OK;
}

// 텍스트 명령 존재 여부 확인
bool ble_controller_has_text_command(const ble_controller_t* ble) {
    return ble ? ble->has_text_command : false;
}

// 텍스트 명령 가져오기
const char* ble_controller_get_text_command(ble_controller_t* ble) {
    if (!ble || !ble->has_text_command) {
        return NULL;
    }
    
    ble->has_text_command = false; // 플래그 클리어
    return ble->last_command;
}