/**
 * @file ble_controller.c
 * @brief BLE (Bluetooth Low Energy) 컨트롤러 구현
 * 
 * BSW BLE 드라이버를 사용하여 모바일 앱과의 무    if (!ble_start_advertising()) {
        BSW_LOGE(TAG, "Failed to start advertising");
        return ESP_FAIL;
    } 구현합니다.
 * GATT 서버로 동작하며, 명령 수신 및 상태 전송 기능을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 1.0
 */

#include "ble_controller.h"
#include "../system/protocol.h"
#include "../bsw/system_services.h"  // BSW 시스템 서비스 추상화
#include "../bsw/ble_driver.h"       // BSW BLE 드라이버 추상화
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "BLE_CONTROLLER";    ///< 로깅 태그

// BLE Service and Characteristic UUIDs
/// 서비스 UUID (128비트): 0000FF00-0000-1000-8000-00805F9B34FB
static uint8_t service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00
};

/// 명령 특성 UUID (128비트): 0000FF01-0000-1000-8000-00805F9B34FB
static uint8_t command_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00
};

/// 상태 특성 UUID (128비트): 0000FF02-0000-1000-8000-00805F9B34FB
static uint8_t status_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x02, 0xff, 0x00, 0x00
};

// 전역 BLE 컨트롤러 인스턴스 참조
static ble_controller_t* g_ble_controller = NULL;

// BSW BLE 이벤트 콜백 함수
static void ble_event_handler(const ble_event_t* event, void* user_data);

/**
 * @brief BLE 컨트롤러 초기화 구현
 */
esp_err_t ble_controller_init(ble_controller_t* ble, const char* device_name) {
    esp_err_t ret;
    
    if (!ble || !device_name) {
        BSW_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }
    
    g_ble_controller = ble;
    
    // 구조체 초기화
    ble->device_connected = false;
    ble->current_command.direction = 0;
    ble->current_command.turn = 0;
    ble->current_command.speed = 0;
    ble->current_command.balance = true;
    ble->current_command.standup = false;
    ble->conn_handle = 0;
    ble->service_handle = 0;
    ble->command_char_handle = 0;
    ble->status_char_handle = 0;
    memset(ble->last_command, 0, sizeof(ble->last_command));

    // BSW BLE 드라이버 초기화
    if (!ble_driver_init(device_name, ble_event_handler, ble)) {
        BSW_LOGE(TAG, "Failed to initialize BLE driver");
        return ESP_FAIL;
    }

    // BLE 서비스 생성
    bsw_ble_uuid_t service_uuid_struct = ble_uuid_from_128(service_uuid);
    if (!ble_create_service(&service_uuid_struct, &ble->service_handle)) {
        BSW_LOGE(TAG, "Failed to create BLE service");
        return ESP_FAIL;
    }

    // 명령 특성 추가
    bsw_ble_uuid_t cmd_uuid = ble_uuid_from_128(command_char_uuid);
    ble_char_properties_t cmd_props = { .read = true, .write = true, .notify = false, .indicate = false };
    ret = ble_add_characteristic(ble->service_handle, &cmd_uuid,
                                &cmd_props,
                                &ble->command_char_handle) ? ESP_OK : ESP_FAIL;
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to add command characteristic: %s", (ret == ESP_OK) ? "ESP_OK" : "ESP_FAIL");
        return ret;
    }

    // 상태 특성 추가
    bsw_ble_uuid_t status_uuid = ble_uuid_from_128(status_char_uuid);
    ble_char_properties_t status_props = { .read = true, .write = false, .notify = true, .indicate = false };
    ret = ble_add_characteristic(ble->service_handle, &status_uuid,
                                &status_props,
                                &ble->status_char_handle) ? ESP_OK : ESP_FAIL;
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to add status characteristic: %s", (ret == ESP_OK) ? "ESP_OK" : "ESP_FAIL");
        return ret;
    }

    // 서비스 시작
    ret = ble_start_service(ble->service_handle) ? ESP_OK : ESP_FAIL;
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to start BLE service: %s", (ret == ESP_OK) ? "ESP_OK" : "ESP_FAIL");
        return ret;
    }

    // 광고 시작
    ret = ble_start_advertising() ? ESP_OK : ESP_FAIL;
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to start BLE advertising: %s", (ret == ESP_OK) ? "ESP_OK" : "ESP_FAIL");
        return ret;
    }

    BSW_LOGI(TAG, "BLE Controller initialized successfully");
    return ESP_OK;
}

/**
 * @brief BLE 컨트롤러 업데이트 구현
 */
void ble_controller_update(ble_controller_t* ble) {
    // BSW BLE 드라이버에서 이벤트 처리를 담당하므로 추가 작업 없음
}

/**
 * @brief 원격 제어 명령 가져오기 구현
 */
remote_command_t ble_controller_get_command(const ble_controller_t* ble) {
    return ble->current_command;
}

/**
 * @brief BLE 연결 상태 확인 구현
 */
bool ble_controller_is_connected(const ble_controller_t* ble) {
    return ble->device_connected;
}

/**
 * @brief 로봇 상태 전송 구현
 */
esp_err_t ble_controller_send_status(ble_controller_t* ble, float angle, float velocity, float battery_voltage) {
    if (!ble->device_connected) {
        return ESP_FAIL;
    }
    
    // Create status response message
    protocol_message_t msg;
    static uint8_t seq_num = 0;
    
    build_status_response(&msg, angle, velocity, 0x02, seq_num++); // State = BALANCING
    
    // Update battery level (convert voltage to percentage)
    uint8_t battery_percentage = (uint8_t)((battery_voltage - 3.0f) / 1.2f * 100.0f);
    if (battery_percentage > 100) battery_percentage = 100;
    msg.payload.status_resp.battery_level = battery_percentage;
    
    // Encode message to buffer
    uint8_t buffer[64];
    int encoded_len = encode_message(&msg, buffer, sizeof(buffer));
    
    if (encoded_len <= 0) {
        BSW_LOGE(TAG, "Failed to encode status message");
        return ESP_FAIL;
    }
    
    // Send via BSW BLE driver
    if (!ble_send_data(ble->conn_handle, ble->status_char_handle,
                       buffer, encoded_len)) {
        BSW_LOGE(TAG, "Failed to send BLE notification");
        return ESP_FAIL;
    }
    
    BSW_LOGD(TAG, "Status sent: angle=%.2f, vel=%.2f, battery=%d%%", 
             angle, velocity, battery_percentage);
    
    return ESP_OK;
}

/**
 * @brief BLE 패킷 처리 구현
 */
esp_err_t ble_controller_process_packet(ble_controller_t* ble, const uint8_t* data, size_t length) {
    protocol_message_t msg;
    
    // 메시지 디코딩
    int result = decode_message(data, (int)length, &msg);
    if (result <= 0) {
        BSW_LOGE(TAG, "Failed to decode message: %d", result);
        return ESP_FAIL;
    }
    
    // 메시지 유효성 검증
    if (!validate_message(&msg)) {
        BSW_LOGE(TAG, "Message validation failed");
        return ESP_FAIL;
    }
    
    // 메시지 타입별 처리
    switch (msg.header.msg_type) {
        case MSG_TYPE_MOVE_CMD: {
            move_command_payload_t* cmd = &msg.payload.move_cmd;
            
            // 안전 범위 제한 적용
            ble->current_command.direction = (cmd->direction > 1) ? 1 : 
                                           ((cmd->direction < -1) ? -1 : cmd->direction);
            ble->current_command.turn = (cmd->turn > 100) ? 100 : 
                                      ((cmd->turn < -100) ? -100 : cmd->turn);
            ble->current_command.speed = (cmd->speed > 100) ? 100 : cmd->speed;
            
            // 제어 플래그 추출
            ble->current_command.balance = (cmd->flags & CMD_FLAG_BALANCE) != 0;
            ble->current_command.standup = (cmd->flags & CMD_FLAG_STANDUP) != 0;
            
            BSW_LOGD(TAG, "Move command: dir=%d, turn=%d, speed=%d, balance=%s, standup=%s", 
                     ble->current_command.direction, 
                     ble->current_command.turn, 
                     ble->current_command.speed,
                     ble->current_command.balance ? "true" : "false",
                     ble->current_command.standup ? "true" : "false");
            break;
        }
        
        case MSG_TYPE_CONFIG_SET: {
            BSW_LOGI(TAG, "Config set command received");
            // PID 설정 등 필요 시 처리
            break;
        }
        
        default:
            BSW_LOGW(TAG, "Unknown message type: 0x%02X", msg.header.msg_type);
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief 레거시 명령 파싱 (호환성용)
 */
void ble_controller_parse_command(ble_controller_t* ble, const char* command) {
    // 이전 버전 호환성을 위해 유지되지만 구현되지 않음
    BSW_LOGW(TAG, "Legacy command parsing not implemented: %s", command);
}

/**
 * @brief BSW BLE 이벤트 콜백 함수
 */
static void ble_event_handler(const ble_event_t* event, void* user_data) {
    ble_controller_t* ble = (ble_controller_t*)user_data;
    if (!ble) {
        return;
    }

    switch (event->type) {
        case BLE_EVENT_CONNECTED:
            BSW_LOGI(TAG, "BLE Client connected");
            ble->device_connected = true;
            ble->conn_handle = event->conn_handle;
            break;

        case BLE_EVENT_DISCONNECTED:
            BSW_LOGI(TAG, "BLE Client disconnected");
            ble->device_connected = false;
            ble->conn_handle = 0;
            break;

        case BLE_EVENT_DATA_RECEIVED:
            BSW_LOGI(TAG, "BLE Data received, length: %d", event->data_received.len);
            // 명령 특성에 데이터가 수신된 경우만 처리
            if (event->data_received.char_handle == ble->command_char_handle) {
                esp_err_t ret = ble_controller_process_packet(ble,
                                                            event->data_received.data,
                                                            event->data_received.len);
                if (ret != ESP_OK) {
                    BSW_LOGW(TAG, "Failed to process command packet");
                }
            }
            break;

        default:
            BSW_LOGD(TAG, "Unknown BLE event: %d", event->type);
            break;
    }
}

