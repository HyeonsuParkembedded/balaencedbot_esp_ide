/**
 * @file ble_controller.c
 * @brief BLE (Bluetooth Low Energy) 컨트롤러 구현
 * 
 * ESP32 BLE 스택을 사용하여 모바일 앱과의 무선 통신을 구현합니다.
 * GATT 서버로 동작하며, 명령 수신 및 상태 전송 기능을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 1.0
 */

#include "ble_controller.h"
#include "../system/protocol.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF 5.5.1 compatibility: Functions are resolved at link time

#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "BLE_CONTROLLER";    ///< 로깅 태그

static ble_controller_t* ble_instance = NULL;  ///< 글로벌 BLE 인스턴스 참조

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

// GATT Profile definitions
#define PROFILE_APP_ID              0         ///< 프로파일 앱 ID
#define DEVICE_NAME                 "BalanceBot"  ///< 기본 기기 이름
#define SVC_INST_ID                 0         ///< 서비스 인스턴스 ID

// Service and characteristic handles
static uint16_t service_handle = 0;         ///< 서비스 핸들
static uint16_t command_char_handle = 0;    ///< 명령 특성 핸들
static uint16_t status_char_handle = 0;     ///< 상태 특성 핸들
static esp_gatt_if_t gatts_if = ESP_GATT_IF_NONE;  ///< GATT 서버 인터페이스

// Advertisement configuration
/// 광고 데이터 설정
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Advertisement configuration flags  
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)
static uint8_t adv_config_done = 0;

#ifndef NATIVE_BUILD
// Forward declarations - 내부 콜백 및 헬퍼 함수들
/// GATT 서버 이벤트 핸들러 선언
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
/// GAP 이벤트 핸들러 선언
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
/// GATT 서비스 생성 함수 선언
static void create_service(void);
/// BLE 광고 시작 함수 선언
static void start_advertising(void);
#endif

/**
 * @brief BLE 컨트롤러 초기화 구현
 * 
 * BLE 스택을 초기화하고 GATT 서버를 설정합니다.
 * 명령 및 상태 특성을 생성하고 광고를 시작합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param device_name BLE 광고에 사용할 디바이스 이름
 * @return esp_err_t 초기화 결과
 */
esp_err_t ble_controller_init(ble_controller_t* ble, const char* device_name) {
    ble_instance = ble;
    
    // 구조체 초기화
    ble->device_connected = false;
    ble->current_command.direction = 0;
    ble->current_command.turn = 0;
    ble->current_command.speed = 0;
    ble->current_command.balance = true;
    ble->current_command.standup = false;
    ble->gatts_if = ESP_GATT_IF_NONE;
    ble->conn_id = 0;
    ble->command_handle = 0;
    ble->status_handle = 0;
    memset(ble->last_command, 0, sizeof(ble->last_command));

#ifndef NATIVE_BUILD
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release classic Bluetooth memory
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid stack
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GATT server callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GAP callbacks
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set device name
    ret = esp_ble_gap_set_device_name(device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GATT application
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set MTU size
    ret = esp_ble_gatt_set_local_mtu(512);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set MTU failed: %s", esp_err_to_name(ret));
        return ret;
    }
#endif

    ESP_LOGI(TAG, "BLE Controller initialized successfully");
    return ESP_OK;
}

/**
 * @brief BLE 컨트롤러 업데이트 구현
 * 
 * BLE 이벤트는 콜백 함수에서 처리되므로 이 함수는 비어있습니다.
 * 향후 주기적인 BLE 관련 작업이 필요할 경우 여기에 구현할 수 있습니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 */
void ble_controller_update(ble_controller_t* ble) {
    // BLE 업데이트는 이벤트 콜백에서 처리됨
}

/**
 * @brief 원격 제어 명령 가져오기 구현
 * 
 * 현재 저장된 원격 제어 명령을 반환합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @return remote_command_t 현재 명령
 */
remote_command_t ble_controller_get_command(const ble_controller_t* ble) {
    return ble->current_command;
}

/**
 * @brief BLE 연결 상태 확인 구현
 * 
 * BLE 기기가 현재 연결되어 있는지 확인합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @return bool 연결 상태
 */
bool ble_controller_is_connected(const ble_controller_t* ble) {
    return ble->device_connected;
}

/**
 * @brief 로봇 상태 전송 구현
 * 
 * 로봇의 현재 상태를 BLE 특성을 통해 클라이언트에게 전송합니다.
 * 상태 정보는 JSON 형태로 직렬화되어 전송됩니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param angle 로봇 기울기 각도 (도)
 * @param velocity 로봇 속도 (m/s)
 * @param battery_voltage 배터리 전압 (V)
 * @return esp_err_t 전송 결과
 */
esp_err_t ble_controller_send_status(ble_controller_t* ble, float angle, float velocity, float battery_voltage) {
    if (!ble->device_connected) {
        return ESP_FAIL;
    }
    
#ifndef NATIVE_BUILD
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
        ESP_LOGE(TAG, "Failed to encode status message");
        return ESP_FAIL;
    }
    
    // Send via BLE characteristic notification
    esp_err_t ret = esp_ble_gatts_send_indicate(ble->gatts_if, ble->conn_id, 
                                               ble->status_handle, encoded_len, 
                                               buffer, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send BLE notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Status sent: angle=%.2f, vel=%.2f, battery=%d%%", 
             angle, velocity, battery_percentage);
#endif
    
    return ESP_OK;
}

/**
 * @brief BLE 패킷 처리 구현
 * 
 * 수신된 BLE 데이터 패킷을 디코딩하고 검증한 후,
 * 메시지 타입에 따라 적절한 처리를 수행합니다.
 * 
 * 지원하는 메시지 타입:
 * - MSG_TYPE_MOVE_CMD: 로봇 이동 명령
 * - MSG_TYPE_CONFIG_SET: 설정 변경 명령
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param data 수신된 데이터 버퍼
 * @param length 데이터 길이
 * @return esp_err_t 처리 결과
 */
esp_err_t ble_controller_process_packet(ble_controller_t* ble, const uint8_t* data, size_t length) {
    protocol_message_t msg;
    
    // 메시지 디코딩
    int result = decode_message(data, (int)length, &msg);
    if (result <= 0) {
        ESP_LOGE(TAG, "Failed to decode message: %d", result);
        return ESP_FAIL;
    }
    
    // 메시지 유효성 검증
    if (!validate_message(&msg)) {
        ESP_LOGE(TAG, "Message validation failed");
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
            
            ESP_LOGD(TAG, "Move command: dir=%d, turn=%d, speed=%d, balance=%s, standup=%s", 
                     ble->current_command.direction, 
                     ble->current_command.turn, 
                     ble->current_command.speed,
                     ble->current_command.balance ? "true" : "false",
                     ble->current_command.standup ? "true" : "false");
            break;
        }
        
        case MSG_TYPE_CONFIG_SET: {
            ESP_LOGI(TAG, "Config set command received");
            // PID 설정 등 필요 시 처리
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown message type: 0x%02X", msg.header.msg_type);
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief 레거시 명령 파싱 (호환성용)
 * 
 * 이전 버전과의 호환성을 위해 유지되는 함수입니다.
 * 새로운 바이너리 프로토콜 사용을 권장합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param command 명령 문자열
 * @deprecated 새로운 바이너리 프로토콜 사용 권장
 */
void ble_controller_parse_command(ble_controller_t* ble, const char* command) {
    // 이전 버전 호환성을 위해 유지되지만 구현되지 않음
    ESP_LOGW(TAG, "Legacy command parsing not implemented: %s", command);
}

#ifndef NATIVE_BUILD

/**
 * @brief GATT 서비스 생성
 * 
 * BalanceBot BLE 서비스를 생성합니다.
 * 128비트 UUID를 사용하여 고유한 서비스를 정의합니다.
 */
static void create_service(void) {
    esp_gatt_srvc_id_t service_id;
    service_id.is_primary = true;
    service_id.id.inst_id = SVC_INST_ID;
    service_id.id.uuid.len = ESP_UUID_LEN_128;
    memcpy(service_id.id.uuid.uuid.uuid128, service_uuid, 16);
    
    esp_ble_gatts_create_service(gatts_if, &service_id, 4);
}

/**
 * @brief BLE 광고 시작
 * 
 * 설정된 광고 데이터로 BLE 광고를 시작합니다.
 * 클라이언트가 디바이스를 발견할 수 있도록 합니다.
 */
static void start_advertising(void) {
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure adv data: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief BLE GAP 이벤트 핸들러
 * 
 * BLE Generic Access Profile 이벤트를 처리합니다.
 * 광고 시작/중지, 연결/해제 등의 이벤트를 처리합니다.
 * 
 * @param event GAP 이벤트 타입
 * @param param 이벤트 매개변수
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
                } else {
                    ESP_LOGI(TAG, "Advertisement data set complete, starting advertising");
                }
            }
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed, status: %d", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed, status: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising stopped");
            }
            break;
            
        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

/**
 * @brief BLE GATT 서버 이벤트 핸들러
 * 
 * GATT 서버 관련 이벤트를 처리합니다.
 * 서비스/특성 생성, 클라이언트 연결, 데이터 읽기/쓰기 등을 처리합니다.
 * 
 * @param event GATT 서버 이벤트 타입
 * @param gatts_if_local GATT 서버 인터페이스
 * @param param 이벤트 매개변수
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_local, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered, app_id: %d", param->reg.app_id);
            gatts_if = gatts_if_local;
            if (ble_instance) {
                ble_instance->gatts_if = gatts_if_local;
            }
            create_service();
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created, service_handle: %d", param->create.service_handle);
            service_handle = param->create.service_handle;
            
            esp_ble_gatts_start_service(service_handle);
            
            // Add command characteristic
            esp_bt_uuid_t char_uuid;
            char_uuid.len = ESP_UUID_LEN_128;
            memcpy(char_uuid.uuid.uuid128, command_char_uuid, 16);
            
            esp_ble_gatts_add_char(service_handle, 
                                  &char_uuid,
                                  ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                  NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "Characteristic added, char_handle: %d", param->add_char.attr_handle);
            
            if (command_char_handle == 0) {
                command_char_handle = param->add_char.attr_handle;
                if (ble_instance) {
                    ble_instance->command_handle = command_char_handle;
                }
                
                // Add status characteristic
                esp_bt_uuid_t status_uuid;
                status_uuid.len = ESP_UUID_LEN_128;
                memcpy(status_uuid.uuid.uuid128, status_char_uuid, 16);
                
                esp_ble_gatts_add_char(service_handle, 
                                      &status_uuid,
                                      ESP_GATT_PERM_READ,
                                      ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                      NULL, NULL);
            } else if (status_char_handle == 0) {
                status_char_handle = param->add_char.attr_handle;
                if (ble_instance) {
                    ble_instance->status_handle = status_char_handle;
                }
                
                // Start advertising after all characteristics are added
                start_advertising();
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started");
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id: %d", param->connect.conn_id);
            if (ble_instance) {
                ble_instance->device_connected = true;
                ble_instance->conn_id = param->connect.conn_id;
            }
            
            // Update connection parameters
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, reason: %d", param->disconnect.reason);
            if (ble_instance) {
                ble_instance->device_connected = false;
                ble_instance->conn_id = 0;
            }
            
            // Restart advertising
            esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restart advertising: %s", esp_err_to_name(ret));
            }
            break;
            
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "Write event, handle: %d, len: %d", param->write.handle, param->write.len);
            
            if (param->write.handle == command_char_handle && ble_instance) {
                // Process received command
                esp_err_t ret = ble_controller_process_packet(ble_instance, 
                                                            param->write.value, 
                                                            param->write.len);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to process command packet");
                }
            }
            
            // Send response if needed
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if_local, param->write.conn_id, 
                                          param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
            
        default:
            ESP_LOGD(TAG, "GATTS event: %d", event);
            break;
    }
}

#endif // NATIVE_BUILD