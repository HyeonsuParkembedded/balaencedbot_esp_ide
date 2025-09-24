/**
 * @file ble_driver.c
 * @brief BLE (Bluetooth Low Energy) 추상화 드라이버 구현 파일
 * 
 * ESP32의 BLE 기능을 추상화한 BSW 계층 드라이버 구현입니다.
 * ESP-IDF BLE 스택을 사용하여 실제 BLE 통신 기능을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 1.0
 */

#include "ble_driver.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include <string.h>

static const char* TAG = "BLE_DRIVER";

// BLE 드라이버 내부 상태 관리
typedef struct {
    bool initialized;
    char device_name[32];
    ble_event_callback_t event_callback;
    void* user_data;
    esp_gatt_if_t gatts_if;
    uint16_t app_id;
    esp_ble_adv_data_t adv_data;
    esp_ble_adv_params_t adv_params;
} ble_driver_context_t;

static ble_driver_context_t g_ble_ctx = {0};

// 서비스 및 특성 관리
#define MAX_SERVICES 4
#define MAX_CHARACTERISTICS 8

typedef struct {
    ble_service_handle_t handle;
    esp_gatt_srvc_id_t service_id;
    bool in_use;
    bool started;
} service_info_t;

typedef struct {
    ble_char_handle_t handle;
    uint16_t char_handle;
    ble_service_handle_t service_handle;
    ble_char_properties_t properties;
    bool in_use;
} char_info_t;

static service_info_t g_services[MAX_SERVICES] = {0};
static char_info_t g_characteristics[MAX_CHARACTERISTICS] = {0};
static uint16_t g_next_service_handle = 1;
static uint16_t g_next_char_handle = 1;

// 연결 관리
#define MAX_CONNECTIONS 4
typedef struct {
    ble_conn_handle_t handle;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    bool connected;
} connection_info_t;

static connection_info_t g_connections[MAX_CONNECTIONS] = {0};

// 내부 함수 선언
static esp_err_t setup_advertising(void);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);
static service_info_t* find_service_by_handle(ble_service_handle_t handle);
static char_info_t* find_char_by_handle(ble_char_handle_t handle);
static connection_info_t* find_connection_by_conn_id(uint16_t conn_id);
static connection_info_t* get_free_connection_slot(void);

// BLE 드라이버 초기화
esp_err_t ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data)
{
    ESP_LOGI(TAG, "Initializing BLE driver...");
    
    if (g_ble_ctx.initialized) {
        ESP_LOGW(TAG, "BLE driver already initialized");
        return ESP_OK;
    }
    
    if (!device_name || !callback) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }
    
    // NVS 초기화 (BLE 스택에 필요)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Bluetooth 컨트롤러 초기화
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Bluedroid 스택 초기화
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 콜백 등록
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // MTU 설정
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 컨텍스트 초기화
    strncpy(g_ble_ctx.device_name, device_name, sizeof(g_ble_ctx.device_name) - 1);
    g_ble_ctx.device_name[sizeof(g_ble_ctx.device_name) - 1] = '\0';
    g_ble_ctx.event_callback = callback;
    g_ble_ctx.user_data = user_data;
    g_ble_ctx.app_id = 0;
    
    // GATTS 앱 등록
    ret = esp_ble_gatts_app_register(g_ble_ctx.app_id);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE driver initialized successfully");
    return ESP_OK;
}

// BLE 서비스 생성
esp_err_t ble_create_service(const ble_uuid_t* service_uuid, ble_service_handle_t* service_handle)
{
    if (!g_ble_ctx.initialized || !service_uuid || !service_handle) {
        return ESP_FAIL;
    }
    
    // 빈 서비스 슬롯 찾기
    service_info_t* service = NULL;
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (!g_services[i].in_use) {
            service = &g_services[i];
            break;
        }
    }
    
    if (!service) {
        ESP_LOGE(TAG, "No free service slots");
        return ESP_FAIL;
    }
    
    // 서비스 ID 설정
    service->handle = g_next_service_handle++;
    service->service_id.is_primary = true;
    service->service_id.id.inst_id = 0;
    
    if (service_uuid->type == 0) { // 16비트 UUID
        service->service_id.id.uuid.len = ESP_UUID_LEN_16;
        service->service_id.id.uuid.uuid.uuid16 = service_uuid->uuid16;
    } else { // 128비트 UUID
        service->service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(service->service_id.id.uuid.uuid.uuid128, service_uuid->uuid128, 16);
    }
    
    service->in_use = true;
    service->started = false;
    
    *service_handle = service->handle;
    
    ESP_LOGI(TAG, "Service created with handle %d", service->handle);
    return ESP_OK;
}

// BLE 특성 추가
esp_err_t ble_add_characteristic(ble_service_handle_t service_handle, 
                                const ble_uuid_t* char_uuid,
                                ble_char_properties_t properties,
                                ble_char_handle_t* char_handle)
{
    if (!g_ble_ctx.initialized || !char_uuid || !char_handle) {
        return ESP_FAIL;
    }
    
    // 서비스 찾기
    service_info_t* service = find_service_by_handle(service_handle);
    if (!service) {
        ESP_LOGE(TAG, "Service not found: %d", service_handle);
        return ESP_FAIL;
    }
    
    // 빈 특성 슬롯 찾기
    char_info_t* characteristic = NULL;
    for (int i = 0; i < MAX_CHARACTERISTICS; i++) {
        if (!g_characteristics[i].in_use) {
            characteristic = &g_characteristics[i];
            break;
        }
    }
    
    if (!characteristic) {
        ESP_LOGE(TAG, "No free characteristic slots");
        return ESP_FAIL;
    }
    
    // 특성 정보 설정
    characteristic->handle = g_next_char_handle++;
    characteristic->service_handle = service_handle;
    characteristic->properties = properties;
    characteristic->in_use = true;
    
    *char_handle = characteristic->handle;
    
    ESP_LOGI(TAG, "Characteristic added with handle %d to service %d", 
             characteristic->handle, service_handle);
    return ESP_OK;
}

// BLE 서비스 시작
esp_err_t ble_start_service(ble_service_handle_t service_handle)
{
    if (!g_ble_ctx.initialized) {
        return ESP_FAIL;
    }
    
    service_info_t* service = find_service_by_handle(service_handle);
    if (!service) {
        ESP_LOGE(TAG, "Service not found: %d", service_handle);
        return ESP_FAIL;
    }
    
    if (service->started) {
        ESP_LOGW(TAG, "Service %d already started", service_handle);
        return ESP_OK;
    }
    
    // ESP-IDF GATTS 서비스 생성
    esp_err_t ret = esp_ble_gatts_create_service(g_ble_ctx.gatts_if, 
                                                &service->service_id, 
                                                4); // 핸들 수
    if (ret) {
        ESP_LOGE(TAG, "Create service failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    service->started = true;
    
    ESP_LOGI(TAG, "Service %d started", service_handle);
    return ESP_OK;
}

// BLE 광고 시작
esp_err_t ble_start_advertising(void)
{
    if (!g_ble_ctx.initialized) {
        return ESP_FAIL;
    }
    
    esp_err_t ret = setup_advertising();
    if (ret) {
        ESP_LOGE(TAG, "Setup advertising failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gap_start_advertising(&g_ble_ctx.adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE advertising started");
    return ESP_OK;
}

// BLE 데이터 전송
esp_err_t ble_send_data(ble_conn_handle_t conn_handle, 
                       ble_char_handle_t char_handle,
                       const uint8_t* data, 
                       size_t length,
                       bool is_notification)
{
    if (!g_ble_ctx.initialized || !data || length == 0) {
        return ESP_FAIL;
    }
    
    connection_info_t* conn = find_connection_by_conn_id(conn_handle);
    if (!conn || !conn->connected) {
        ESP_LOGE(TAG, "Connection not found or not connected: %d", conn_handle);
        return ESP_FAIL;
    }
    
    char_info_t* characteristic = find_char_by_handle(char_handle);
    if (!characteristic) {
        ESP_LOGE(TAG, "Characteristic not found: %d", char_handle);
        return ESP_FAIL;
    }
    
    esp_err_t ret;
    if (is_notification) {
        ret = esp_ble_gatts_send_indicate(g_ble_ctx.gatts_if, 
                                         conn->conn_id,
                                         characteristic->char_handle,
                                         length, 
                                         (uint8_t*)data, 
                                         false);
    } else {
        ret = esp_ble_gatts_send_indicate(g_ble_ctx.gatts_if, 
                                         conn->conn_id,
                                         characteristic->char_handle,
                                         length, 
                                         (uint8_t*)data, 
                                         true);
    }
    
    if (ret) {
        ESP_LOGE(TAG, "Send data failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// BLE 연결 상태 확인
bool ble_is_connected(ble_conn_handle_t conn_handle)
{
    connection_info_t* conn = find_connection_by_conn_id(conn_handle);
    return (conn && conn->connected);
}

// UUID 헬퍼 함수들
ble_uuid_t ble_uuid_from_16(uint16_t uuid16)
{
    ble_uuid_t uuid = {0};
    uuid.type = 0; // 16비트
    uuid.uuid16 = uuid16;
    return uuid;
}

ble_uuid_t ble_uuid_from_128(const uint8_t uuid128[16])
{
    ble_uuid_t uuid = {0};
    uuid.type = 1; // 128비트
    memcpy(uuid.uuid128, uuid128, 16);
    return uuid;
}

// 내부 함수 구현들

static esp_err_t setup_advertising(void)
{
    // 광고 데이터 설정
    g_ble_ctx.adv_data.set_scan_rsp = false;
    g_ble_ctx.adv_data.include_name = true;
    g_ble_ctx.adv_data.include_txpower = true;
    g_ble_ctx.adv_data.min_interval = 0x0006;
    g_ble_ctx.adv_data.max_interval = 0x0010;
    g_ble_ctx.adv_data.appearance = 0x00;
    g_ble_ctx.adv_data.manufacturer_len = 0;
    g_ble_ctx.adv_data.p_manufacturer_data = NULL;
    g_ble_ctx.adv_data.service_data_len = 0;
    g_ble_ctx.adv_data.p_service_data = NULL;
    g_ble_ctx.adv_data.service_uuid_len = 0;
    g_ble_ctx.adv_data.p_service_uuid = NULL;
    g_ble_ctx.adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    
    esp_err_t ret = esp_ble_gap_config_adv_data(&g_ble_ctx.adv_data);
    if (ret) {
        ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 광고 파라미터 설정
    g_ble_ctx.adv_params.adv_int_min = 0x20;
    g_ble_ctx.adv_params.adv_int_max = 0x40;
    g_ble_ctx.adv_params.adv_type = ADV_TYPE_IND;
    g_ble_ctx.adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    g_ble_ctx.adv_params.channel_map = ADV_CHNL_ALL;
    g_ble_ctx.adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    
    return ESP_OK;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising stopped");
            break;
            
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATTS app registered, status %d, app_id %d", param->reg.status, param->reg.app_id);
            g_ble_ctx.gatts_if = gatts_if;
            g_ble_ctx.initialized = true;
            
            // 디바이스 이름 설정
            esp_ble_gap_set_device_name(g_ble_ctx.device_name);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id %d", param->connect.conn_id);
            
            // 연결 정보 저장
            connection_info_t* conn = get_free_connection_slot();
            if (conn) {
                conn->handle = param->connect.conn_id;
                conn->conn_id = param->connect.conn_id;
                memcpy(conn->remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
                conn->connected = true;
                
                // 이벤트 콜백 호출
                if (g_ble_ctx.event_callback) {
                    ble_event_t event = {0};
                    event.type = BLE_EVENT_CONNECTED;
                    event.conn_handle = conn->handle;
                    g_ble_ctx.event_callback(&event, g_ble_ctx.user_data);
                }
            }
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, conn_id %d", param->disconnect.conn_id);
            
            // 연결 정보 정리
            connection_info_t* disconn = find_connection_by_conn_id(param->disconnect.conn_id);
            if (disconn) {
                // 이벤트 콜백 호출
                if (g_ble_ctx.event_callback) {
                    ble_event_t event = {0};
                    event.type = BLE_EVENT_DISCONNECTED;
                    event.conn_handle = disconn->handle;
                    g_ble_ctx.event_callback(&event, g_ble_ctx.user_data);
                }
                
                disconn->connected = false;
                memset(disconn, 0, sizeof(connection_info_t));
            }
            
            // 광고 재시작
            esp_ble_gap_start_advertising(&g_ble_ctx.adv_params);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            // 데이터 수신 이벤트
            if (g_ble_ctx.event_callback) {
                // 특성 핸들로 특성 찾기
                char_info_t* characteristic = NULL;
                for (int i = 0; i < MAX_CHARACTERISTICS; i++) {
                    if (g_characteristics[i].in_use && 
                        g_characteristics[i].char_handle == param->write.handle) {
                        characteristic = &g_characteristics[i];
                        break;
                    }
                }
                
                if (characteristic) {
                    ble_event_t event = {0};
                    event.type = BLE_EVENT_DATA_RECEIVED;
                    event.conn_handle = param->write.conn_id;
                    event.data_received.char_handle = characteristic->handle;
                    event.data_received.data = param->write.value;
                    event.data_received.length = param->write.len;
                    g_ble_ctx.event_callback(&event, g_ble_ctx.user_data);
                }
            }
            break;
            
        default:
            break;
    }
}

// 헬퍼 함수들
static service_info_t* find_service_by_handle(ble_service_handle_t handle)
{
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (g_services[i].in_use && g_services[i].handle == handle) {
            return &g_services[i];
        }
    }
    return NULL;
}

static char_info_t* find_char_by_handle(ble_char_handle_t handle)
{
    for (int i = 0; i < MAX_CHARACTERISTICS; i++) {
        if (g_characteristics[i].in_use && g_characteristics[i].handle == handle) {
            return &g_characteristics[i];
        }
    }
    return NULL;
}

static connection_info_t* find_connection_by_conn_id(uint16_t conn_id)
{
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (g_connections[i].connected && g_connections[i].conn_id == conn_id) {
            return &g_connections[i];
        }
    }
    return NULL;
}

static connection_info_t* get_free_connection_slot(void)
{
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (!g_connections[i].connected) {
            return &g_connections[i];
        }
    }
    return NULL;
}