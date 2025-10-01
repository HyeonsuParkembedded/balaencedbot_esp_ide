/**
 * @file ble_driver_bitwise.c
 * @brief BLE (Bluetooth Low Energy) 비트연산 드라이버 구현
 * 
 * ESP32-C6의 BLE 컨트롤러 레지스터를 직접 제어하는 순수 비트연산 구현bool ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data) {니다.
 * ESP-IDF 함수를 사용하지 않고 하드웨어 레지스bool ble_create_service(const ble_uuid_t* service_uuid, ble_service_handle_t* service_handle) {만을 사용합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 2.0bool ble_start_advertising(void) { */

#include "ble_driver.h"
#include <string.h>

static const char* TAG = "BLE_DRIVER_BITWISE";

// BSW BLE 드라이버 내부 상태 관리
typedef struct {
    bool initialized;
    char device_name[32];
    ble_event_callback_t event_callback;
    void* user_data;
    uint32_t controller_state;
    uint32_t adv_config;
    uint8_t adv_data[31];  // BLE 광고 데이터 최대 크기
    uint8_t adv_data_len;
    uint16_t connection_count;
} bsw_ble_context_t;

static bsw_ble_context_t g_bsw_ble = {0};

// BSW BLE 서비스 관리 (소프트웨어 구현)
#define BSW_MAX_SERVICES 8
#define BSW_MAX_CHARACTERISTICS 16

typedef struct {
    ble_service_handle_t handle;
    ble_uuid_t uuid;
    bool in_use;
    bool started;
} bsw_service_info_t;

typedef struct {
    ble_char_handle_t handle;
    ble_service_handle_t service_handle;
    ble_uuid_t uuid;
    ble_char_properties_t properties;
    bool in_use;
    uint8_t value[64];  // 특성 값 저장
    size_t value_len;
} bsw_char_info_t;

static bsw_service_info_t g_bsw_services[BSW_MAX_SERVICES] = {0};
static bsw_char_info_t g_bsw_characteristics[BSW_MAX_CHARACTERISTICS] = {0};
static uint16_t g_next_service_handle = 1;
static uint16_t g_next_char_handle = 1;

// BSW BLE 연결 관리 (소프트웨어 구현)
#define BSW_MAX_CONNECTIONS 4

typedef struct {
    ble_conn_handle_t handle;
    bool connected;
    uint8_t remote_addr[6];  // BLE MAC 주소
} bsw_connection_info_t;

static bsw_connection_info_t g_bsw_connections[BSW_MAX_CONNECTIONS] = {0};

// 내부 함수 선언
static void bsw_ble_delay_us_inline(uint32_t us);
static bool bsw_ble_controller_reset_bitwise(void);
static bool bsw_ble_controller_enable_bitwise(void);
static void bsw_ble_setup_default_adv_data(void);
static bsw_service_info_t* bsw_find_service_by_handle(ble_service_handle_t handle);
static bsw_char_info_t* bsw_find_char_by_handle(ble_char_handle_t handle);

// BSW 인라인 지연 함수 (CPU 사이클 기반)
static void bsw_ble_delay_us_inline(uint32_t us)
{
    volatile uint32_t cycles = us * 160;  // ESP32-C6 @ 160MHz 기준
    while (cycles--) {
        __asm__ __volatile__("nop");
    }
}

// BSW BLE 컨트롤러 리셋 (비트연산)
static bool bsw_ble_controller_reset_bitwise(void)
{
    // BLE 컨트롤러 리셋 비트 설정
    BSW_BLE_REG_SET_BIT(BSW_BLE_CTRL_CONFIG, BSW_BLE_CTRL_RESET_BIT);
    bsw_ble_delay_us_inline(1000);  // 1ms 대기
    
    // 리셋 비트 해제
    BSW_BLE_REG_CLEAR_BIT(BSW_BLE_CTRL_CONFIG, BSW_BLE_CTRL_RESET_BIT);
    bsw_ble_delay_us_inline(1000);  // 안정화 대기
    
    // 리셋 완료 확인
    uint32_t status = BSW_BLE_REG_READ(BSW_BLE_CTRL_STATUS);
    return (status & 0x01) == 0x01;  // 준비 상태 확인
}

// BSW BLE 컨트롤러 활성화 (비트연산)
static bool bsw_ble_controller_enable_bitwise(void)
{
    // BLE 컨트롤러 활성화
    BSW_BLE_REG_SET_BIT(BSW_BLE_CTRL_ENABLE, BSW_BLE_CTRL_ENABLE_BIT);
    bsw_ble_delay_us_inline(5000);  // 5ms 초기화 대기
    
    // 활성화 상태 확인
    uint32_t status = BSW_BLE_REG_READ(BSW_BLE_CTRL_STATUS);
    return (status & 0x03) == 0x03;  // 활성화 및 준비 상태
}

// BSW 기본 광고 데이터 설정
static void bsw_ble_setup_default_adv_data(void)
{
    uint8_t* data = g_bsw_ble.adv_data;
    uint8_t len = 0;
    
    // 디바이스 이름 추가 (최대 18바이트)
    size_t name_len = strlen(g_bsw_ble.device_name);
    if (name_len > 18) name_len = 18;
    
    data[len++] = name_len + 1;      // 길이
    data[len++] = 0x09;              // Complete Local Name
    memcpy(&data[len], g_bsw_ble.device_name, name_len);
    len += name_len;
    
    // Flags 추가
    data[len++] = 0x02;              // 길이
    data[len++] = 0x01;              // Flags
    data[len++] = 0x06;              // LE General Discoverable + BR/EDR Not Supported
    
    g_bsw_ble.adv_data_len = len;
}

// BSW BLE 드라이버 초기화 (비트연산 방식)
bool ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data)
{
    if (g_bsw_ble.initialized) {
        return true;  // 이미 초기화됨
    }
    
    if (!device_name || !callback) {
        return false;  // 잘못된 파라미터
    }
    
    // BLE 컨트롤러 리셋
    if (!bsw_ble_controller_reset_bitwise()) {
        return false;
    }
    
    // BLE 컨트롤러 활성화
    if (!bsw_ble_controller_enable_bitwise()) {
        return false;
    }
    
    // 컨텍스트 초기화
    strncpy(g_bsw_ble.device_name, device_name, sizeof(g_bsw_ble.device_name) - 1);
    g_bsw_ble.device_name[sizeof(g_bsw_ble.device_name) - 1] = '\0';
    g_bsw_ble.event_callback = callback;
    g_bsw_ble.user_data = user_data;
    g_bsw_ble.controller_state = 0x03;  // 활성화됨
    g_bsw_ble.adv_config = 0;
    g_bsw_ble.connection_count = 0;
    
    // 기본 광고 데이터 설정
    bsw_ble_setup_default_adv_data();
    
    g_bsw_ble.initialized = true;
    return true;
}

// BSW BLE 서비스 생성 (소프트웨어 구현)
bool ble_create_service(const ble_uuid_t* service_uuid, ble_service_handle_t* service_handle)
{
    if (!g_bsw_ble.initialized || !service_uuid || !service_handle) {
        return false;
    }
    
    // 빈 서비스 슬롯 찾기
    bsw_service_info_t* service = NULL;
    for (int i = 0; i < BSW_MAX_SERVICES; i++) {
        if (!g_bsw_services[i].in_use) {
            service = &g_bsw_services[i];
            break;
        }
    }
    
    if (!service) {
        return false;  // 서비스 슬롯 없음
    }
    
    // 서비스 정보 설정
    service->handle = g_next_service_handle++;
    service->uuid = *service_uuid;  // UUID 복사
    service->in_use = true;
    service->started = false;
    
    *service_handle = service->handle;
    return true;
}

// BSW BLE 특성 추가 (소프트웨어 구현)
bool ble_add_characteristic(ble_service_handle_t service_handle,
                                   const ble_uuid_t* char_uuid,
                                   ble_char_properties_t properties,
                                   ble_char_handle_t* char_handle)
{
    if (!g_bsw_ble.initialized || !char_uuid || !char_handle) {
        return false;
    }
    
    // 서비스 존재 확인
    bsw_service_info_t* service = bsw_find_service_by_handle(service_handle);
    if (!service) {
        return false;
    }
    
    // 빈 특성 슬롯 찾기
    bsw_char_info_t* characteristic = NULL;
    for (int i = 0; i < BSW_MAX_CHARACTERISTICS; i++) {
        if (!g_bsw_characteristics[i].in_use) {
            characteristic = &g_bsw_characteristics[i];
            break;
        }
    }
    
    if (!characteristic) {
        return false;  // 특성 슬롯 없음
    }
    
    // 특성 정보 설정
    characteristic->handle = g_next_char_handle++;
    characteristic->service_handle = service_handle;
    characteristic->uuid = *char_uuid;  // UUID 복사
    characteristic->properties = properties;
    characteristic->in_use = true;
    characteristic->value_len = 0;
    
    *char_handle = characteristic->handle;
    return true;
}

// BSW BLE 서비스 시작 (소프트웨어 구현)
bool ble_start_service(ble_service_handle_t service_handle)
{
    if (!g_bsw_ble.initialized) {
        return false;
    }
    
    bsw_service_info_t* service = bsw_find_service_by_handle(service_handle);
    if (!service) {
        return false;
    }
    
    if (service->started) {
        return true;  // 이미 시작됨
    }
    
    service->started = true;
    return true;
}

// BSW BLE 광고 시작 (비트연산 방식)
bool ble_start_advertising(void)
{
    if (!g_bsw_ble.initialized) {
        return false;
    }
    
    // 광고 데이터 레지스터에 로드
    volatile uint32_t* adv_data_reg = (volatile uint32_t*)BSW_BLE_ADV_DATA;
    uint32_t* data_words = (uint32_t*)g_bsw_ble.adv_data;
    
    // 광고 데이터를 32비트 단위로 레지스터에 기록
    for (int i = 0; i < (g_bsw_ble.adv_data_len + 3) / 4; i++) {
        adv_data_reg[i] = data_words[i];
    }
    
    // 광고 설정 레지스터 구성
    uint32_t adv_config = 0;
    adv_config |= (g_bsw_ble.adv_data_len & 0x1F);  // 데이터 길이 (비트 0-4)
    adv_config |= (0x00 << 5);   // 광고 타입: ADV_IND (비트 5-7)
    adv_config |= (0x07 << 8);   // 광고 채널: 37,38,39 (비트 8-10)
    adv_config |= (100 << 16);   // 광고 간격: 100ms (비트 16-31)
    
    BSW_BLE_REG_WRITE(BSW_BLE_ADV_CONFIG, adv_config);
    
    // 광고 시작
    BSW_BLE_REG_SET_BIT(BSW_BLE_ADV_CONFIG, BSW_BLE_ADV_ENABLE_BIT);
    
    g_bsw_ble.adv_config = adv_config;
    return true;
}

// BSW BLE 광고 중지 (비트연산 방식)
bool ble_stop_advertising(void)
{
    if (!g_bsw_ble.initialized) {
        return false;
    }
    
    // 광고 중지
    BSW_BLE_REG_CLEAR_BIT(BSW_BLE_ADV_CONFIG, BSW_BLE_ADV_ENABLE_BIT);
    
    g_bsw_ble.adv_config = 0;
    return true;
}

// BSW BLE 데이터 전송 (소프트웨어 구현)
bool ble_send_data(ble_conn_handle_t conn_handle,
                          ble_char_handle_t char_handle,
                          const uint8_t* data,
                          size_t length,
                          bool is_notification)
{
    if (!g_bsw_ble.initialized || !data || length == 0 || length > 64) {
        return false;
    }
    
    // 연결 확인
    bool connection_found = false;
    for (int i = 0; i < BSW_MAX_CONNECTIONS; i++) {
        if (g_bsw_connections[i].handle == conn_handle && g_bsw_connections[i].connected) {
            connection_found = true;
            break;
        }
    }
    
    if (!connection_found) {
        return false;  // 연결이 없음
    }
    
    // 특성 확인
    bsw_char_info_t* characteristic = bsw_find_char_by_handle(char_handle);
    if (!characteristic) {
        return false;
    }
    
    // 데이터 복사 (실제 전송은 소프트웨어 시뮬레이션)
    if (length <= sizeof(characteristic->value)) {
        memcpy(characteristic->value, data, length);
        characteristic->value_len = length;
    }
    
    // 이벤트 콜백 호출
    if (g_bsw_ble.event_callback) {
        ble_event_t event = {0};
        event.type = BLE_EVENT_DATA_RECEIVED;
        event.conn_handle = conn_handle;
        event.data_received.char_handle = char_handle;
        event.data_received.data = data;
        event.data_received.length = length;
        
        g_bsw_ble.event_callback(&event, g_bsw_ble.user_data);
    }
    
    return true;
}

// BSW BLE 연결 상태 확인
bool ble_is_connected(ble_conn_handle_t conn_handle)
{
    for (int i = 0; i < BSW_MAX_CONNECTIONS; i++) {
        if (g_bsw_connections[i].handle == conn_handle && g_bsw_connections[i].connected) {
            return true;
        }
    }
    return false;
}

// BSW 헬퍼 함수들
static bsw_service_info_t* bsw_find_service_by_handle(ble_service_handle_t handle)
{
    for (int i = 0; i < BSW_MAX_SERVICES; i++) {
        if (g_bsw_services[i].in_use && g_bsw_services[i].handle == handle) {
            return &g_bsw_services[i];
        }
    }
    return NULL;
}

static bsw_char_info_t* bsw_find_char_by_handle(ble_char_handle_t handle)
{
    for (int i = 0; i < BSW_MAX_CHARACTERISTICS; i++) {
        if (g_bsw_characteristics[i].in_use && g_bsw_characteristics[i].handle == handle) {
            return &g_bsw_characteristics[i];
        }
    }
    return NULL;
}

// BSW BLE 상태 조회 함수
bool ble_get_controller_status(void)
{
    if (!g_bsw_ble.initialized) {
        return false;
    }
    
    uint32_t status = BSW_BLE_REG_READ(BSW_BLE_CTRL_STATUS);
    return (status & 0x03) == 0x03;  // 활성화 및 준비 상태 확인
}

// BSW BLE 광고 상태 조회 함수
bool ble_get_advertising_status(void)
{
    if (!g_bsw_ble.initialized) {
        return false;
    }
    
    uint32_t config = BSW_BLE_REG_READ(BSW_BLE_ADV_CONFIG);
    return (config & BSW_BLE_ADV_ENABLE_BIT) != 0;
}

// UUID 헬퍼 함수들 (기존과 동일)
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