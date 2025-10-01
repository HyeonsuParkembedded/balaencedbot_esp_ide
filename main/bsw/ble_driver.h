/**
 * @file ble_driver.h
 * @brief BLE (Bluetooth Low Energy) 추상화 드라이버 헤더 파일
 * 
 * ESP32-C6의 BLE 컨트롤러 레지스터를 직접 제어하는 BSW 계층 드라이버입니다.
 * 상위 계층에서 하드웨어에 종속되지 않는 BLE 통신을 제공합니다.
 * 
 * 주요 기능:
 * - 비트연산 기반 BLE 컨트롤러 직접 제어
 * - 소프트웨어 BLE 스택 구현
 * - 클라이언트 연결 관리
 * - 데이터 송수신 추상화
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-09-20
 * @version 1.0
 */

#ifndef BLE_DRIVER_H
#define BLE_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ESP32-C6 BLE 컨트롤러 레지스터 정의 (비트연산 방식)
#define BSW_BLE_CTRL_BASE           0x600A0000  // BLE 컨트롤러 베이스 주소
#define BSW_BLE_CTRL_CONFIG         (BSW_BLE_CTRL_BASE + 0x000)
#define BSW_BLE_CTRL_STATUS         (BSW_BLE_CTRL_BASE + 0x004)
#define BSW_BLE_CTRL_ENABLE         (BSW_BLE_CTRL_BASE + 0x008)
#define BSW_BLE_ADV_CONFIG          (BSW_BLE_CTRL_BASE + 0x100)
#define BSW_BLE_ADV_DATA            (BSW_BLE_CTRL_BASE + 0x104)
#define BSW_BLE_SCAN_CONFIG         (BSW_BLE_CTRL_BASE + 0x200)
#define BSW_BLE_CONN_CONFIG         (BSW_BLE_CTRL_BASE + 0x300)

// BLE 컨트롤러 제어 비트 정의
#define BSW_BLE_CTRL_RESET_BIT      (1 << 0)
#define BSW_BLE_CTRL_ENABLE_BIT     (1 << 1)
#define BSW_BLE_ADV_ENABLE_BIT      (1 << 0)
#define BSW_BLE_SCAN_ENABLE_BIT     (1 << 0)

// 비트연산 기반 레지스터 접근 매크로
#define BSW_BLE_REG_READ(addr)      (*((volatile uint32_t*)(addr)))
#define BSW_BLE_REG_WRITE(addr, val) (*((volatile uint32_t*)(addr)) = (val))
#define BSW_BLE_REG_SET_BIT(addr, bit) (*((volatile uint32_t*)(addr)) |= (bit))
#define BSW_BLE_REG_CLEAR_BIT(addr, bit) (*((volatile uint32_t*)(addr)) &= ~(bit))

// BSW 추상화 계층 - BLE 타입 정의

/**
 * @brief BLE 연결 핸들 타입
 */
typedef uint16_t ble_conn_handle_t;

/**
 * @brief BLE 특성 핸들 타입
 */
typedef uint16_t ble_char_handle_t;

/**
 * @brief BLE 서비스 핸들 타입
 */
typedef uint16_t ble_service_handle_t;

/**
 * @brief BLE 특성 속성
 */
typedef enum {
    BLE_CHAR_PROP_READ          = (1 << 0),
    BLE_CHAR_PROP_WRITE         = (1 << 1),
    BLE_CHAR_PROP_NOTIFY        = (1 << 2),
    BLE_CHAR_PROP_INDICATE      = (1 << 3),
    BLE_CHAR_PROP_WRITE_NO_RESP = (1 << 4)
} ble_char_properties_t;

/**
 * @brief BLE UUID 타입
 */
typedef struct {
    uint8_t type;           ///< UUID 타입 (16비트 또는 128비트)
    union {
        uint16_t uuid16;    ///< 16비트 UUID
        uint8_t uuid128[16]; ///< 128비트 UUID
    };
} ble_uuid_t;

/**
 * @brief BLE 이벤트 타입
 */
typedef enum {
    BLE_EVENT_CONNECTED,        ///< 클라이언트 연결됨
    BLE_EVENT_DISCONNECTED,     ///< 클라이언트 연결 해제됨
    BLE_EVENT_DATA_RECEIVED,    ///< 데이터 수신됨
    BLE_EVENT_SERVICE_STARTED   ///< 서비스 시작됨
} ble_event_type_t;

/**
 * @brief BLE 이벤트 데이터
 */
typedef struct {
    ble_event_type_t type;      ///< 이벤트 타입
    ble_conn_handle_t conn_handle; ///< 연결 핸들
    union {
        struct {
            ble_char_handle_t char_handle; ///< 특성 핸들
            const uint8_t* data;          ///< 수신된 데이터
            size_t length;                ///< 데이터 길이
        } data_received;
        
        struct {
            ble_service_handle_t service_handle; ///< 서비스 핸들
        } service_started;
    };
} ble_event_t;

/**
 * @brief BLE 이벤트 콜백 함수 타입
 */
typedef void (*ble_event_callback_t)(const ble_event_t* event, void* user_data);

/**
 * @defgroup BLE_DRIVER BLE 드라이버 API
 * @brief BLE 통신을 위한 추상화 함수들
 * @{
 */

/**
 * @brief BLE 드라이버 초기화
 * 
 * BLE 컨트롤러 레지스터를 직접 제어하여 초기화합니다.
 * 
 * @param device_name BLE 광고에 사용할 디바이스 이름
 * @param callback 이벤트 콜백 함수
 * @param user_data 콜백 함수에 전달될 사용자 데이터
 * @return bool 
 *         - true: 초기화 성공
 *         - false: 초기화 실패
 */
bool ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data);

/**
 * @brief BLE 서비스 생성 (비트연산 방식)
 * 
 * 소프트웨어 서비스 테이블에 새로운 서비스를 등록합니다.
 * 
 * @param service_uuid 서비스 UUID
 * @param service_handle 생성된 서비스 핸들 (출력)
 * @return bool 
 *         - true: 서비스 생성 성공
 *         - false: 서비스 생성 실패
 */
bool ble_create_service(const ble_uuid_t* service_uuid, ble_service_handle_t* service_handle);

/**
 * @brief BLE 특성 추가
 * 
 * 지정된 서비스에 새로운 특성을 추가합니다.
 * 
 * @param service_handle 서비스 핸들
 * @param char_uuid 특성 UUID
 * @param properties 특성 속성
 * @param char_handle 생성된 특성 핸들 (출력)
 * @return esp_err_t 
 *         - ESP_OK: 특성 추가 성공
 *         - ESP_FAIL: 특성 추가 실패
 */
bool ble_add_characteristic(ble_service_handle_t service_handle,
                           const ble_uuid_t* char_uuid,
                           ble_char_properties_t properties,
                           ble_char_handle_t* char_handle);/**
 * @brief BLE 서비스 시작
 * 
 * 생성된 서비스를 시작하여 클라이언트에게 제공합니다.
 * 
 * @param service_handle 서비스 핸들
 * @return esp_err_t 
 *         - ESP_OK: 서비스 시작 성공
 *         - ESP_FAIL: 서비스 시작 실패
 */
bool ble_start_service(ble_service_handle_t service_handle);

/**
 * @brief BLE 광고 시작 (비트연산 방식)
 * 
 * BLE 컨트롤러 레지스터를 직접 제어하여 광고를 시작합니다.
 * 
 * @return bool 
 *         - true: 광고 시작 성공
 *         - false: 광고 시작 실패
 */
bool ble_start_advertising(void);

/**
 * @brief BLE 데이터 전송 (비트연산 방식)
 * 
 * 소프트웨어 구현으로 클라이언트에게 데이터를 전송합니다.
 * 
 * @param conn_handle 연결 핸들
 * @param char_handle 특성 핸들
 * @param data 전송할 데이터
 * @param length 데이터 길이
 * @param is_notification true: 알림, false: 표시
 * @return bool 
 *         - true: 전송 성공
 *         - false: 전송 실패
 */
bool ble_send_data(ble_conn_handle_t conn_handle, 
                   ble_char_handle_t char_handle,
                   const uint8_t* data, 
                   size_t length,
                   bool is_notification);

/**
 * @brief BLE 연결 상태 확인
 * 
 * 지정된 연결이 활성화되어 있는지 확인합니다.
 * 
 * @param conn_handle 연결 핸들
 * @return bool 
 *         - true: 연결됨
 *         - false: 연결 안됨
 */
bool ble_is_connected(ble_conn_handle_t conn_handle);

/**
 * @brief UUID 생성 헬퍼 (16비트)
 * 
 * 16비트 UUID를 생성합니다.
 * 
 * @param uuid16 16비트 UUID 값
 * @return ble_uuid_t 생성된 UUID
 */
ble_uuid_t ble_uuid_from_16(uint16_t uuid16);

/**
 * @brief UUID 생성 헬퍼 (128비트)
 * 
 * 128비트 UUID를 생성합니다.
 * 
 * @param uuid128 128비트 UUID 배열
 * @return ble_uuid_t 생성된 UUID
 */
ble_uuid_t ble_uuid_from_128(const uint8_t uuid128[16]);

/**
 * @brief BLE 특성 추가 (비트연산 방식)
 * 
 * 소프트웨어 특성 테이블에 새로운 특성을 추가합니다.
 * 
 * @param service_handle 서비스 핸들
 * @param char_uuid 특성 UUID
 * @param properties 특성 속성
 * @param char_handle 생성된 특성 핸들 (출력)
 * @return bool 
 *         - true: 특성 추가 성공
 *         - false: 특성 추가 실패
 */
bool ble_add_characteristic(ble_service_handle_t service_handle,
                                   const ble_uuid_t* char_uuid,
                                   ble_char_properties_t properties,
                                   ble_char_handle_t* char_handle);

/**
 * @brief BLE 서비스 시작 (비트연산 방식)
 * 
 * 소프트웨어 서비스를 시작합니다.
 * 
 * @param service_handle 서비스 핸들
 * @return bool 
 *         - true: 서비스 시작 성공
 *         - false: 서비스 시작 실패
 */
bool ble_start_service(ble_service_handle_t service_handle);

/**
 * @brief BLE 광고 중지 (비트연산 방식)
 * 
 * BLE 컨트롤러 레지스터를 직접 제어하여 광고를 중지합니다.
 * 
 * @return bool 
 *         - true: 광고 중지 성공
 *         - false: 광고 중지 실패
 */
bool ble_stop_advertising(void);

/**
 * @brief BLE 연결 상태 확인 (비트연산 방식)
 * 
 * 소프트웨어 연결 테이블에서 연결 상태를 확인합니다.
 * 
 * @param conn_handle 연결 핸들
 * @return bool 
 *         - true: 연결됨
 *         - false: 연결 안됨
 */
bool ble_is_connected(ble_conn_handle_t conn_handle);

/**
 * @brief BLE 컨트롤러 상태 조회
 * 
 * BLE 컨트롤러 레지스터에서 상태를 읽어옵니다.
 * 
 * @return bool 
 *         - true: 컨트롤러 활성화됨
 *         - false: 컨트롤러 비활성화됨
 */
bool ble_get_controller_status(void);

/**
 * @brief BLE 광고 상태 조회
 * 
 * BLE 광고 레지스터에서 상태를 읽어옵니다.
 * 
 * @return bool 
 *         - true: 광고 중
 *         - false: 광고 중지됨
 */
bool ble_get_advertising_status(void);

/** @} */ // BLE_DRIVER

#ifdef __cplusplus
}
#endif

#endif // BLE_DRIVER_H