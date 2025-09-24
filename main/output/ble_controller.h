/**
 * @file ble_controller.h
 * @brief BLE (Bluetooth Low Energy) 컨트롤러 인터페이스
 * 
 * 이 모듈은 ESP32의 BLE 기능을 사용하여 외부 기기(모바일 앱)와의
 * 무선 통신을 제공합니다. 원격 제어 명령 수신 및 로봇 상태 전송을 담당합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#ifndef BLE_CONTROLLER_H
#define BLE_CONTROLLER_H

#ifndef NATIVE_BUILD
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_err.h"
#else
// Native build - types defined in test file
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// BLE Service and Characteristic UUIDs (128-bit UUIDs)
// These match the UUIDs used in the Flutter app
// Service: 0000FF00-0000-1000-8000-00805F9B34FB
// Command: 0000FF01-0000-1000-8000-00805F9B34FB  
// Status:  0000FF02-0000-1000-8000-00805F9B34FB
#define BLE_SERVICE_UUID        0x00FF       ///< BLE 서비스 UUID
#define BLE_COMMAND_CHAR_UUID   0xFF01       ///< 명령 특성 UUID
#define BLE_STATUS_CHAR_UUID    0xFF02       ///< 상태 특성 UUID

/**
 * @brief 원격 제어 명령 구조체
 * 
 * BLE를 통해 수신되는 모바일 앱의 제어 명령을 담는 구조체입니다.
 */
typedef struct {
    int direction;    ///< 방향 (0: 정지, 1: 전진, -1: 후진)
    int turn;         ///< 회전 (-100~100, 좌측에서 우측)
    int speed;        ///< 속도 (0~100)
    bool balance;     ///< 밸런싱 활성화/비활성화
    bool standup;     ///< 기립 명령
} remote_command_t;

/**
 * @brief BLE 컨트롤러 상태 구조체
 * 
 * BLE 연결 상태와 통신 관련 정보를 관리하는 구조체입니다.
 */
typedef struct {
    bool device_connected;           ///< 기기 연결 상태
    remote_command_t current_command; ///< 현재 수신된 명령
    char last_command[64];           ///< 마지막 수신 명령 문자열
    uint16_t gatts_if;              ///< GATT 서버 인터페이스
    uint16_t conn_id;               ///< 연결 ID
    uint16_t command_handle;        ///< 명령 특성 핸들
    uint16_t status_handle;         ///< 상태 특성 핸들
} ble_controller_t;

/**
 * @brief BLE 컨트롤러 초기화
 * 
 * BLE 스택을 초기화하고 GATT 서버를 시작합니다.
 * 지정된 디바이스명으로 광고를 시작합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param device_name BLE 광고에 사용할 디바이스 이름
 * @return esp_err_t 초기화 결과
 * @retval ESP_OK 성공
 * @retval ESP_FAIL 실패
 */
esp_err_t ble_controller_init(ble_controller_t* ble, const char* device_name);

/**
 * @brief BLE 컨트롤러 업데이트
 * 
 * BLE 이벤트를 처리하고 연결 상태를 갱신합니다.
 * 주기적으로 호출되어야 합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 */
void ble_controller_update(ble_controller_t* ble);

/**
 * @brief 원격 제어 명령 가져오기
 * 
 * 현재 수신된 원격 제어 명령을 반환합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @return remote_command_t 현재 명령
 */
remote_command_t ble_controller_get_command(const ble_controller_t* ble);

/**
 * @brief BLE 연결 상태 확인
 * 
 * BLE 기기가 연결되어 있는지 확인합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @return bool 연결 상태
 * @retval true 연결됨
 * @retval false 연결 안됨
 */
bool ble_controller_is_connected(const ble_controller_t* ble);

/**
 * @brief 로봇 상태 전송
 * 
 * 로봇의 현재 상태(각도, 속도, 배터리 전압)를 BLE를 통해 전송합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param angle 로봇 기울기 각도 (도)
 * @param velocity 로봇 속도 (m/s)
 * @param battery_voltage 배터리 전압 (V)
 * @return esp_err_t 전송 결과
 * @retval ESP_OK 성공
 * @retval ESP_FAIL 실패
 */
esp_err_t ble_controller_send_status(ble_controller_t* ble, float angle, float velocity, float battery_voltage);

/**
 * @brief BLE 패킷 처리
 * 
 * 수신된 BLE 데이터 패킷을 파싱하여 명령으로 변환합니다.
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param data 수신된 데이터
 * @param length 데이터 길이
 * @return esp_err_t 처리 결과
 * @retval ESP_OK 성공
 * @retval ESP_FAIL 실패
 */
esp_err_t ble_controller_process_packet(ble_controller_t* ble, const uint8_t* data, size_t length);

/**
 * @brief 명령 문자열 파싱 (레거시)
 * 
 * 문자열 형태의 명령을 파싱합니다. (이전 버전 호환용)
 * 
 * @param ble BLE 컨트롤러 구조체 포인터
 * @param command 명령 문자열
 * @deprecated 새로운 바이너리 프로토콜 사용 권장
 */
void ble_controller_parse_command(ble_controller_t* ble, const char* command); // Legacy function

#ifdef __cplusplus
}
#endif

#endif