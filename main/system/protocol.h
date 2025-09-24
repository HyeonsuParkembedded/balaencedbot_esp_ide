/**
 * @file protocol.h
 * @brief 통신 프로토콜 정의 및 메시지 처리 인터페이스
 * 
 * BLE 및 다른 통신 채널을 통한 로봇 제어 및 상태 교환을 위한
 * 바이너리 프로토콜을 정의합니다. 안전하고 효율적인 통신을 제공합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Protocol version
#define PROTOCOL_VERSION 0x01    ///< 프로토콜 버전

// Message types
#define MSG_TYPE_MOVE_CMD       0x01  ///< 이동 명령
#define MSG_TYPE_STATUS_REQ     0x02  ///< 상태 요청
#define MSG_TYPE_STATUS_RESP    0x03  ///< 상태 응답
#define MSG_TYPE_CONFIG_SET     0x04  ///< 설정 변경
#define MSG_TYPE_CONFIG_GET     0x05  ///< 설정 조회
#define MSG_TYPE_ERROR          0xFF  ///< 오류 메시지

// Command flags
#define CMD_FLAG_BALANCE        0x01  ///< 밸런싱 활성화
#define CMD_FLAG_STANDUP        0x02  ///< 기립 명령
#define CMD_FLAG_EMERGENCY      0x04  ///< 비상 정지

// Maximum payload size
#define MAX_PAYLOAD_SIZE        64    ///< 최대 페이로드 크기 (바이트)

/**
 * @brief 프로토콜 헤더 구조체 (고정 8바이트)
 * 
 * 모든 프로토콜 메시지의 공통 헤더입니다.
 * 메시지 타입, 길이, 체크섬 등의 메타데이터를 포함합니다.
 */
typedef struct __attribute__((packed)) {
    uint8_t start_marker;       ///< 시작 마커 (항상 0xAA)
    uint8_t version;           ///< 프로토콜 버전
    uint8_t msg_type;          ///< 메시지 타입
    uint8_t seq_num;           ///< 시퀀스 번호
    uint16_t payload_len;      ///< 페이로드 길이
    uint16_t checksum;         ///< CRC16 체크섬
} protocol_header_t;

/**
 * @brief 이동 명령 페이로드
 * 
 * 로봇의 이동 및 제어 명령을 담는 구조체입니다.
 */
typedef struct __attribute__((packed)) {
    int8_t direction;          ///< 방향 (-1: 후진, 0: 정지, 1: 전진)
    int8_t turn;              ///< 회전 (-100 ~ 100, 좌측에서 우측)
    uint8_t speed;            ///< 속도 (0 ~ 100)
    uint8_t flags;            ///< 명령 플래그 (밸런싱, 기립 등)
    uint32_t timestamp;       ///< 명령 타임스탬프 (ms)
} move_command_payload_t;

/**
 * @brief 상태 응답 페이로드
 * 
 * 로봇의 현재 상태 정보를 담는 구조체입니다.
 */
typedef struct __attribute__((packed)) {
    float angle;              ///< 현재 기울기 각도 (도)
    float velocity;           ///< 현재 속도 (m/s)
    uint8_t robot_state;      ///< 로봇 상태 (안정, 넘어짐 등)
    uint8_t gps_status;       ///< GPS 상태 플래그
    float latitude;           ///< GPS 위도 (사용 가능한 경우)
    float longitude;          ///< GPS 경도 (사용 가능한 경우)
    uint8_t battery_level;    ///< 배터리 잔량 (%)
    uint8_t error_flags;      ///< 오류 상태 플래그
} status_response_payload_t;

/**
 * @brief 설정 페이로드
 * 
 * 로봇 설정 변경/조회를 위한 구조체입니다.
 */
typedef struct __attribute__((packed)) {
    uint8_t config_id;        ///< 설정 매개변수 ID
    float value;              ///< 설정 값
} config_payload_t;

/**
 * @brief 완전한 프로토콜 메시지
 * 
 * 헤더와 다양한 타입의 페이로드를 포함하는 완전한 메시지 구조체입니다.
 */
typedef struct __attribute__((packed)) {
    protocol_header_t header;  ///< 메시지 헤더
    union {
        move_command_payload_t move_cmd;     ///< 이동 명령 페이로드
        status_response_payload_t status_resp; ///< 상태 응답 페이로드
        config_payload_t config;             ///< 설정 페이로드
        uint8_t raw_data[MAX_PAYLOAD_SIZE];  ///< 원시 데이터 버퍼
    } payload;
} protocol_message_t;

/**
 * @brief 체크섬 계산
 * 
 * 주어진 데이터에 대한 CRC16 체크섬을 계산합니다.
 * 
 * @param data 체크섬을 계산할 데이터
 * @param length 데이터 길이
 * @return uint16_t CRC16 체크섬
 */
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);

/**
 * @brief 메시지 유효성 검증
 * 
 * 수신된 메시지의 체크섬과 구조적 유효성을 검증합니다.
 * 
 * @param msg 검증할 메시지
 * @return bool 유효성 여부
 * @retval true 유효한 메시지
 * @retval false 무효한 메시지
 */
bool validate_message(const protocol_message_t* msg);

/**
 * @brief 메시지 인코딩
 * 
 * 프로토콜 메시지를 바이너리 버퍼로 인코딩합니다.
 * 
 * @param msg 인코딩할 메시지
 * @param buffer 출력 버퍼
 * @param buffer_size 버퍼 크기
 * @return int 인코딩된 바이트 수 (음수: 오류)
 */
int encode_message(const protocol_message_t* msg, uint8_t* buffer, int buffer_size);

/**
 * @brief 메시지 디코딩
 * 
 * 바이너리 버퍼에서 프로토콜 메시지를 디코딩합니다.
 * 
 * @param buffer 입력 버퍼
 * @param buffer_len 버퍼 길이
 * @param msg 디코딩된 메시지 출력
 * @return int 처리된 바이트 수 (음수: 오류)
 */
int decode_message(const uint8_t* buffer, int buffer_len, protocol_message_t* msg);

/**
 * @brief 이동 명령 메시지 생성
 * 
 * 이동 명령을 위한 프로토콜 메시지를 생성합니다.
 * 
 * @param msg 출력 메시지
 * @param direction 이동 방향
 * @param turn 회전 값
 * @param speed 속도 값
 * @param flags 명령 플래그
 * @param seq_num 시퀀스 번호
 */
void build_move_command(protocol_message_t* msg, int8_t direction, int8_t turn, 
                       uint8_t speed, uint8_t flags, uint8_t seq_num);

/**
 * @brief 상태 응답 메시지 생성
 * 
 * 로봇 상태 응답을 위한 프로토콜 메시지를 생성합니다.
 * 
 * @param msg 출력 메시지
 * @param angle 로봇 각도
 * @param velocity 로봇 속도
 * @param state 로봇 상태
 * @param seq_num 시퀀스 번호
 */
void build_status_response(protocol_message_t* msg, float angle, float velocity,
                         uint8_t state, uint8_t seq_num);

/**
 * @brief 오류 메시지 생성
 * 
 * 오류 상황을 알리는 프로토콜 메시지를 생성합니다.
 * 
 * @param msg 출력 메시지
 * @param error_code 오류 코드
 * @param seq_num 시퀀스 번호
 */
void build_error_message(protocol_message_t* msg, uint8_t error_code, uint8_t seq_num);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H