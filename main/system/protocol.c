/**
 * @file protocol.c
 * @brief 통신 프로토콜 구현
 * 
 * 바이너리 프로토콜의 메시지 인코딩, 디코딩, 검증 및
 * 체크섬 계산 기능을 구현합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#include "protocol.h"
#include <string.h>

#define PROTOCOL_START_MARKER 0xAA  ///< 프로토콜 시작 마커

/**
 * @brief CRC16 체크섬 계산 구현
 * 
 * 데이터 무결성 확인을 위한 CRC16 체크섬을 계산합니다.
 * 표준 CRC16-ANSI 알고리즘을 사용합니다.
 * 
 * @param data 체크섬을 계산할 데이터
 * @param length 데이터 길이
 * @return uint16_t CRC16 체크섬
 */
uint16_t calculate_checksum(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // CRC 초기값
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // CRC16 다항식
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief 메시지 유효성 검증 구현
 * 
 * 수신된 메시지의 구조적 유효성과 체크섬을 검증합니다.
 * 시작 마커, 버전, 길이, 체크섬을 모두 확인합니다.
 * 
 * @param msg 검증할 메시지
 * @return bool 유효성 여부
 */
bool validate_message(const protocol_message_t* msg) {
    if (msg == NULL) return false;
    
    // 시작 마커 확인
    if (msg->header.start_marker != PROTOCOL_START_MARKER) return false;
    
    // 프로토콜 버전 확인
    if (msg->header.version != PROTOCOL_VERSION) return false;
    
    // 페이로드 길이 확인
    if (msg->header.payload_len > MAX_PAYLOAD_SIZE) return false;
    
    // 체크섬 계산 및 검증
    uint16_t calc_checksum = calculate_checksum((const uint8_t*)&msg->header + 6, 
                                              sizeof(protocol_header_t) - 6 + msg->header.payload_len);
    
    return (calc_checksum == msg->header.checksum);
}

/**
 * @brief 메시지 인코딩 구현
 * 
 * 프로토콜 메시지를 바이너리 버퍼로 인코딩합니다.
 * 헤더와 페이로드를 연속된 바이트 스트림으로 변환합니다.
 * 
 * @param msg 인코딩할 메시지
 * @param buffer 출력 버퍼
 * @param buffer_size 버퍼 크기
 * @return int 인코딩된 바이트 수 (음수: 오류)
 */
int encode_message(const protocol_message_t* msg, uint8_t* buffer, int buffer_size) {
    if (msg == NULL || buffer == NULL) return -1;
    
    int total_size = sizeof(protocol_header_t) + msg->header.payload_len;
    if (buffer_size < total_size) return -1;
    
    // 헤더와 페이로드 복사
    memcpy(buffer, &msg->header, sizeof(protocol_header_t));
    if (msg->header.payload_len > 0) {
        memcpy(buffer + sizeof(protocol_header_t), &msg->payload, msg->header.payload_len);
    }
    
    return total_size;
}

/**
 * @brief 바이너리 버퍼에서 메시지 디코딩
 * 
 * 수신된 바이너리 데이터를 프로토콜 메시지 구조체로 디코딩합니다.
 * 헤더 유효성, 페이로드 길이, 체크섬을 모두 검증합니다.
 * 
 * 디코딩 과정:
 * 1. 헤더 복사 및 기본 유효성 확인
 * 2. 시작 마커와 페이로드 길이 검증
 * 3. 페이로드 복사 (존재하는 경우)
 * 4. 전체 메시지 유효성 및 체크섬 검증
 * 
 * @param buffer 디코딩할 바이너리 데이터
 * @param buffer_len 버퍼 길이
 * @param msg 출력 메시지 구조체
 * @return 디코딩된 바이트 수 (음수: 오류)
 */
int decode_message(const uint8_t* buffer, int buffer_len, protocol_message_t* msg) {
    if (buffer == NULL || msg == NULL) return -1;
    if (buffer_len < sizeof(protocol_header_t)) return -1;
    
    // Copy header
    memcpy(&msg->header, buffer, sizeof(protocol_header_t));
    
    // Validate basic header
    if (msg->header.start_marker != PROTOCOL_START_MARKER) return -1;
    if (msg->header.payload_len > MAX_PAYLOAD_SIZE) return -1;
    
    int total_size = sizeof(protocol_header_t) + msg->header.payload_len;
    if (buffer_len < total_size) return -1;
    
    // Copy payload if present
    if (msg->header.payload_len > 0) {
        memcpy(&msg->payload, buffer + sizeof(protocol_header_t), msg->header.payload_len);
    }
    
    // Validate complete message
    if (!validate_message(msg)) return -1;
    
    return total_size;
}

/**
 * @brief 이동 명령 메시지 생성
 * 
 * 로봇의 이동 제어를 위한 명령 메시지를 생성합니다.
 * 방향, 회전, 속도, 플래그 등의 제어 파라미터를 포함합니다.
 * 
 * 메시지 구성:
 * - 헤더: 시작 마커, 버전, 타입(MOVE_CMD), 시퀀스 번호
 * - 페이로드: 방향, 회전, 속도, 플래그, 타임스탬프
 * - 체크섬: 헤더와 페이로드의 CRC16 체크섬
 * 
 * @param msg 출력 메시지 구조체
 * @param direction 이동 방향 (-100~100, 음수: 후진, 양수: 전진)
 * @param turn 회전 방향 (-100~100, 음수: 좌회전, 양수: 우회전)
 * @param speed 이동 속도 (0~100)
 * @param flags 제어 플래그 (비트마스크)
 * @param seq_num 메시지 시퀀스 번호
 */
void build_move_command(protocol_message_t* msg, int8_t direction, int8_t turn, 
                       uint8_t speed, uint8_t flags, uint8_t seq_num) {
    if (msg == NULL) return;
    
    // Build header
    msg->header.start_marker = PROTOCOL_START_MARKER;
    msg->header.version = PROTOCOL_VERSION;
    msg->header.msg_type = MSG_TYPE_MOVE_CMD;
    msg->header.seq_num = seq_num;
    msg->header.payload_len = sizeof(move_command_payload_t);
    
    // Build payload
    msg->payload.move_cmd.direction = direction;
    msg->payload.move_cmd.turn = turn;
    msg->payload.move_cmd.speed = speed;
    msg->payload.move_cmd.flags = flags;
    msg->payload.move_cmd.timestamp = 0; // Would be filled with actual timestamp
    
    // Calculate checksum
    msg->header.checksum = calculate_checksum((const uint8_t*)&msg->header + 6,
                                            sizeof(protocol_header_t) - 6 + msg->header.payload_len);
}

/**
 * @brief 상태 응답 메시지 생성
 * 
 * 로봇의 현재 상태 정보를 포함하는 응답 메시지를 생성합니다.
 * 각도, 속도, 로봇 상태, GPS 정보, 배터리 등의 정보를 포함합니다.
 * 
 * 상태 정보 구성:
 * - 자세 정보: 기울어짐 각도, 이동 속도
 * - 로봇 상태: 현재 동작 모드
 * - GPS 정보: 위치 좌표 (사용 가능한 경우)
 * - 시스템 정보: 배터리 레벨, 오류 플래그
 * 
 * @param msg 출력 메시지 구조체
 * @param angle 현재 기울어짐 각도 (도 단위)
 * @param velocity 현재 이동 속도 (cm/s 단위)
 * @param state 로봇 상태 코드
 * @param seq_num 메시지 시퀀스 번호
 */
void build_status_response(protocol_message_t* msg, float angle, float velocity,
                         uint8_t state, uint8_t seq_num) {
    if (msg == NULL) return;
    
    // Build header
    msg->header.start_marker = PROTOCOL_START_MARKER;
    msg->header.version = PROTOCOL_VERSION;
    msg->header.msg_type = MSG_TYPE_STATUS_RESP;
    msg->header.seq_num = seq_num;
    msg->header.payload_len = sizeof(status_response_payload_t);
    
    // Build payload
    msg->payload.status_resp.angle = angle;
    msg->payload.status_resp.velocity = velocity;
    msg->payload.status_resp.robot_state = state;
    msg->payload.status_resp.gps_status = 0;
    msg->payload.status_resp.latitude = 0.0f;
    msg->payload.status_resp.longitude = 0.0f;
    msg->payload.status_resp.battery_level = 100;
    msg->payload.status_resp.error_flags = 0;
    
    // Calculate checksum
    msg->header.checksum = calculate_checksum((const uint8_t*)&msg->header + 6,
                                            sizeof(protocol_header_t) - 6 + msg->header.payload_len);
}

/**
 * @brief 오류 메시지 생성
 * 
 * 시스템 오류나 명령 처리 실패를 알리는 오류 메시지를 생성합니다.
 * 간단한 오류 코드를 페이로드로 포함합니다.
 * 
 * 오류 메시지 용도:
 * - 잘못된 명령 수신
 * - 시스템 오류 발생
 * - 하드웨어 장애 알림
 * - 프로토콜 오류 응답
 * 
 * @param msg 출력 메시지 구조체
 * @param error_code 오류 코드 (시스템 정의)
 * @param seq_num 메시지 시퀀스 번호
 */
void build_error_message(protocol_message_t* msg, uint8_t error_code, uint8_t seq_num) {
    if (msg == NULL) return;
    
    // Build header
    msg->header.start_marker = PROTOCOL_START_MARKER;
    msg->header.version = PROTOCOL_VERSION;
    msg->header.msg_type = MSG_TYPE_ERROR;
    msg->header.seq_num = seq_num;
    msg->header.payload_len = 1;
    
    // Build payload
    msg->payload.raw_data[0] = error_code;
    
    // Calculate checksum
    msg->header.checksum = calculate_checksum((const uint8_t*)&msg->header + 6,
                                            sizeof(protocol_header_t) - 6 + msg->header.payload_len);
}