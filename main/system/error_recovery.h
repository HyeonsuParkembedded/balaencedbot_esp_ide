/**
 * @file error_recovery.h
 * @brief 시스템 오류 복구 및 안전 관리 인터페이스
 * 
 * 시스템 컴포넌트의 초기화 실패나 런타임 오류를 처리하고,
 * 안전한 운영을 위한 복구 메커니즘을 제공합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#ifndef ERROR_RECOVERY_H
#define ERROR_RECOVERY_H

#ifndef NATIVE_BUILD
#include "esp_err.h"
#else
// Native build - define ESP types
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 컴포넌트 우선순위 열거형
 * 
 * 시스템 컴포넌트의 중요도에 따른 분류입니다.
 * 우선순위에 따라 실패 시 대응 방식이 달라집니다.
 */
typedef enum {
    COMPONENT_CRITICAL,    ///< 필수 컴포넌트 (실패 시 시스템 중단)
    COMPONENT_IMPORTANT,   ///< 중요 컴포넌트 (제한된 기능으로 계속 운영)
    COMPONENT_OPTIONAL     ///< 선택적 컴포넌트 (실패해도 무시 가능)
} component_priority_t;

/**
 * @brief 컴포넌트 초기화 함수 포인터 타입
 * 
 * 각 컴포넌트의 초기화 함수 시그니처입니다.
 */
typedef esp_err_t (*component_init_func_t)(void);

/**
 * @brief 컴포넌트 정보 구조체
 * 
 * 시스템 컴포넌트의 상태와 설정을 관리합니다.
 */
typedef struct {
    const char* name;                 ///< 컴포넌트 이름
    component_init_func_t init_func;  ///< 초기화 함수 포인터
    component_priority_t priority;    ///< 컴포넌트 우선순위
    bool initialized;                 ///< 초기화 완료 플래그
    int retry_count;                  ///< 재시도 횟수
} component_info_t;

/**
 * @brief 오류 복구 시스템 초기화
 * 
 * 오류 복구 메커니즘을 초기화하고 시스템 모니터링을 시작합니다.
 * 
 * @return esp_err_t 초기화 결과
 * @retval ESP_OK 성공
 * @retval ESP_FAIL 실패
 */
esp_err_t error_recovery_init(void);

/**
 * @brief 재시도 기능이 있는 컴포넌트 초기화
 * 
 * 지정된 컴포넌트를 초기화하며, 실패 시 재시도를 수행합니다.
 * 컴포넌트 우선순위에 따라 재시도 횟수가 달라집니다.
 * 
 * @param component 초기화할 컴포넌트 정보
 * @return bool 초기화 성공 여부
 * @retval true 초기화 성공
 * @retval false 초기화 실패
 */
bool initialize_component_with_retry(component_info_t* component);

/**
 * @brief 컴포넌트 실패 처리
 * 
 * 컴포넌트 실패 시 우선순위에 따른 적절한 대응을 수행합니다.
 * 필수 컴포넌트 실패 시 안전 모드로 전환할 수 있습니다.
 * 
 * @param component 실패한 컴포넌트 정보
 */
void handle_component_failure(component_info_t* component);

/**
 * @brief 컴포넌트 작동 상태 확인
 * 
 * 지정된 이름의 컴포넌트가 정상적으로 작동 중인지 확인합니다.
 * 
 * @param name 확인할 컴포넌트 이름
 * @return bool 작동 상태
 * @retval true 정상 작동
 * @retval false 작동 중단 또는 미초기화
 */
bool is_component_operational(const char* name);

/**
 * @brief 안전 모드 진입
 * 
 * 시스템을 안전 모드로 전환합니다.
 * 모든 액추에이터를 정지하고 최소한의 기능만 유지합니다.
 */
void enter_safe_mode(void);

/**
 * @brief 시스템 상태 로깅
 * 
 * 현재 시스템의 전반적인 상태를 로그로 출력합니다.
 * 모든 컴포넌트의 상태와 오류 정보를 포함합니다.
 */
void log_system_health(void);

#ifdef __cplusplus
}
#endif

#endif // ERROR_RECOVERY_H