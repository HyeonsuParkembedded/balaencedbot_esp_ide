/**
 * @file error_recovery.c
 * @brief 시스템 오류 복구 및 안전 관리 구현
 * 
 * 컴포넌트 초기화 재시도, 실패 처리, 안전 모드 전환 등의
 * 시스템 안정성 관련 기능을 구현합니다.
 * 
 * @author BalanceBot Team
 * @date 2024-12-19
 * @version 1.0
 */

#include "error_recovery.h"
#include "config.h"
#include <string.h>
#ifndef NATIVE_BUILD
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
// Native build - mock ESP functions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ESP_LOGI(tag, format, ...) printf("[INFO] " tag ": " format "\n", ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("[WARN] " tag ": " format "\n", ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) printf("[ERROR] " tag ": " format "\n", ##__VA_ARGS__)
#define esp_err_to_name(err) ((err == ESP_OK) ? "ESP_OK" : "ESP_FAIL")
#define vTaskDelay(ticks) // No-op for native
#define pdMS_TO_TICKS(ms) (ms)
#define esp_restart() exit(1)
#endif

static const char* TAG = "ERROR_RECOVERY";  ///< 로깅 태그

static component_info_t system_components[10];  ///< 시스템 컴포넌트 목록
static int component_count = 0;                 ///< 등록된 컴포넌트 수
static bool safe_mode_active = false;           ///< 안전 모드 활성화 플래그

/**
 * @brief 오류 복구 시스템 초기화 구현
 * 
 * 오류 복구 시스템을 초기화하고 모니터링을 시작합니다.
 * 
 * @return esp_err_t 초기화 결과
 */
esp_err_t error_recovery_init(void) {
    ESP_LOGI(TAG, "Error recovery system initialized");
    return ESP_OK;
}

/**
 * @brief 재시도 기능이 있는 컴포넌트 초기화 구현
 * 
 * 컴포넌트 초기화를 시도하고, 실패 시 설정된 횟수만큼 재시도합니다.
 * 성공 시 시스템 컴포넌트 목록에 추가합니다.
 * 
 * @param component 초기화할 컴포넌트 정보
 * @return bool 초기화 성공 여부
 */
bool initialize_component_with_retry(component_info_t* component) {
    // 입력 유효성 검사
    if (component == NULL || component->init_func == NULL) {
        ESP_LOGE(TAG, "Invalid component configuration");
        return false;
    }

    ESP_LOGI(TAG, "Initializing component: %s", component->name);
    
    // 설정된 횟수만큼 초기화 재시도
    for (int retry = 0; retry < CONFIG_MAX_INIT_RETRIES; retry++) {
        esp_err_t ret = component->init_func();
        if (ret == ESP_OK) {
            component->initialized = true;
            component->retry_count = retry;
            ESP_LOGI(TAG, "Component %s initialized successfully", component->name);
            
            // 시스템 컴포넌트 목록에 추가
            if (component_count < 10) {
                system_components[component_count++] = *component;
            }
            return true;
        }
        
        ESP_LOGW(TAG, "%s initialization failed (attempt %d/%d): %s", 
                component->name, retry + 1, CONFIG_MAX_INIT_RETRIES, esp_err_to_name(ret));
        
        if (retry < CONFIG_MAX_INIT_RETRIES - 1) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ERROR_RECOVERY_DELAY));
        }
    }
    
    // All retries failed
    component->initialized = false;
    component->retry_count = CONFIG_MAX_INIT_RETRIES;
    handle_component_failure(component);
    
    return false;
}

/**
 * @brief 컴포넌트 실패 처리 구현
 * 
 * 컴포넌트 실패 시 우선순위에 따른 적절한 대응을 수행합니다.
 * - 필수: 안전 모드 진입
 * - 중요: 제한된 기능으로 계속 운영
 * - 선택적: 경고만 출력하고 무시
 * 
 * @param component 실패한 컴포넌트 정보
 */
void handle_component_failure(component_info_t* component) {
    ESP_LOGE(TAG, "Component %s failed after %d retries", 
            component->name, component->retry_count);
    
    switch (component->priority) {
    case COMPONENT_CRITICAL:
        ESP_LOGE(TAG, "Critical component %s failed - entering safe mode", component->name);
        enter_safe_mode();
        break;
        
    case COMPONENT_IMPORTANT:
        ESP_LOGW(TAG, "Important component %s failed - continuing with limited functionality", 
                component->name);
        break;
        
    case COMPONENT_OPTIONAL:
        ESP_LOGI(TAG, "Optional component %s failed - continuing normally", component->name);
        break;
    }
}

/**
 * @brief 컴포넌트 작동 상태 확인 구현
 * 
 * 시스템 컴포넌트 목록에서 지정된 이름의 컴포넌트를 찾아
 * 초기화 상태를 확인합니다.
 * 
 * @param name 확인할 컴포넌트 이름
 * @return bool 작동 상태
 */
bool is_component_operational(const char* name) {
    for (int i = 0; i < component_count; i++) {
        if (strcmp(system_components[i].name, name) == 0) {
            return system_components[i].initialized;
        }
    }
    return false;  // 컴포넌트를 찾을 수 없으면 작동 중이 아님
}

/**
 * @brief 안전 모드 진입 구현
 * 
 * 시스템을 안전 모드로 전환하고 설정된 지연 시간 후 재시작합니다.
 * 재시작 전에 시스템 상태를 로그로 기록합니다.
 */
void enter_safe_mode(void) {
    safe_mode_active = true;
    ESP_LOGE(TAG, "ENTERING SAFE MODE - System will restart in %d seconds", 
            CONFIG_ERROR_RECOVERY_DELAY / 1000);
    
    // 재시작 전 시스템 상태 기록
    log_system_health();
    
    // 로그 출력을 위한 대기 시간
    vTaskDelay(pdMS_TO_TICKS(CONFIG_ERROR_RECOVERY_DELAY));
    
    ESP_LOGE(TAG, "Restarting system...");
    esp_restart();
}

/**
 * @brief 시스템 상태 로깅 구현
 * 
 * 현재 시스템의 전반적인 상태를 로그로 출력합니다.
 * 안전 모드 상태, 컴포넌트별 초기화 상태, 재시도 횟수 등을 포함합니다.
 */
void log_system_health(void) {
    ESP_LOGI(TAG, "=== SYSTEM HEALTH REPORT ===");
    ESP_LOGI(TAG, "Safe mode active: %s", safe_mode_active ? "YES" : "NO");
    ESP_LOGI(TAG, "Total components: %d", component_count);
    
    // 모든 컴포넌트 상태 출력
    for (int i = 0; i < component_count; i++) {
        ESP_LOGI(TAG, "Component %s: %s (retries: %d, priority: %d)", 
                system_components[i].name,
                system_components[i].initialized ? "OK" : "FAILED",
                system_components[i].retry_count,
                system_components[i].priority);
    }
    ESP_LOGI(TAG, "========================");
}