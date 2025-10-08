/**
 * @file config_manager.c
 * @brief 설정 값 관리 시스템 구현 파일
 * 
 * NVS를 사용하여 PID 게인, 센서 노이즈 파라미터 등을
 * 저장하고 BLE를 통해 실시간으로 튜닝할 수 있는 기능을 구현합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-08
 * @version 1.0
 */

#include "config_manager.h"
#include "../bsw/system_services.h"
#include "../config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "CONFIG_MANAGER";
static const char* NVS_NAMESPACE = "balance_bot";

// 현재 튜닝 파라미터 (메모리)
static tuning_params_t current_params;
static bool initialized = false;

// 파라미터 이름 배열 (디버깅 및 BLE 명령용)
static const char* param_names[CONFIG_PARAM_COUNT] = {
    "Balance_Kp",
    "Balance_Ki", 
    "Balance_Kd",
    "Velocity_Kp",
    "Velocity_Ki",
    "Velocity_Kd",
    "Kalman_Q_Angle",
    "Kalman_Q_Bias",
    "Kalman_R_Measure",
    "Max_Tilt_Angle",
    "Fallen_Threshold"
};

// NVS 키 배열
static const char* nvs_keys[CONFIG_PARAM_COUNT] = {
    CONFIG_KEY_BALANCE_KP,
    CONFIG_KEY_BALANCE_KI,
    CONFIG_KEY_BALANCE_KD,
    CONFIG_KEY_VELOCITY_KP,
    CONFIG_KEY_VELOCITY_KI,
    CONFIG_KEY_VELOCITY_KD,
    CONFIG_KEY_KALMAN_Q_ANGLE,
    CONFIG_KEY_KALMAN_Q_BIAS,
    CONFIG_KEY_KALMAN_R_MEASURE,
    CONFIG_KEY_MAX_TILT_ANGLE,
    CONFIG_KEY_FALLEN_THRESHOLD
};

/**
 * @brief 기본값으로 파라미터 설정
 */
static void set_default_params(void) {
    // config.h의 기본값 사용
    current_params.balance_kp = CONFIG_BALANCE_PID_KP;
    current_params.balance_ki = CONFIG_BALANCE_PID_KI;
    current_params.balance_kd = CONFIG_BALANCE_PID_KD;
    current_params.velocity_kp = 1.0f;
    current_params.velocity_ki = 0.1f;
    current_params.velocity_kd = 0.0f;
    current_params.kalman_q_angle = CONFIG_KALMAN_Q_ANGLE;
    current_params.kalman_q_bias = CONFIG_KALMAN_Q_BIAS;
    current_params.kalman_r_measure = CONFIG_KALMAN_R_MEASURE;
    current_params.max_tilt_angle = CONFIG_FALLEN_ANGLE_THRESHOLD;
    current_params.fallen_threshold = CONFIG_FALLEN_ANGLE_THRESHOLD;
}

/**
 * @brief 파라미터 ID로부터 값 포인터 가져오기
 */
static float* get_param_ptr(config_param_id_t param_id) {
    switch (param_id) {
        case CONFIG_PARAM_BALANCE_KP:       return &current_params.balance_kp;
        case CONFIG_PARAM_BALANCE_KI:       return &current_params.balance_ki;
        case CONFIG_PARAM_BALANCE_KD:       return &current_params.balance_kd;
        case CONFIG_PARAM_VELOCITY_KP:      return &current_params.velocity_kp;
        case CONFIG_PARAM_VELOCITY_KI:      return &current_params.velocity_ki;
        case CONFIG_PARAM_VELOCITY_KD:      return &current_params.velocity_kd;
        case CONFIG_PARAM_KALMAN_Q_ANGLE:   return &current_params.kalman_q_angle;
        case CONFIG_PARAM_KALMAN_Q_BIAS:    return &current_params.kalman_q_bias;
        case CONFIG_PARAM_KALMAN_R_MEASURE: return &current_params.kalman_r_measure;
        case CONFIG_PARAM_MAX_TILT_ANGLE:   return &current_params.max_tilt_angle;
        case CONFIG_PARAM_FALLEN_THRESHOLD: return &current_params.fallen_threshold;
        default: return NULL;
    }
}

/**
 * @brief 설정 관리자 초기화
 */
esp_err_t config_manager_init(void) {
    esp_err_t ret;
    
    // NVS 초기화
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS 파티션이 꽉 찼거나 새 버전이면 지우고 다시 초기화
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 기본값 설정
    set_default_params();
    
    // NVS에서 저장된 값 로드
    ret = config_manager_load_all();
    if (ret != ESP_OK) {
        BSW_LOGW(TAG, "Failed to load config from NVS, using defaults");
        // 기본값 사용하고 NVS에 저장
        config_manager_save_all();
    }
    
    initialized = true;
    BSW_LOGI(TAG, "Config manager initialized");
    
    return ESP_OK;
}

/**
 * @brief 현재 튜닝 파라미터 가져오기
 */
const tuning_params_t* config_manager_get_params(void) {
    if (!initialized) {
        BSW_LOGE(TAG, "Config manager not initialized");
        return NULL;
    }
    return &current_params;
}

/**
 * @brief 특정 파라미터 값 설정
 */
esp_err_t config_manager_set_param(config_param_id_t param_id, float value, bool save_to_nvs) {
    if (!initialized) {
        BSW_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (param_id >= CONFIG_PARAM_COUNT) {
        BSW_LOGE(TAG, "Invalid parameter ID: %d", param_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    float* param_ptr = get_param_ptr(param_id);
    if (!param_ptr) {
        BSW_LOGE(TAG, "Failed to get parameter pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    *param_ptr = value;
    
    BSW_LOGI(TAG, "Set %s = %.3f", param_names[param_id], value);
    
    if (save_to_nvs) {
        nvs_handle_t nvs_handle;
        esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if (ret == ESP_OK) {
            ret = nvs_set_blob(nvs_handle, nvs_keys[param_id], &value, sizeof(float));
            if (ret == ESP_OK) {
                ret = nvs_commit(nvs_handle);
            }
            nvs_close(nvs_handle);
            
            if (ret != ESP_OK) {
                BSW_LOGE(TAG, "Failed to save %s to NVS: %s", param_names[param_id], esp_err_to_name(ret));
            }
        }
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief 특정 파라미터 값 가져오기
 */
float config_manager_get_param(config_param_id_t param_id) {
    if (!initialized || param_id >= CONFIG_PARAM_COUNT) {
        return 0.0f;
    }
    
    float* param_ptr = get_param_ptr(param_id);
    return param_ptr ? *param_ptr : 0.0f;
}

/**
 * @brief 모든 설정을 NVS에 저장
 */
esp_err_t config_manager_save_all(void) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 모든 파라미터를 NVS에 저장
    for (int i = 0; i < CONFIG_PARAM_COUNT; i++) {
        float* param_ptr = get_param_ptr((config_param_id_t)i);
        if (param_ptr) {
            ret = nvs_set_blob(nvs_handle, nvs_keys[i], param_ptr, sizeof(float));
            if (ret != ESP_OK) {
                BSW_LOGE(TAG, "Failed to save %s: %s", param_names[i], esp_err_to_name(ret));
                break;
            }
        }
    }
    
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }
    
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        BSW_LOGI(TAG, "All parameters saved to NVS");
    }
    
    return ret;
}

/**
 * @brief NVS에서 모든 설정 로드
 */
esp_err_t config_manager_load_all(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 모든 파라미터를 NVS에서 로드
    for (int i = 0; i < CONFIG_PARAM_COUNT; i++) {
        float* param_ptr = get_param_ptr((config_param_id_t)i);
        if (param_ptr) {
            size_t required_size = sizeof(float);
            ret = nvs_get_blob(nvs_handle, nvs_keys[i], param_ptr, &required_size);
            if (ret != ESP_OK) {
                BSW_LOGW(TAG, "Failed to load %s, using default", param_names[i]);
            }
        }
    }
    
    nvs_close(nvs_handle);
    BSW_LOGI(TAG, "Parameters loaded from NVS");
    
    return ESP_OK;
}

/**
 * @brief 기본값으로 초기화
 */
esp_err_t config_manager_reset_defaults(bool save_to_nvs) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    set_default_params();
    BSW_LOGI(TAG, "Parameters reset to defaults");
    
    if (save_to_nvs) {
        return config_manager_save_all();
    }
    
    return ESP_OK;
}

/**
 * @brief 파라미터 이름 가져오기
 */
const char* config_manager_get_param_name(config_param_id_t param_id) {
    if (param_id >= CONFIG_PARAM_COUNT) {
        return "Unknown";
    }
    return param_names[param_id];
}

/**
 * @brief BLE 명령으로 파라미터 튜닝
 */
esp_err_t config_manager_handle_ble_command(const char* command) {
    if (!initialized || !command) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 명령 형식: "SET <param_id> <value>" 또는 "GET <param_id>" 또는 "RESET" 또는 "SAVE"
    char cmd[16];
    if (sscanf(command, "%15s", cmd) != 1) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strcmp(cmd, "SET") == 0) {
        int param_id;
        float value;
        if (sscanf(command, "SET %d %f", &param_id, &value) == 2) {
            if (param_id >= 0 && param_id < CONFIG_PARAM_COUNT) {
                return config_manager_set_param((config_param_id_t)param_id, value, true);
            }
        }
    } else if (strcmp(cmd, "GET") == 0) {
        int param_id;
        if (sscanf(command, "GET %d", &param_id) == 1) {
            if (param_id >= 0 && param_id < CONFIG_PARAM_COUNT) {
                float value = config_manager_get_param((config_param_id_t)param_id);
                BSW_LOGI(TAG, "%s = %.3f", param_names[param_id], value);
                return ESP_OK;
            }
        }
    } else if (strcmp(cmd, "RESET") == 0) {
        return config_manager_reset_defaults(true);
    } else if (strcmp(cmd, "SAVE") == 0) {
        return config_manager_save_all();
    }
    
    return ESP_ERR_INVALID_ARG;
}

/**
 * @brief 현재 파라미터 상태 문자열 생성
 */
esp_err_t config_manager_get_status_string(char* buffer, size_t buffer_size) {
    if (!initialized || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int len = snprintf(buffer, buffer_size,
        "PID:Bal[%.2f,%.3f,%.2f] Vel[%.2f,%.3f,%.2f] "
        "Kalman:[%.3f,%.3f,%.3f] Tilt:%.1f Fall:%.1f",
        current_params.balance_kp, current_params.balance_ki, current_params.balance_kd,
        current_params.velocity_kp, current_params.velocity_ki, current_params.velocity_kd,
        current_params.kalman_q_angle, current_params.kalman_q_bias, current_params.kalman_r_measure,
        current_params.max_tilt_angle, current_params.fallen_threshold
    );
    
    return (len > 0 && len < buffer_size) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}