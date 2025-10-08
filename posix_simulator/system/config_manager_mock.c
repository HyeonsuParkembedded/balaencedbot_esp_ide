/**
 * @file config_manager_mock.c
 * @brief POSIX 시뮬레이터용 설정 관리자 목 구현
 * 
 * ESP32 NVS 없이도 설정 관리 기능을 시뮬레이션하기 위한 목 구현입니다.
 * 메모리 기반으로 동작하며 파일 저장/로드 기능을 제공합니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

// Mock ESP types
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2

// Mock 로깅
#define ESP_LOGI(tag, format, ...) printf("[INFO][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("[WARN][%s] " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) printf("[ERROR][%s] " format "\n", tag, ##__VA_ARGS__)

static const char* TAG = "CONFIG_MOCK";

// 파라미터 ID 열거형
typedef enum {
    CONFIG_PARAM_BALANCE_KP = 0,
    CONFIG_PARAM_BALANCE_KI,
    CONFIG_PARAM_BALANCE_KD,
    CONFIG_PARAM_VELOCITY_KP,
    CONFIG_PARAM_VELOCITY_KI,
    CONFIG_PARAM_VELOCITY_KD,
    CONFIG_PARAM_KALMAN_Q_ANGLE,
    CONFIG_PARAM_KALMAN_Q_BIAS,
    CONFIG_PARAM_KALMAN_R_MEASURE,
    CONFIG_PARAM_MAX_TILT_ANGLE,
    CONFIG_PARAM_FALLEN_THRESHOLD,
    CONFIG_PARAM_COUNT
} config_param_id_t;

// 튜닝 파라미터 구조체
typedef struct {
    float balance_kp;
    float balance_ki;
    float balance_kd;
    float velocity_kp;
    float velocity_ki;
    float velocity_kd;
    float kalman_q_angle;
    float kalman_q_bias;
    float kalman_r_measure;
    float max_tilt_angle;
    float fallen_threshold;
} tuning_params_t;

// 전역 상태
static bool initialized = false;
static tuning_params_t current_params;
static const char* config_file = "/tmp/balancebot_config.txt";

// 기본값 정의
static const tuning_params_t default_params = {
    .balance_kp = 25.0f,
    .balance_ki = 0.5f,
    .balance_kd = 0.8f,
    .velocity_kp = 1.2f,
    .velocity_ki = 0.02f,
    .velocity_kd = 0.0f,
    .kalman_q_angle = 0.001f,
    .kalman_q_bias = 0.003f,
    .kalman_r_measure = 0.03f,
    .max_tilt_angle = 45.0f,
    .fallen_threshold = 60.0f
};

// 파라미터 이름 배열
static const char* param_names[CONFIG_PARAM_COUNT] = {
    "balance_kp", "balance_ki", "balance_kd",
    "velocity_kp", "velocity_ki", "velocity_kd", 
    "kalman_q_angle", "kalman_q_bias", "kalman_r_measure",
    "max_tilt_angle", "fallen_threshold"
};

// 파라미터 값 포인터 배열
static float* get_param_ptr(config_param_id_t param_id) {
    switch (param_id) {
        case CONFIG_PARAM_BALANCE_KP: return &current_params.balance_kp;
        case CONFIG_PARAM_BALANCE_KI: return &current_params.balance_ki;
        case CONFIG_PARAM_BALANCE_KD: return &current_params.balance_kd;
        case CONFIG_PARAM_VELOCITY_KP: return &current_params.velocity_kp;
        case CONFIG_PARAM_VELOCITY_KI: return &current_params.velocity_ki;
        case CONFIG_PARAM_VELOCITY_KD: return &current_params.velocity_kd;
        case CONFIG_PARAM_KALMAN_Q_ANGLE: return &current_params.kalman_q_angle;
        case CONFIG_PARAM_KALMAN_Q_BIAS: return &current_params.kalman_q_bias;
        case CONFIG_PARAM_KALMAN_R_MEASURE: return &current_params.kalman_r_measure;
        case CONFIG_PARAM_MAX_TILT_ANGLE: return &current_params.max_tilt_angle;
        case CONFIG_PARAM_FALLEN_THRESHOLD: return &current_params.fallen_threshold;
        default: return NULL;
    }
}

// 파라미터 유효성 검사
static bool is_param_valid(config_param_id_t param_id, float value) {
    if (isnanf(value) || isinf(value)) return false;
    
    switch (param_id) {
        case CONFIG_PARAM_BALANCE_KP:
        case CONFIG_PARAM_BALANCE_KI:
        case CONFIG_PARAM_BALANCE_KD:
        case CONFIG_PARAM_VELOCITY_KP:
        case CONFIG_PARAM_VELOCITY_KI:
        case CONFIG_PARAM_VELOCITY_KD:
            return value >= 0.0f && value <= 1000.0f;
        case CONFIG_PARAM_KALMAN_Q_ANGLE:
        case CONFIG_PARAM_KALMAN_Q_BIAS:
        case CONFIG_PARAM_KALMAN_R_MEASURE:
            return value > 0.0f && value <= 1.0f;
        case CONFIG_PARAM_MAX_TILT_ANGLE:
        case CONFIG_PARAM_FALLEN_THRESHOLD:
            return value > 0.0f && value <= 90.0f;
        default:
            return false;
    }
}

// 파일에서 설정 로드
static esp_err_t load_config_from_file(void) {
    FILE* file = fopen(config_file, "r");
    if (!file) {
        ESP_LOGW(TAG, "Config file not found, using defaults");
        return ESP_FAIL;
    }
    
    char line[128];
    int loaded_count = 0;
    
    while (fgets(line, sizeof(line), file)) {
        char name[32];
        float value;
        if (sscanf(line, "%31s %f", name, &value) == 2) {
            for (int i = 0; i < CONFIG_PARAM_COUNT; i++) {
                if (strcmp(name, param_names[i]) == 0) {
                    if (is_param_valid(i, value)) {
                        *get_param_ptr(i) = value;
                        loaded_count++;
                    }
                    break;
                }
            }
        }
    }
    
    fclose(file);
    ESP_LOGI(TAG, "Loaded %d parameters from file", loaded_count);
    return ESP_OK;
}

// 파일에 설정 저장
static esp_err_t save_config_to_file(void) {
    FILE* file = fopen(config_file, "w");
    if (!file) {
        ESP_LOGE(TAG, "Failed to create config file");
        return ESP_FAIL;
    }
    
    for (int i = 0; i < CONFIG_PARAM_COUNT; i++) {
        fprintf(file, "%s %.6f\n", param_names[i], *get_param_ptr(i));
    }
    
    fclose(file);
    ESP_LOGI(TAG, "Config saved to file");
    return ESP_OK;
}

// 공개 함수 구현
esp_err_t config_manager_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    // 기본값으로 초기화
    current_params = default_params;
    
    // 파일에서 설정 로드 시도
    load_config_from_file();
    
    initialized = true;
    ESP_LOGI(TAG, "Config manager initialized (POSIX Mock)");
    return ESP_OK;
}

esp_err_t config_manager_set_param(config_param_id_t param_id, float value, bool save_to_file) {
    if (!initialized) return ESP_FAIL;
    if (param_id >= CONFIG_PARAM_COUNT) return ESP_ERR_INVALID_ARG;
    if (!is_param_valid(param_id, value)) return ESP_ERR_INVALID_ARG;
    
    *get_param_ptr(param_id) = value;
    
    if (save_to_file) {
        save_config_to_file();
    }
    
    ESP_LOGI(TAG, "Set %s = %.6f", param_names[param_id], value);
    return ESP_OK;
}

float config_manager_get_param(config_param_id_t param_id) {
    if (!initialized || param_id >= CONFIG_PARAM_COUNT) {
        return 0.0f;
    }
    return *get_param_ptr(param_id);
}

tuning_params_t* config_manager_get_tuning_params(void) {
    return initialized ? &current_params : NULL;
}

esp_err_t config_manager_save_all_params(void) {
    if (!initialized) return ESP_FAIL;
    return save_config_to_file();
}

esp_err_t config_manager_reset_defaults(bool save_to_file) {
    if (!initialized) return ESP_FAIL;
    
    current_params = default_params;
    
    if (save_to_file) {
        save_config_to_file();
    }
    
    ESP_LOGI(TAG, "Reset to default parameters");
    return ESP_OK;
}

const char* config_manager_get_param_name(config_param_id_t param_id) {
    if (param_id >= CONFIG_PARAM_COUNT) return NULL;
    return param_names[param_id];
}

esp_err_t config_manager_handle_ble_command(const char* command) {
    if (!initialized || !command) {
        return ESP_ERR_INVALID_ARG;
    }
    
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
                ESP_LOGI(TAG, "%s = %.3f", param_names[param_id], value);
                return ESP_OK;
            }
        }
    } else if (strcmp(cmd, "RESET") == 0) {
        return config_manager_reset_defaults(true);
    } else if (strcmp(cmd, "SAVE") == 0) {
        return config_manager_save_all_params();
    }
    
    ESP_LOGE(TAG, "Unknown command: %s", command);
    return ESP_FAIL;
}