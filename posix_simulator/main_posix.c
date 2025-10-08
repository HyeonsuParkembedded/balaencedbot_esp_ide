/**
 * @file main_posix.c
 * @brief BalanceBot POSIX Simulator Main Application
 * 
 * FreeRTOS POSIX 포트를 사용한 BalanceBot 시뮬레이터입니다.
 * 실제 하드웨어 없이도 균형 제어 로직을 테스트할 수 있습니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

// Mock layer
#include "mock_esp.h"
#include "mock_bsw.h"

// BalanceBot logic (platform independent)
#include "logic/pid_controller.h"
#include "logic/kalman_filter.h"

// BalanceBot systems (mocked for POSIX)
extern int config_manager_init(void);
extern int config_manager_handle_ble_command(const char* command);
extern float config_manager_get_param(int param_id);
extern void* config_manager_get_tuning_params(void);

// BLE Controller Mock
extern int ble_controller_init(void* ble, const char* device_name);
extern void ble_controller_update(void* ble);
extern void ble_controller_get_command(const void* ble, void* cmd);
extern bool ble_controller_is_connected(const void* ble);
extern int ble_controller_send_status(void* ble, float angle, float velocity, float battery_voltage);
extern bool ble_controller_has_text_command(const void* ble);
extern const char* ble_controller_get_text_command(void* ble);

// 더미 BLE 컨트롤러 타입
typedef struct {
    bool device_connected;
    struct {
        int direction;
        int turn; 
        int speed;
        bool balance;
        bool standup;
    } current_command;
    char last_command[64];
    bool has_text_command;
} mock_ble_controller_t;

static const char* TAG = "POSIX_SIM";

// ============================================================================
// Keyboard Input Handling for Simulation
// ============================================================================

// 키보드 입력 상태
typedef struct {
    bool forward;     // 위쪽 화살표 키
    bool backward;    // 아래쪽 화살표 키
    bool left;        // 왼쪽 화살표 키
    bool right;       // 오른쪽 화살표 키
    bool balance_toggle; // 스페이스바 (밸런싱 토글)
    bool quit;        // 'q' 키 (종료)
} keyboard_state_t;

static keyboard_state_t keyboard_state = {0};
static struct termios old_termios;

// 키보드를 비차단 모드로 설정
static void setup_keyboard(void) {
    struct termios new_termios;
    
    // 현재 터미널 설정 저장
    tcgetattr(STDIN_FILENO, &old_termios);
    
    // 새 터미널 설정
    new_termios = old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO); // 캐노니컬 모드와 에코 비활성화
    new_termios.c_cc[VMIN] = 0;  // 최소 문자 수
    new_termios.c_cc[VTIME] = 0; // 타임아웃
    
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    
    // 비차단 모드 설정
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

// 키보드 설정 복원
static void restore_keyboard(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
}

// 키보드 입력 처리
static void process_keyboard_input(void) {
    char buf[10];
    ssize_t bytes_read = read(STDIN_FILENO, buf, sizeof(buf) - 1);
    
    if (bytes_read > 0) {
        buf[bytes_read] = '\0';
        
        // ESC 시퀀스 처리 (화살표 키)
        if (bytes_read >= 3 && buf[0] == 0x1B && buf[1] == '[') {
            switch (buf[2]) {
            case 'A': // 위쪽 화살표
                keyboard_state.forward = true;
                keyboard_state.backward = false;
                printf("[키보드] 앞으로 이동\n");
                break;
            case 'B': // 아래쪽 화살표
                keyboard_state.backward = true;
                keyboard_state.forward = false;
                printf("[키보드] 뒤로 이동\n");
                break;
            case 'C': // 오른쪽 화살표
                keyboard_state.right = true;
                keyboard_state.left = false;
                printf("[키보드] 오른쪽 회전\n");
                break;
            case 'D': // 왼쪽 화살표
                keyboard_state.left = true;
                keyboard_state.right = false;
                printf("[키보드] 왼쪽 회전\n");
                break;
            }
        }
        // 일반 키 처리
        else if (bytes_read == 1) {
            switch (buf[0]) {
            case ' ': // 스페이스바
                keyboard_state.balance_toggle = !keyboard_state.balance_toggle;
                printf("[키보드] 밸런싱 %s\n", keyboard_state.balance_toggle ? "활성화" : "비활성화");
                break;
            case 'q':
            case 'Q':
                keyboard_state.quit = true;
                printf("[키보드] 종료 요청\n");
                break;
            case 's':
            case 'S': // 정지
                keyboard_state.forward = false;
                keyboard_state.backward = false;
                keyboard_state.left = false;
                keyboard_state.right = false;
                printf("[키보드] 정지\n");
                break;
            }
        }
    }
}

// ============================================================================
// Global State and Configuration
// ============================================================================

// Robot state machine
typedef enum {
    ROBOT_STATE_INIT,
    ROBOT_STATE_IDLE, 
    ROBOT_STATE_BALANCING,
    ROBOT_STATE_STANDING_UP,
    ROBOT_STATE_FALLEN,
    ROBOT_STATE_ERROR
} robot_state_t;

static robot_state_t current_state = ROBOT_STATE_INIT;
static SemaphoreHandle_t state_mutex = NULL;
static SemaphoreHandle_t data_mutex = NULL;

// Control system components
static balance_pid_t balance_pid;
static kalman_filter_t kalman_filter;

// BLE Controller Mock
static mock_ble_controller_t ble_ctrl;

// Shared sensor data
static float filtered_angle = 0.0f;
static float robot_velocity = 0.0f;
static bool balancing_enabled __attribute__((unused)) = true;

// Task handles  
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t balance_task_handle = NULL;
static TaskHandle_t ble_task_handle = NULL;
static TaskHandle_t status_task_handle = NULL;
static TaskHandle_t keyboard_task_handle = NULL;

// ESP32-C6 Simulation parameters
#define ESP32C6_CPU_FREQ_MHZ    160     // RISC-V 160MHz 프로세서
#define ESP32C6_SRAM_SIZE_KB    512     // 512KB SRAM
#define ESP32C6_PERFORMANCE_FACTOR 1.6f // ESP32 대비 성능 향상 계수
#define SIMULATION_DT           0.005f  // 200Hz simulation rate (ESP32-C6 고성능)
#define MAX_TILT_ANGLE         45.0f    // Maximum tilt before "fallen" state

// ============================================================================
// Helper Functions
// ============================================================================

// 현재 원격 명령 가져오기
static void get_current_remote_command(float* target_velocity, float* target_turn, bool* balance_enabled, bool* standup_cmd) {
    // 키보드 입력 처리
    process_keyboard_input();
    
    if (ble_controller_is_connected(&ble_ctrl)) {
        *target_velocity = ble_ctrl.current_command.direction * ble_ctrl.current_command.speed * 0.01f; // 0-1 범위로 변환
        *target_turn = ble_ctrl.current_command.turn * 0.01f; // -1 to 1 범위로 변환
        *balance_enabled = ble_ctrl.current_command.balance;
        *standup_cmd = ble_ctrl.current_command.standup;
    } else {
        // 키보드 입력을 기반으로 명령 생성
        *target_velocity = 0.0f;
        *target_turn = 0.0f;
        
        if (keyboard_state.forward) {
            *target_velocity = 1.0f; // 최대 속도로 앞으로
        } else if (keyboard_state.backward) {
            *target_velocity = -1.0f; // 최대 속도로 뒤로
        }
        
        if (keyboard_state.left) {
            *target_turn = -0.5f; // 왼쪽 회전
        } else if (keyboard_state.right) {
            *target_turn = 0.5f; // 오른쪽 회전
        }
        
        *balance_enabled = keyboard_state.balance_toggle; // 스페이스바로 토글
        *standup_cmd = false;
        
        // 종료 요청 처리
        if (keyboard_state.quit) {
            printf("[키보드] 사용자 종료 요청\n");
            restore_keyboard();
            exit(0);
        }
    }
}

// 텍스트 명령 처리
static void handle_text_commands(void) {
    if (ble_controller_has_text_command(&ble_ctrl)) {
        const char* text_cmd = ble_controller_get_text_command(&ble_ctrl);
        if (text_cmd != NULL) {
            int ret = config_manager_handle_ble_command(text_cmd);
            if (ret == 0) { // ESP_OK equivalent
                printf("[MAIN] 텍스트 명령 처리 완료: %s\n", text_cmd);
                
                // 업데이트된 파라미터를 PID 컨트롤러에 적용
                void* params = config_manager_get_tuning_params();
                if (params) {
                    // 파라미터 업데이트 (간단화된 버전)
                    float new_kp = config_manager_get_param(0); // BALANCE_KP
                    float new_ki = config_manager_get_param(1); // BALANCE_KI  
                    float new_kd = config_manager_get_param(2); // BALANCE_KD
                    
                    balance_pid_set_balance_tunings(&balance_pid, new_kp, new_ki, new_kd);
                    printf("[MAIN] PID 파라미터 업데이트: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", new_kp, new_ki, new_kd);
                }
            } else {
                printf("[MAIN] 텍스트 명령 처리 실패: %s\n", text_cmd);
            }
        }
    }
}

// ============================================================================
// Thread-Safe Data Access Functions
// ============================================================================
static float get_filtered_angle(void) {
    float angle = 0.0f;
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        angle = filtered_angle;
        xSemaphoreGive(data_mutex);
    }
    return angle;
}

static void set_filtered_angle(float angle) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        filtered_angle = angle;
        xSemaphoreGive(data_mutex);
    }
}

static robot_state_t get_robot_state(void) {
    robot_state_t state = ROBOT_STATE_ERROR;
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_state;
        xSemaphoreGive(state_mutex);
    }
    return state;
}

static void set_robot_state(robot_state_t new_state) {
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        if (current_state != new_state) {
            printf("[%s] State change: %d -> %d\n", TAG, current_state, new_state);
            current_state = new_state;
        }
        xSemaphoreGive(state_mutex);
    }
}

// ============================================================================
// Mock IMU Physics Simulation
// ============================================================================
static void simulate_physics_step(float motor_output, float dt) {
    // Enhanced physics simulation for balance bot movement
    static float angle = 0.1f;          // Current angle (radians)
    static float angular_velocity = 0.0f; // Angular velocity (rad/s)
    static float position = 0.0f;       // Robot position (m)
    static float velocity = 0.0f;       // Robot velocity (m/s)
    static float target_velocity = 0.0f; // Target velocity from remote commands
    
    // Get target velocity from remote commands
    float cmd_velocity, cmd_turn;
    bool balance_enabled, standup_cmd;
    get_current_remote_command(&cmd_velocity, &cmd_turn, &balance_enabled, &standup_cmd);
    target_velocity = cmd_velocity * 2.0f; // Scale command to m/s
    
    // Physical constants (approximate for balance bot)
    const float gravity = 9.81f;        // m/s²
    const float length = 0.1f;          // Effective pendulum length (m)
    const float damping = 0.95f;        // Angular damping factor
    const float linear_damping = 0.9f;  // Linear velocity damping
    const float motor_force_factor = 0.015f; // Motor force scaling
    const float velocity_coupling = 0.1f; // How much velocity affects tilt
    
    // Calculate desired tilt angle for forward/backward movement
    // Forward movement requires slight backward tilt to accelerate
    float desired_tilt = -target_velocity * velocity_coupling;
    
    // Calculate angular acceleration from gravity, motor force, and movement dynamics
    float gravity_torque = -(gravity / length) * sin(angle);
    float motor_torque = motor_output * motor_force_factor;
    float movement_torque = (desired_tilt - angle) * 2.0f; // Gentle correction toward desired tilt
    float angular_acceleration = gravity_torque + motor_torque + movement_torque;
    
    // Integrate angular motion
    angular_velocity += angular_acceleration * dt;
    angular_velocity *= damping; // Apply damping
    angle += angular_velocity * dt;
    
    // Calculate linear motion based on tilt and motor output
    // When tilted forward (negative angle), robot should move forward (positive velocity)
    float tilt_acceleration = -sin(angle) * gravity * 0.5f; // Tilt contributes to linear motion
    float motor_acceleration = motor_output * 0.005f; // Direct motor contribution to acceleration
    
    // Update linear velocity and position
    velocity += (tilt_acceleration + motor_acceleration) * dt;
    velocity *= linear_damping; // Apply linear damping
    position += velocity * dt;
    
    // Convert angle to degrees and update shared data
    float angle_degrees = angle * 180.0f / M_PI;
    set_filtered_angle(angle_degrees);
    
    // Update robot velocity for display (convert m/s to cm/s)
    robot_velocity = velocity * 100.0f;
    
    // Debug output occasionally
    static int debug_counter = 0;
    if (++debug_counter >= 200) { // Every second at 200Hz
        printf("[PHYSICS] Pos=%.2fm, Vel=%.2fm/s, Angle=%.1f°, Target=%.2fm/s\n", 
               position, velocity, angle_degrees, target_velocity);
        debug_counter = 0;
    }
}

// ============================================================================
// Sensor Task (200Hz - ESP32-C6 고성능)
// ============================================================================
static void sensor_task(void *pvParameters) {
    BSW_LOGI(TAG, "Sensor task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(5); // 200Hz (ESP32-C6)
    
    // Initialize Kalman filter
    kalman_filter_init(&kalman_filter);
    kalman_filter_set_angle(&kalman_filter, 0.0f);
    
    while (1) {
        // Mock IMU data would normally be read here
        // For simulation, we'll use the physics simulation results
        float current_angle = get_filtered_angle();
        
        // Apply Kalman filtering (with mock gyro data)
        float mock_gyro = 0.0f; // Simplified - could add noise/dynamics  
        float filtered = kalman_filter_get_angle(&kalman_filter, current_angle, mock_gyro, 0.005f);
        set_filtered_angle(filtered);
        
        BSW_LOGD(TAG, "Sensor: Angle=%.2f°", filtered);
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// Balance Control Task (200Hz - ESP32-C6 고성능)
// ============================================================================
static void balance_task(void *pvParameters) {
    BSW_LOGI(TAG, "Balance task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(5); // 200Hz (ESP32-C6)
    
    while (1) {
        robot_state_t state = get_robot_state();
        float current_angle = get_filtered_angle();
        float motor_output = 0.0f;
        
        // 원격 명령 가져오기
        float target_velocity, target_turn;
        bool balance_enabled, standup_cmd;
        get_current_remote_command(&target_velocity, &target_turn, &balance_enabled, &standup_cmd);
        
        switch (state) {
        case ROBOT_STATE_INIT:
            // Initialization complete, move to idle
            set_robot_state(ROBOT_STATE_IDLE);
            break;
            
        case ROBOT_STATE_IDLE:
            // Check if we should start balancing
            if (balance_enabled && fabsf(current_angle) < MAX_TILT_ANGLE) {
                set_robot_state(ROBOT_STATE_BALANCING);
                balance_pid_reset(&balance_pid);
            }
            motor_output = 0.0f;
            break;
            
        case ROBOT_STATE_BALANCING:
            // Check if we've fallen over
            if (fabsf(current_angle) > MAX_TILT_ANGLE) {
                set_robot_state(ROBOT_STATE_FALLEN);
                balance_pid_reset(&balance_pid);
                break;
            }
            
            // Check if balancing should stop
            if (!balance_enabled) {
                set_robot_state(ROBOT_STATE_IDLE);
                break;
            }
            
            // Compute PID control output
            balance_pid_set_target_velocity(&balance_pid, target_velocity);
            motor_output = balance_pid_compute_balance(&balance_pid, 
                                                     current_angle, 
                                                     0.0f,  // Mock gyro 
                                                     robot_velocity, 
                                                     SIMULATION_DT);
            
            // 회전 명령 추가 (단순화)
            motor_output += target_turn * 10.0f;
            
            BSW_LOGD(TAG, "Balance: Angle=%.2f°, Output=%.2f", current_angle, motor_output);
            break;
            
        case ROBOT_STATE_FALLEN:
            // Try to recover if angle becomes reasonable
            if (fabsf(current_angle) < 30.0f) {
                set_robot_state(ROBOT_STATE_IDLE);
            }
            motor_output = 0.0f;
            break;
            
        case ROBOT_STATE_STANDING_UP:
            // Standing up sequence (simplified)
            if (standup_cmd) {
                motor_output = (current_angle > 0) ? -100.0f : 100.0f;
                if (fabsf(current_angle) < 10.0f) {
                    set_robot_state(ROBOT_STATE_IDLE);
                }
            } else {
                motor_output = 0.0f;
            }
            break;
            
        case ROBOT_STATE_ERROR:
        default:
            motor_output = 0.0f;
            break;
        }
        
        // Apply motor output to physics simulation
        simulate_physics_step(motor_output, SIMULATION_DT);
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// BLE Communication Task (새로 추가)
// ============================================================================
static void ble_task(void *pvParameters) {
    BSW_LOGI(TAG, "BLE task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(10); // 100Hz
    
    while (1) {
        // BLE 이벤트 처리
        ble_controller_update(&ble_ctrl);
        
        // 텍스트 명령 처리
        handle_text_commands();
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// Keyboard Input Task (50Hz)
// ============================================================================
static void keyboard_task(void *pvParameters) {
    BSW_LOGI(TAG, "Keyboard task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(20); // 50Hz
    
    while (1) {
        // 키보드 입력은 get_current_remote_command에서 처리됨
        // 이 태스크는 주기적으로 키 상태를 체크하고 디바운싱 처리
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// Status Task (1Hz)
// ============================================================================
static void status_task(void *pvParameters) {
    BSW_LOGI(TAG, "Status task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(1000); // 1Hz
    
    uint32_t loop_counter = 0;
    
    while (1) {
        robot_state_t state = get_robot_state();
        float angle = get_filtered_angle();
        
        printf("\n=== BalanceBot ESP32-C6 Simulator Status ===\n");
        printf("Loop: %u\n", loop_counter++);
        printf("State: %d\n", state);
        printf("Angle: %.2f°\n", angle);
        printf("Velocity: %.2f cm/s\n", robot_velocity);
        printf("Balance Enabled: %s\n", ble_ctrl.current_command.balance ? "YES" : "NO");
        printf("CPU: RISC-V %dMHz | SRAM: %dKB\n", ESP32C6_CPU_FREQ_MHZ, ESP32C6_SRAM_SIZE_KB);
        printf("Control Rate: 200Hz | Performance: x%.1f\n", ESP32C6_PERFORMANCE_FACTOR);
        printf("Free Heap: %dKB (Simulated)\n", ESP32C6_SRAM_SIZE_KB - 128); // 시뮬레이션
        printf("==========================================\n\n");
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// System Initialization
// ============================================================================
static void initialize_robot_systems(void) {
    BSW_LOGI(TAG, "Initializing ESP32-C6 robot systems...");
    BSW_LOGI(TAG, "CPU: RISC-V %dMHz, SRAM: %dKB", ESP32C6_CPU_FREQ_MHZ, ESP32C6_SRAM_SIZE_KB);
    
    // Initialize Config Manager
    int ret = config_manager_init();
    if (ret != 0) {
        BSW_LOGE(TAG, "Failed to initialize config manager");
        return;
    }
    
    // Initialize BLE Controller Mock
    ret = ble_controller_init(&ble_ctrl, "BalanceBot-C6-Sim");
    if (ret != 0) {
        BSW_LOGE(TAG, "Failed to initialize BLE controller");
        return;
    }
    
    // Initialize BSW layer
    bsw_gpio_init();
    
    bsw_i2c_config_t i2c_config = {
        .sda_pin = 21,
        .scl_pin = 22, 
        .clk_speed = 400000
    };
    bsw_i2c_init(BSW_I2C_NUM_0, &i2c_config);
    
    bsw_pwm_init();
    bsw_adc_init();
    
    // Initialize PID controller with saved parameters
    balance_pid_init(&balance_pid);
    
    // 저장된 파라미터로 PID 설정
    float balance_kp = config_manager_get_param(0);
    float balance_ki = config_manager_get_param(1);
    float balance_kd = config_manager_get_param(2);
    float velocity_kp = config_manager_get_param(3);
    float velocity_ki = config_manager_get_param(4);
    float velocity_kd = config_manager_get_param(5);
    
    balance_pid_set_balance_tunings(&balance_pid, balance_kp, balance_ki, balance_kd);
    balance_pid_set_velocity_tunings(&balance_pid, velocity_kp, velocity_ki, velocity_kd);
    balance_pid_set_max_tilt_angle(&balance_pid, MAX_TILT_ANGLE);
    
    BSW_LOGI(TAG, "ESP32-C6 robot systems initialized successfully");
    BSW_LOGI(TAG, "Balance PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", balance_kp, balance_ki, balance_kd);
    BSW_LOGI(TAG, "Performance factor: x%.1f, Control rate: 200Hz", ESP32C6_PERFORMANCE_FACTOR);
}

// ============================================================================
// Signal Handler for Clean Shutdown
// ============================================================================
static void signal_handler(int signum) {
    printf("\n[%s] Received signal %d, shutting down...\n", TAG, signum);
    
    // 키보드 설정 복원
    restore_keyboard();
    
    if (sensor_task_handle) vTaskDelete(sensor_task_handle);
    if (balance_task_handle) vTaskDelete(balance_task_handle);
    if (ble_task_handle) vTaskDelete(ble_task_handle);
    if (status_task_handle) vTaskDelete(status_task_handle);
    if (keyboard_task_handle) vTaskDelete(keyboard_task_handle);
    
    exit(0);
}

// ============================================================================
// FreeRTOS Hook Functions
// ============================================================================
void vApplicationIdleHook(void) {
    // Idle hook - can be used for low priority tasks
}

void vApplicationTickHook(void) {
    // Tick hook - called every tick interrupt
}

void vApplicationMallocFailedHook(void) {
    printf("[%s] ERROR: Malloc failed!\n", TAG);
    for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("[%s] ERROR: Stack overflow in task %s!\n", TAG, pcTaskName);
    for (;;);
}

// ============================================================================
// Main Function
// ============================================================================
int main(void) {
    printf("\n");
    printf("==========================================\n");
    printf("   BalanceBot ESP32-C6 Simulator v2.0   \n");
    printf("   RISC-V %dMHz + %dKB SRAM          \n", ESP32C6_CPU_FREQ_MHZ, ESP32C6_SRAM_SIZE_KB);
    printf("   FreeRTOS + Physics @ 200Hz           \n");
    printf("==========================================\n");
    printf("\n");
    
    printf("=== 키보드 조작법 ===\n");
    printf("↑: 앞으로      ↓: 뒤로\n");
    printf("←: 왼쪽 회전   →: 오른쪽 회전\n");
    printf("SPACE: 밸런싱 토글\n");
    printf("s: 정지        q: 종료\n");
    printf("==================\n");
    printf("\n");
    
    // Setup signal handler for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Setup keyboard input
    setup_keyboard();
    
    // 기본적으로 밸런싱 활성화
    keyboard_state.balance_toggle = true;
    
    // Initialize NVS (mock)
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        printf("[%s] Failed to initialize NVS\n", TAG);
        return -1;
    }
    
    // Create mutexes
    data_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL || state_mutex == NULL) {
        printf("[%s] Failed to create mutexes\n", TAG);
        return -1;
    }
    
    // Initialize robot systems
    initialize_robot_systems();
    
    // Create FreeRTOS tasks
    BaseType_t result;
    
    result = xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &sensor_task_handle);
    if (result != pdPASS) {
        printf("[%s] Failed to create sensor task\n", TAG);
        return -1;
    }
    
    result = xTaskCreate(balance_task, "balance_task", 4096, NULL, 4, &balance_task_handle);
    if (result != pdPASS) {
        printf("[%s] Failed to create balance task\n", TAG);
        return -1;
    }
    
    result = xTaskCreate(ble_task, "ble_task", 4096, NULL, 3, &ble_task_handle);
    if (result != pdPASS) {
        printf("[%s] Failed to create BLE task\n", TAG);
        return -1;
    }
    
    result = xTaskCreate(status_task, "status_task", 4096, NULL, 2, &status_task_handle);
    if (result != pdPASS) {
        printf("[%s] Failed to create status task\n", TAG);
        return -1;
    }
    
    result = xTaskCreate(keyboard_task, "keyboard_task", 4096, NULL, 1, &keyboard_task_handle);
    if (result != pdPASS) {
        printf("[%s] Failed to create keyboard task\n", TAG);
        return -1;
    }
    
    printf("[%s] All tasks created successfully\n", TAG);
    printf("[%s] Starting FreeRTOS scheduler...\n", TAG);
    printf("[%s] Use arrow keys to control the robot!\n", TAG);
    printf("[%s] Press 'q' to quit\n\n", TAG);
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    printf("[%s] ERROR: Scheduler returned!\n", TAG);
    return -1;
}