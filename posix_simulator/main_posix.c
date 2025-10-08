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

static const char* TAG = "POSIX_SIM";

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

// Shared sensor data
static float filtered_angle = 0.0f;
static float robot_velocity = 0.0f;
static bool balancing_enabled __attribute__((unused)) = true;

// Task handles
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t balance_task_handle = NULL;
static TaskHandle_t status_task_handle = NULL;

// Simulation parameters
#define SIMULATION_DT           0.02f   // 50Hz simulation rate
#define MAX_TILT_ANGLE         45.0f    // Maximum tilt before "fallen" state

// ============================================================================
// Mock Remote Command Interface
// ============================================================================
typedef struct {
    bool balance;
    bool standup;
    float forward_speed;
    float turn_speed;
} remote_command_t;

static remote_command_t mock_remote_cmd = {
    .balance = true,
    .standup = false, 
    .forward_speed = 0.0f,
    .turn_speed = 0.0f
};

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
    // Simple inverted pendulum physics simulation
    static float angle = 0.1f;          // Current angle (radians)
    static float angular_velocity = 0.0f; // Angular velocity (rad/s)
    
    // Physical constants (approximate for balance bot)
    const float gravity = 9.81f;        // m/s²
    const float length = 0.1f;          // Effective pendulum length (m)
    const float damping = 0.95f;        // Damping factor
    const float motor_force_factor = 0.01f; // Motor force scaling
    
    // Calculate angular acceleration from gravity and motor force
    float gravity_torque = -(gravity / length) * sin(angle);
    float motor_torque = motor_output * motor_force_factor;
    float angular_acceleration = gravity_torque + motor_torque;
    
    // Integrate to get new angular velocity and angle
    angular_velocity += angular_acceleration * dt;
    angular_velocity *= damping; // Apply damping
    angle += angular_velocity * dt;
    
    // Convert angle to degrees and update filtered angle
    float angle_degrees = angle * 180.0f / M_PI;
    set_filtered_angle(angle_degrees);
    
    // Update velocity based on motor output (simplified)
    robot_velocity = motor_output * 0.1f; // Very simple velocity model
}

// ============================================================================
// Sensor Task (50Hz)
// ============================================================================
static void sensor_task(void *pvParameters) {
    BSW_LOGI(TAG, "Sensor task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(20); // 50Hz
    
    // Initialize Kalman filter
    kalman_filter_init(&kalman_filter);
    kalman_filter_set_angle(&kalman_filter, 0.0f);
    
    while (1) {
        // Mock IMU data would normally be read here
        // For simulation, we'll use the physics simulation results
        float current_angle = get_filtered_angle();
        
        // Apply Kalman filtering (with mock gyro data)
        float mock_gyro = 0.0f; // Simplified - could add noise/dynamics
        float filtered = kalman_filter_get_angle(&kalman_filter, current_angle, mock_gyro, 0.02f);
        set_filtered_angle(filtered);
        
        BSW_LOGD(TAG, "Sensor: Angle=%.2f°", filtered);
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// Balance Control Task (50Hz)
// ============================================================================
static void balance_task(void *pvParameters) {
    BSW_LOGI(TAG, "Balance task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(20); // 50Hz
    
    while (1) {
        robot_state_t state = get_robot_state();
        float current_angle = get_filtered_angle();
        float motor_output = 0.0f;
        
        switch (state) {
        case ROBOT_STATE_INIT:
            // Initialization complete, move to idle
            set_robot_state(ROBOT_STATE_IDLE);
            break;
            
        case ROBOT_STATE_IDLE:
            // Check if we should start balancing
            if (mock_remote_cmd.balance && fabsf(current_angle) < MAX_TILT_ANGLE) {
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
            if (!mock_remote_cmd.balance) {
                set_robot_state(ROBOT_STATE_IDLE);
                break;
            }
            
            // Compute PID control output
            balance_pid_set_target_velocity(&balance_pid, mock_remote_cmd.forward_speed);
            motor_output = balance_pid_compute_balance(&balance_pid, 
                                                     current_angle, 
                                                     0.0f,  // Mock gyro 
                                                     robot_velocity, 
                                                     SIMULATION_DT);
            
            BSW_LOGI(TAG, "Balance: Angle=%.2f°, Output=%.2f", current_angle, motor_output);
            break;
            
        case ROBOT_STATE_FALLEN:
            // Try to recover if angle becomes reasonable
            if (fabsf(current_angle) < 30.0f) {
                set_robot_state(ROBOT_STATE_IDLE);
            }
            motor_output = 0.0f;
            break;
            
        case ROBOT_STATE_STANDING_UP:
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
        
        printf("\n=== BalanceBot POSIX Simulator Status ===\n");
        printf("Loop: %u\n", loop_counter++);
        printf("State: %d\n", state);
        printf("Angle: %.2f°\n", angle);
        printf("Velocity: %.2f cm/s\n", robot_velocity);
        printf("Balance Enabled: %s\n", mock_remote_cmd.balance ? "YES" : "NO");
        printf("Free Heap: Simulated (POSIX)\n");
        printf("========================================\n\n");
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// System Initialization
// ============================================================================
static void initialize_robot_systems(void) {
    BSW_LOGI(TAG, "Initializing robot systems...");
    
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
    
    // Initialize PID controller with tuned parameters
    balance_pid_init(&balance_pid);
    balance_pid_set_balance_tunings(&balance_pid, 50.0f, 0.5f, 2.0f);
    balance_pid_set_velocity_tunings(&balance_pid, 1.0f, 0.1f, 0.0f);
    balance_pid_set_max_tilt_angle(&balance_pid, MAX_TILT_ANGLE);
    
    BSW_LOGI(TAG, "Robot systems initialized successfully");
}

// ============================================================================
// Signal Handler for Clean Shutdown
// ============================================================================
static void signal_handler(int signum) {
    printf("\n[%s] Received signal %d, shutting down...\n", TAG, signum);
    
    if (sensor_task_handle) vTaskDelete(sensor_task_handle);
    if (balance_task_handle) vTaskDelete(balance_task_handle);
    if (status_task_handle) vTaskDelete(status_task_handle);
    
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
    printf("========================================\n");
    printf("   BalanceBot POSIX Simulator v1.0     \n");
    printf("   FreeRTOS + Physics Simulation       \n");
    printf("========================================\n");
    printf("\n");
    
    // Setup signal handler for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
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
    
    result = xTaskCreate(status_task, "status_task", 4096, NULL, 3, &status_task_handle);
    if (result != pdPASS) {
        printf("[%s] Failed to create status task\n", TAG);
        return -1;
    }
    
    printf("[%s] All tasks created successfully\n", TAG);
    printf("[%s] Starting FreeRTOS scheduler...\n", TAG);
    printf("[%s] Press Ctrl+C to exit\n\n", TAG);
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    printf("[%s] ERROR: Scheduler returned!\n", TAG);
    return -1;
}