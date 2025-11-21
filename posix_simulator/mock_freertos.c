/**
 * @file mock_freertos.c
 * @brief FreeRTOS Mock Implementation using Pthreads
 * 
 * POSIX 환경에서 FreeRTOS API를 시뮬레이션하기 위한 구현체입니다.
 * pthread를 사용하여 태스크와 뮤텍스를 에뮬레이션합니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#include "mock_freertos/FreeRTOS.h"
#include "mock_freertos/task.h"
#include "mock_freertos/semphr.h"
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <stdbool.h>

// ============================================================================
// Time Management
// ============================================================================
static uint32_t start_time_ms = 0;

static uint32_t get_current_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

TickType_t xTaskGetTickCount(void) {
    if (start_time_ms == 0) {
        start_time_ms = get_current_time_ms();
    }
    return get_current_time_ms() - start_time_ms;
}

void vTaskDelay(const TickType_t xTicksToDelay) {
    usleep(xTicksToDelay * 1000);
}

void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement) {
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t targetTime = *pxPreviousWakeTime + xTimeIncrement;
    
    if (targetTime > currentTime) {
        vTaskDelay(targetTime - currentTime);
    }
    
    *pxPreviousWakeTime = targetTime;
}

// ============================================================================
// Task Management
// ============================================================================
typedef struct {
    void (*task_code)(void*);
    void* params;
    pthread_t thread;
    bool running;
} mock_task_t;

static void* thread_wrapper(void* arg) {
    mock_task_t* t = (mock_task_t*)arg;
    t->task_code(t->params);
    return NULL;
}

BaseType_t xTaskCreate(void (*pxTaskCode)(void*), const char * const pcName, const uint16_t usStackDepth, void * const pvParameters, UBaseType_t uxPriority, TaskHandle_t * const pxCreatedTask) {
    mock_task_t* task = (mock_task_t*)malloc(sizeof(mock_task_t));
    if (task == NULL) return pdFAIL;
    
    task->task_code = pxTaskCode;
    task->params = pvParameters;
    task->running = true;
    
    int ret = pthread_create(&task->thread, NULL, thread_wrapper, task);
    if (ret != 0) {
        free(task);
        return pdFAIL;
    }
    
    if (pxCreatedTask != NULL) {
        *pxCreatedTask = (TaskHandle_t)task;
    }
    
    printf("[MOCK_RTOS] Task created: %s\n", pcName);
    return pdPASS;
}

void vTaskDelete(TaskHandle_t xTaskToDelete) {
    if (xTaskToDelete != NULL) {
        mock_task_t* task = (mock_task_t*)xTaskToDelete;
        if (task->running) {
            pthread_cancel(task->thread);
            pthread_join(task->thread, NULL);
            task->running = false;
        }
        free(task);
    } else {
        // Delete self - tricky with pthreads, usually just exit thread
        pthread_exit(NULL);
    }
}

void vTaskStartScheduler(void) {
    // In POSIX simulation, main thread usually blocks or does something else
    // Here we just loop forever to keep the process alive
    while (1) {
        sleep(1);
    }
}

// ============================================================================
// Semaphore/Mutex Management
// ============================================================================
typedef struct {
    pthread_mutex_t mutex;
} mock_mutex_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    mock_mutex_t* m = (mock_mutex_t*)malloc(sizeof(mock_mutex_t));
    if (m == NULL) return NULL;
    
    pthread_mutex_init(&m->mutex, NULL);
    return (SemaphoreHandle_t)m;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime) {
    if (xSemaphore == NULL) return pdFAIL;
    
    mock_mutex_t* m = (mock_mutex_t*)xSemaphore;
    pthread_mutex_lock(&m->mutex);
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    if (xSemaphore == NULL) return pdFAIL;
    
    mock_mutex_t* m = (mock_mutex_t*)xSemaphore;
    pthread_mutex_unlock(&m->mutex);
    return pdTRUE;
}
