#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>
#include <stddef.h>

typedef uint32_t TickType_t;
typedef unsigned long UBaseType_t;
typedef long BaseType_t;

typedef void* TaskHandle_t;
typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* TimerHandle_t;

#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define pdFAIL 0

#define portMAX_DELAY 0xFFFFFFFFUL

#define pdMS_TO_TICKS(x) (x)

#endif
