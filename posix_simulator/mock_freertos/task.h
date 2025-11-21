#ifndef TASK_H
#define TASK_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskCreate(void (*pxTaskCode)(void*), const char * const pcName, const uint16_t usStackDepth, void * const pvParameters, UBaseType_t uxPriority, TaskHandle_t * const pxCreatedTask);
void vTaskDelete(TaskHandle_t xTaskToDelete);
void vTaskStartScheduler(void);

#ifdef __cplusplus
}
#endif

#endif
