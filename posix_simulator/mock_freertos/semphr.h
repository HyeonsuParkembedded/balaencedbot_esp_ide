#ifndef SEMPHR_H
#define SEMPHR_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define xSemaphoreCreateMutex() (SemaphoreHandle_t)1
#define xSemaphoreTake(xSemaphore, xBlockTime) pdTRUE
#define xSemaphoreGive(xSemaphore) pdTRUE

#ifdef __cplusplus
}
#endif

#endif
