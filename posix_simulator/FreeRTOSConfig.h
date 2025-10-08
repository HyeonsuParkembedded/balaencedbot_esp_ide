/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS Configuration for BalanceBot POSIX Simulator
 * 
 * POSIX 포트에 최적화된 FreeRTOS 설정입니다.
 * 
 * @author Hyeonsu Park
 * @date 2025-10-08
 * @version 1.0
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    extern uint32_t SystemCoreClock;
#endif

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     1
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 1000000000 ) /* Simulated 1GHz */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 7 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 512 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 65 * 1024 ) )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xSemaphoreGetMutexHolder        1

/* Normal assert() semantics without relying on the provision of an assert.h header file. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - or at least those used in the unmodified vector table. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* POSIX port specific definitions */
#define portUSE_HEAP_MALLOC                     1

/* Enable stack overflow checking. */
#define configCHECK_FOR_STACK_OVERFLOW          2

/* The size of the global output buffer that is available for use when
there are multiple command interpreters running at once (for example, one on
a UART and one on TCP/IP).  This is done to prevent an output buffer being
defined by each implementation - which would waste RAM.  In this case, there
is only one command interpreter running. */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE       2048

/* The __weak attribute does not exist for the IAR compiler.  It is only
used with the generic script, and the generic script doesn't work with the
IAR compiler. */
#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION 0

/* This demo makes use of one or more example stats formatting functions.  These
format the raw data provided by the uxTaskGetSystemState() function in to human
readable ASCII form.  See the notes in the implementation of vTaskList() within
FreeRTOS/Source/tasks.c for limitations. */
#define configUSE_STATS_FORMATTING_FUNCTIONS    1

/* Dimensions a buffer into which task state information is written.  The buffer
must be large enough to hold information on the number of tasks created by the
example code. */
#define configSTATS_BUFFER_MAX_LENGTH           ( ( size_t ) 0x8000 )

/* Set configUSE_TASK_NOTIFICATIONS to 1 to enable task notifications, or 0
to disable. */
#define configUSE_TASK_NOTIFICATIONS            1

/* Set configUSE_POSIX_ERRNO to 1 to enable per task POSIX style errno
functionality, or 0 to disable it. */
#define configUSE_POSIX_ERRNO                   1

/* When the FreeRTOS POSIX port is used main() is called by the POSIX port so
main() cannot be used as the FreeRTOS idle task. */
#define configIDLE_TASK_NAME                    "IDLE"
#define configTIMER_SERVICE_TASK_NAME           "Tmr Svc"

/* Methods for obtaining seed for random number generator */
#include <time.h>
#define configRAND32()    rand()
#define configSEED_RANDOM( x ) srand( x )

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */