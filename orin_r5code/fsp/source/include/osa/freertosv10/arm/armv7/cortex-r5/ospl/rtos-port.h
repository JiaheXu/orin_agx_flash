/* Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OSPL__RTOS_PORT_H
#define OSPL__RTOS_PORT_H
#define FSP__OSPL__RTOS_PORT_H                          1

/* Compiler headers */
#include <stdint.h>                         // for uint32_t, int32_t, int16_t

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <FreeRTOSConfig.h>                 // for configTICK_RATE_HZ, config...
#include <FreeRTOS.h>                       // IWYU pragma: export
                                            // IWYU pragma: no_include "FreeRTOS.h"
#include <portmacro.h>                      // IWYU pragma: export
                                            // IWYU pragma: no_include "portmacro.h"
                                            // for UBaseType_t, portBASE_TYPE, portSTACK_TYPE
#include <projdefs.h>                       // IWYU pragma: export
                                            // IWYU pragma: no_include "projdefs.h"
                                            // for TaskFunction_t
#include <portable.h>                       // IWYU pragma: export
                                            // IWYU pragma: no_include "portable.h"


/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
CT_ASSERT(FSP__CONFIG__FREERTOSCONFIG_H, "Header file missing or invalid.")

/*
 * Port specific types and data structures needed by nvosRTOS.h
 *
 * This file should not be included directly from any other file.
 */

/**
 * @brief Make sure that the sizes of types defined by FreeRTOS are what they
 * are expected to be.
 */
CT_ASSERT(sizeof(char) == sizeof(portCHAR), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(float) == sizeof(portFLOAT), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(double) == sizeof(portDOUBLE), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(long) == sizeof(portLONG), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(short) == sizeof(portSHORT), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(uint32_t) == sizeof(portSTACK_TYPE), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(long) == sizeof(portBASE_TYPE), "ASSERT due to data type mismatch.")

/**
 * @brief Provide wrapper type definitions for RTOS values to avoid exposing the
 * FreeRTOS types directly.
 *
 * @typedef-title OSA variable type space
 */
typedef portBASE_TYPE           rtosError;
typedef void                    *rtosTaskHandle;
typedef void                    *rtosEventGroupHandle;
typedef StaticEventGroup_t      rtosEventGroup;
typedef void                    *rtosQueueHandle;
typedef void                    *rtosTimerHandle;
typedef void                    *rtosTimerLocalStorageObject;
typedef uint32_t                portUInt32Type;
typedef uint8_t                 portUInt8Type;
typedef UBaseType_t             rtosUCount;
typedef portSTACK_TYPE          rtosTick;
typedef portBASE_TYPE           rtosBool;
typedef portBASE_TYPE           rtosTimerIDType;
typedef void                    *rtosMutexHandle;
typedef void                    *rtosSemaphoreHandle;
typedef void                    *rtosEventPollHandle;
typedef UBaseType_t             rtosTaskNotifyBits;
typedef UBaseType_t             rtosEventPollValue;
typedef portBASE_TYPE           rtosTaskNotifyAction;
typedef UBaseType_t             rtosTaskNotifyValue;
typedef UBaseType_t             rtosPriority;
typedef UBaseType_t             rtosInterruptState;
typedef UBaseType_t             rtosMutexState;
typedef unsigned long           rtosEventBits;
typedef portSTACK_TYPE          rtosHz;
typedef UBaseType_t             rtosPrivilegeLevel;
typedef UBaseType_t             rtosUBaseType_t;
typedef portBASE_TYPE           rtosPortBaseType;
typedef void *			portTaskHandleType;
typedef StaticSemaphore_t       rtosSemaphoreBuffer;
typedef StaticQueue_t           rtosQueueBuffer;
typedef StaticEventGroup_t      rtosEventGroupBuffer;
typedef StaticTask_t            rtosTaskBuffer;
typedef StackType_t             rtosStackType;

/**
 * @brief Provide wrapper definition for pointer to RTOS task function.
 */
typedef TaskFunction_t             rtosTaskFunction;


/**
 * @brief Provide wrapper definition for pointer to RTOS timer callback function
 * which gets invoked when a timer expires.
 */
typedef void (*rtosTimerCallbackFunction)(void * pvParameters);

/**
 * @brief Structure supplied to rtosTaskCreate()
 *
 * @pvTaskCode        Pointer to task entry function
 * @pcTaskName        Task descriptive name
 * @pxTCB             Pointer to task control block
 * @pcStackBuffer     Pointer to task stack buffer
 * @uxStackDepthBytes Size of task stack buffer in bytes. The minimum
 *                    allowable size for the stack buffer is portdependent.
 * @pvParameters      Pointer to task parameters
 * @uxPriority        Priority of task.
 *                    Valid range is [0 - (configMAX_PRIORITIES - 1])
 * @pvObject          Pointer to user defined data associated with this task.
 * @xUsingFPU         Bool to indicate use of FPU
 * @uxPrivilegeLevel  Task Privilege level
 */
typedef struct {
    rtosTaskFunction     pvTaskCode;
    char                *pcTaskName;
    void                *pxTCB;
    void                *pcStackBuffer;
    rtosUCount           uxStackDepthBytes;
    void                *pvParameters;
    rtosPriority         uxPriority;
    void                *pvObject;
    rtosBool             xUsingFPU;
    rtosPrivilegeLevel   uxPrivilegeLevel;
} rtosTaskParameters;

/**
 * @brief Structure supplied to rtosTaskInitializeScheduler()
 *
 * @ulCPUClockHz                      Speed of the system clock
 * @ulTickRateHz                      Desired frequency of kernel tick
 * @uxAdditionalStackCheckMarginBytes Lower limit on number of bytes that
 *                                    should be left in the stack post context
 *                                    switch. Can be set greater than or equal
 *                                    to 0
 * @pcIdleTaskStackBuffer             Pointer to the lowest address of task
 *                                    buffer to hold idle task
 * @uxIdleTaskStackSizeBytes          Number of bytes in the stack buffer to be
 *                                    kept for idle task. Should be greater
 *                                    than 0
 * @xIdleTaskUsingFPU                 Bool variable to indicate use of FPU by
 *                                    idle task
 * @uxIdleTaskPrivilegeLevel          Idle task privilege level
 *                                    [RTOS_SYSTEM_MODE, RTOS_USER_MODE]
 * @pvIdleTaskTLSObject               Idle task timer local storage object
 * @uxTimerTaskPriority               Priority of timer service.
 *                                    Valid range is [0 - (configMAX_PRIORITIES - 1)]
 * @uxTimerTaskStackSize              Number of bytes in stack buffer to be kept
 *                                    for timer task. Should be greater than 0
 * @pcTimerTaskStackBuffer            Pointer to the timer task stack buffer.
 * @uxTimerCommandQueueLength         Timer command queue length. Should be
 *                                    greater than 0.
 * @uxTimerCommandQueueBufferSize     Timer command queue buffer size.
 *                                    Size of the buffer should be at
 *                                    least enough to accomodate the entire
 *                                    command queue
 * @pcTimerCommandQueueBuffer         Pointer to timer command queue buffer
 */
typedef struct {
    rtosHz              ulCPUClockHz;
    rtosHz              ulTickRateHz;
    rtosUCount          uxAdditionalStackCheckMarginBytes;
    void                *pcIdleTaskStackBuffer;
    rtosUCount          uxIdleTaskStackSizeBytes;
    rtosBool            xIdleTaskUsingFPU;
    rtosPrivilegeLevel  uxIdleTaskPrivilegeLevel;
    void                *pvIdleTaskTLSObject;
    rtosPriority        uxTimerTaskPriority;
    rtosUCount          uxTimerTaskStackSize;
    void                *pcTimerTaskStackBuffer;
    rtosUCount          uxTimerCommandQueueLength;
    rtosUCount          uxTimerCommandQueueBufferSize;
    void                *pcTimerCommandQueueBuffer;
} rtosSchedParameters;

/**
 * @brief Structure supplied to rtosTimerCreate()
 *
 * @pcTimerName         Descriptive name of the timer
 * @xTimerPeriodInTicks Timer period in ticks. Should be greater than 0
 * @xIsPeriodic         Bool variable to indicate if timer is periodic
 * @xTimerID            Timer ID
 * @pxNewTimer          Pointer to timer control block
 * @pxCallbackFunction  Timer callback function
 * @pxTimerInstance     Timer instance to which the timer belongs
 * @pvObject            Pointer to user defined data associated with this timer.
 */
typedef struct {
    const char                *pcTimerName;
    rtosTick                  xTimerPeriodInTicks;
    rtosBool                  xIsPeriodic;
    rtosTimerIDType           xTimerID;
    void                      *pxNewTimer;
    rtosTimerCallbackFunction pxCallbackFunction;
    void                      *pxTimerInstance;
    void                      *pvObject;
} rtosTimerInitParametersType;

/**
 * @brief Provide wrapper definitions for general architecture specific values
 *
 * @macro-title RTOS general architecture values
 *
 * @rtosMAX_DELAY             Defines maximum delay supported by rtos
 * @rtosSTACK_IN_USE          Defines state value indicating stack is in use
 * @rtosSTACK_NOT_IN_USE      Defines state value indicating stack is not in use
 * @rtosTICK_COUNT_BITS       Defines number of bits used to represent tick
 *                            count
 * @rtosTICK_RATE_HZ          Defines frequency of the underlying RTOS scheduler
 *                            tick
 * @rtosTICK_RATE_MS          Defines number of ticks per millisecond.
 */
#define rtosMAX_DELAY                           portMAX_DELAY
#define rtosSTACK_IN_USE                        portSTACK_IN_USE
#define rtosSTACK_NOT_IN_USE                    portSTACK_NOT_IN_USE
#define rtosTICK_COUNT_BITS                     portTICK_COUNT_NUM_BITS
#define rtosTICK_RATE_HZ                        configTICK_RATE_HZ
#define rtosTICK_RATE_MS                        portTICK_PERIOD_MS

#endif
