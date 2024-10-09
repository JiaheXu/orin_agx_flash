/* Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>                        // for uint32_t, int32_t, int16_t

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <SafeRTOSConfig.h>                // for configTICK_RATE_HZ, config...
                                           // IWYU pragma: no_include "portmacro.h"
#include <SafeRTOS_API.h>                  // for safertosapiQUEUE_OVERHEAD_BYTES
#include <eventGroupsAPI.h>                // for eventBitsType, eventGroupType
#include <evtmplxAPI.h>                    // for evtMplxObjectEventsType
#include <cpu/armv7-mpu-safertos.h>        // for mpuParameters_t

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
    CT_ASSERT(FSP__CPU__ARMV7_MPU_SAFERTOS_H, "Header file missing or invalid.")

/*
 * Port specific types and data structures needed by nvosRTOS.h
 *
 * This file should not be included directly from any other file.
 */

/**
 * @brief Make sure that the sizes of types defined by SafeRTOS are what they
 * are expected to be.
 */
CT_ASSERT(sizeof(int16_t) == sizeof(portInt16Type), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(uint8_t) == sizeof(portUInt8Type), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(int8_t) == sizeof(portInt8Type), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(char) == sizeof(portCharType), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(uint32_t) == sizeof(portStackType), "ASSERT due to data type mismatch.")
CT_ASSERT(sizeof(uint32_t) == sizeof(portTickType), "ASSERT due to data type mismatch.")

/**
 * @brief Provide wrapper type definitions for RTOS values to avoid exposing the
 * SafeRTOS types directly.
 *
 * @typedef-title OSA variable type space
 */
typedef portBaseType            rtosError;
typedef void                    *rtosTaskHandle;
typedef void                    *rtosEventGroupHandle;
typedef eventGroupType          rtosEventGroup;
typedef void                    *rtosQueueHandle;
typedef void                    *rtosTimerHandle;
typedef void                    *rtosTimerLocalStorageObject;
typedef portUnsignedBaseType    rtosUCount;
typedef portTickType            rtosTick;
typedef portBaseType            rtosBool;
typedef portBaseType            rtosTimerIDType;
typedef void                    *rtosMutexHandle;
typedef void                    *rtosSemaphoreHandle;
typedef void                    *rtosEventPollHandle;
typedef portUnsignedBaseType    rtosTaskNotifyBits;
typedef portUnsignedBaseType    rtosEventPollValue;
typedef portBaseType            rtosTaskNotifyAction;
typedef portUnsignedBaseType    rtosTaskNotifyValue;
typedef portUnsignedBaseType    rtosPriority;
typedef portUnsignedBaseType    rtosInterruptState;
typedef portUnsignedBaseType    rtosMutexState;
typedef eventBitsType           rtosEventBits;
typedef portUInt32Type          rtosHz;
typedef portUnsignedBaseType    rtosPrivilegeLevel;
typedef portUnsignedBaseType    rtosUInt32Type;

/**
 * @brief Sizes of objects that need to be statically allocated but whose
 * contents are not visible to the application.
 *
 * @macro-title Macro definitions for size of kernel objects
 *
 * @rtosQUEUE_OVERHEAD_BYTES                    Defines size in bytes needed to
 *                                              create a queue
 * @rtosSEMAPHORE_OVERHEAD_BYTES                Defines size in bytes needed to
 *                                              create a semaphore object
 * @rtosMUTEX_OVERHEAD_BYTES                    Defines size in bytes needed to
 *                                              create a mutex object
 * @rtosEVENT_GROUP_OVERHEAD_BYTES              Defines size in bytes needed to
 *                                              create an event group object
 * @rtosTCB_OVERHEAD_BYTES                      Defines size in bytes needed for
 *                                              task control block
 * @rtosTIMER_CONTROL_BLOCK_OVERHEAD_BYTES      Defines size in bytes needed for
 *                                              timer control block
 * @rtosTIMER_INSTANCE_PARAMETER_OVERHEAD_BYTES Defines size in bytes needed for
 *                                              timer instance parameters
 * @rtosEVENT_POLL_OVERHEAD_BYTES               Defines size in bytes needed to
 *                                              create an event poll object
 */
#define rtosQUEUE_OVERHEAD_BYTES                   safertosapiQUEUE_OVERHEAD_BYTES
#define rtosSEMAPHORE_OVERHEAD_BYTES               safertosapiQUEUE_OVERHEAD_BYTES
#define rtosMUTEX_OVERHEAD_BYTES                   safertosapiQUEUE_OVERHEAD_BYTES
#define rtosEVENT_GROUP_OVERHEAD_BYTES             sizeof(eventGroupType)
#define rtosTCB_OVERHEAD_BYTES                     sizeof(xTCB)
#define rtosTIMER_CONTROL_BLOCK_OVERHEAD_BYTES     sizeof(timerControlBlockType)
#define rtosTIMER_INSTANCE_PARAMETER_OVERHEAD_BYTES \
        sizeof(struct timerInstanceParameters)
#define rtosEVENT_POLL_OVERHEAD_BYTES(uxMaximumRegisteredObjectEvents) \
        evtmplxGET_REQUIRED_CREATE_BUFFER_SIZE(uxMaximumRegisteredObjectEvents)

/**
 * @brief Provide wrapper definition for pointer to RTOS task function.
 */
typedef pdTASK_CODE             rtosTaskFunction;

/**
 * @brief Provide a wrapper data type to encapsulate MPU related information
 */
typedef mpuParameters_t rtosMPURegionConfig;

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
 * @pxMPUParameters   Pointer to task MPU parameters
 */
typedef struct {
    rtosTaskFunction    pvTaskCode;
    const char          *pcTaskName;
    void                *pxTCB;
    void                *pcStackBuffer;
    rtosUCount          uxStackDepthBytes;
    void                *pvParameters;
    rtosPriority        uxPriority;
    void                *pvObject;
    rtosBool            xUsingFPU;
    rtosPrivilegeLevel  uxPrivilegeLevel;
    rtosMPURegionConfig *pxMPUParameters;
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
 * @pxIdleTaskMPUParameters           Pointer to idle task MPU parameter
 *                                    structure
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
 * @pxTimerTaskMPUParameters          Pointer to timer task MPU parameters
 */
typedef struct {
    rtosHz              ulCPUClockHz;
    rtosHz              ulTickRateHz;
    rtosUCount          uxAdditionalStackCheckMarginBytes;
#if defined(SAFERTOS_VARIANT_SVC) && (SAFERTOS_VARIANT_SVC == 1)
    void                *pcSVCStackBaseAddress;
#endif
    void                *pcIdleTaskStackBuffer;
    rtosUCount          uxIdleTaskStackSizeBytes;
    rtosBool            xIdleTaskUsingFPU;
    rtosPrivilegeLevel  uxIdleTaskPrivilegeLevel;
    rtosMPURegionConfig *pxIdleTaskMPUParameters;
    void                *pvIdleTaskTLSObject;
    rtosPriority        uxTimerTaskPriority;
    rtosUCount          uxTimerTaskStackSize;
    void                *pcTimerTaskStackBuffer;
    rtosUCount          uxTimerCommandQueueLength;
    rtosUCount          uxTimerCommandQueueBufferSize;
    void                *pcTimerCommandQueueBuffer;
    rtosMPURegionConfig *pxTimerTaskMPUParameters;
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
    rtosTaskHandle            xTaskToNotify;
} rtosTimerInitParametersType;

/**
 * @brief Wrapper structure for RTOS object-events
 */
#define rtosEventPollObjectEventsType evtMplxObjectEventsType 

/**
 * @brief Provide wrapper definitions for general architecture specific values
 *
 * @macro-title RTOS general architecture values
 *
 * @rtosMAX_DELAY             Defines maximum delay supported by rtos
 * @rtosMAX_LIST_ITEM_VALUE   Defines maximum list items supported by rtos
 * @rtosWORD_ALIGNMENT        Defines alignment value set for queue buffer
 * @rtosWORD_ALIGNMENT_MASK   Defines alignment mask corresponding to alignment
 *                            value set for queue buffer
 * @rtosSTACK_ALIGNMENT       Defines alignment value set for stack buffer
 * @rtosSTACK_ALIGNMENT_MASK  Defines alignment mask corresponding to alignment
 *                            value set for stack buffer
 * @rtosCONTEXT_SIZE_NO_FPU   Defines context size in bytes without FPU
 * @rtosCONTEXT_SIZE_WITH_FPU Defines context size in bytes with FPU in use
 * @rtosSTACK_IN_USE          Defines state value indicating stack is in use
 * @rtosSTACK_NOT_IN_USE      Defines state value indicating stack is not in use
 * @rtosTICK_COUNT_BITS       Defines number of bits used to represent tick
 *                            count
 * @rtosTICK_RATE_HZ          Defines frequency of the underlying RTOS scheduler
 *                            tick
 * @rtosTICK_RATE_MS          Defines number of ticks per millisecond.
 */
#define rtosMAX_DELAY                           safertosapiMAX_DELAY
#define rtosMAX_LIST_ITEM_VALUE                 portMAX_LIST_ITEM_VALUE
#define rtosWORD_ALIGNMENT                      portWORD_ALIGNMENT
#define rtosWORD_ALIGNMENT_MASK                 portWORD_ALIGNMENT_MASK
#define rtosSTACK_ALIGNMENT                     portSTACK_ALIGNMENT
#define rtosSTACK_ALIGNMENT_MASK                portSTACK_ALIGNMENT_MASK
#define rtosCONTEXT_SIZE_NO_FPU                 portCONTEXT_SIZE_BYTES_NO_FPU
#define rtosCONTEXT_SIZE_WITH_FPU               portCONTEXT_SIZE_BYTES_WITH_FPU
#define rtosSTACK_IN_USE                        portSTACK_IN_USE
#define rtosSTACK_NOT_IN_USE                    portSTACK_NOT_IN_USE
#define rtosTICK_COUNT_BITS                     portTICK_COUNT_NUM_BITS
#define rtosTICK_RATE_HZ                        configTICK_RATE_HZ
#define rtosTICK_RATE_MS                        configTICK_RATE_MS

extern portUInt32Type xHookGetRTSCounter(void);

/**
 * @brief Provide macro definitions for ARM state values.
 *
 * @macro-title ARMV7 specific values
 *
 * @RTOS_SYSTEM_MODE Defines value indicating ARM system mode operation
 * @RTOS_USER_MODE   Defines value indicating ARM user mode operation
 */
#define RTOS_SYSTEM_MODE                safertosapiPRIVILEGED_TASK
#define RTOS_USER_MODE                  safertosapiUNPRIVILEGED_TASK

#endif
