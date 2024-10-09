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

#ifndef OSA__OSA_VALUES_H
#define OSA__OSA_VALUES_H
#define FSP__OSA__OSA_VALUES_H                          1

/* Compiler headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <ospl/rtos-port.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "ospl/rtos-port.h"
                                        // for pdFALSE, pdTRUE, pdFAIL, pdPASS

/**
 * @brief Mapping of FreeRTOS values to rtos value space
 *
 * @macro-title RTOS boolean values published by OSA layer
 *
 * @rtosTRUE  Defines true state of a boolean variable
 * @rtosFALSE Defines false state of a boolean variable
 * @rtosPASS  Defines return value indicating success
 * @rtosFAIL  Defines return value indicating failure
 */
#define rtosTRUE                                ((rtosBool)pdTRUE)
#define rtosFALSE                               ((rtosBool)pdFALSE)
#define rtosPASS                                pdPASS
#define rtosFAIL                                pdFAIL

/**
 * @brief Mapping of FreeRTOS kernel version values to rtos value space
 *
 * @macro-title Kernel version values published by OSA layer.
 *
 * @rtosKERNEL_MAJOR_VERSION Defines kernel major version number
 * @rtosKERNEL_MINOR_VERSION Defines kernel minor version number
 */
#define rtosKERNEL_VERSION_NUMBER               tskKERNEL_VERSION_NUMBER
#define rtosKERNEL_MAJOR_VERSION                tskKERNEL_VERSION_MAJOR
#define rtosKERNEL_MINOR_VERSION                tskKERNEL_VERSION_MINOR
#define rtosKERNEL_MINOR_BUILD                  tskKERNEL_VERSION_BUILD

/**
 * @brief Mapping of FreeRTOS event group values to rtos value space
 *
 * @macro-title Event group values published by OSA layer
 *
 * @rtosEVENTGRP_CLEAR_EVENTS_ON_EXIT Defines bit-mask mapped to eventgroup
 *                                    control flag set to clear event group
 *                                    bits on exit
 * @rtosEVENTGRP_WAIT_FOR_ALL_BITS    Defines bit-mask mapped to eventgroup
 *                                    control flag, set to wait until all the
 *                                    requested event group bits are set.
 */
#define rtosEVENTGRP_CLEAR_EVENTS_ON_EXIT       eventCLEAR_EVENTS_ON_EXIT_BIT
#define rtosEVENTGRP_WAIT_FOR_ALL_BITS          eventWAIT_FOR_ALL_BITS
#define rtosEVENTGRP_UNBLOCKED_DUE_TO_BIT_SET   eventUNBLOCKED_DUE_TO_BIT_SET
#define rtosEVENTGRP_BITS_CONTROL_BYTES         eventEVENT_BITS_CONTROL_BYTES

/**
 * @brief Mapping of FreeRTOS mutex values to rtos value space
 *
 * @macro-title Mutex state values published by OSA layer
 *
 * @rtosMUTEX_AVAILABLE Defines state of a mutex object indicating lock is
 *                      available
 * @rtosMUTEX_TAKEN     Defines state of a mutex object indicating lock is
 *                      acquired
 */
#define rtosMUTEX_TAKEN                         pdTRUE
#define rtosMUTEX_AVAILABLE                     pdFALSE

/**
 * @brief Mapping of FreeRTOS semaphore values to rtos value space
 *
 * @macro-title Semaphore value published by OSA layer
 *
 * @rtosSEMA_GIVE_NOBLOCK Defines macro to be used for setting semaphore block
 *                        time as 0.
 */
#define rtosSEMA_GIVE_NOBLOCK                   semGIVE_BLOCK_TIME

#endif
