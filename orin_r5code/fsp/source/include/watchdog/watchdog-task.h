/* Copyright (c) 2019-2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef WATCHDOG__WATCHDOG_TASK_H
#define WATCHDOG__WATCHDOG_TASK_H
#define FSP__WATCHDOG__WATCHDOG_TASK_H                  1

/**
 * @file watchdog-task.h
 * @brief Definitions for use of watchdog task
 */

/* Compiler headers */
#include <stdint.h>                   // for uint64_t, uint32_t
#include <stdbool.h>                  // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>           // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>            // for rtosTaskHandle
#include <misc/macros.h>              // for START_RFD_BLOCK, END_RFD_BLOCK, ...

/* Module-specific FSP headers */

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/*
 * Watchdog Task's stack depth.
 *
 * In general this shouldn't need adjusting, but care must be taken
 * in the case of the callbacks
 */
#define WATCHDOG_STACK_DEPTH    512

/**
 * @brief Watchdog Task Initialization Parameters
 *
 * Structure passed to the WatchdogTaskInit() function, this is used to
 * initialize various aspects of the watchdog task.  This allows an
 * application firmware to specify various attributes of the watchdog
 * task without having to expose various internal apsects of the watchdog
 * task itself.
 */
typedef struct {
    uint8_t             hw_period;      // milliseconds for WDT timeout
    bool                wdt_enabled;    // enable WDT timer
    bool                watchdog_mpu;   // use per task MPU region
    uint8_t             pad;            // pad to align next field
    rtosTick            pet_period;     // milliseconds between pet attempts
    rtosPriority        task_priority;  // priority of the watchdog task
    uint32_t            stack_priority; // MPU priority for watchdog's stack
    uint64_t            min_timeout;    // minimum value for a task timeout
    uint64_t            max_timeout;    // maximum value for a task timeout
    uint64_t            default_timeout;// default value for a task timeout
} WatchdogInitParameters;

/**
 * @brief Task Parameters for Watchdog Task
 *
 * The task parameters that are to be used when the application firmware
 * creates the watchdog task.  The values in this structure should not be
 * modified by the application firmware.
 */
extern rtosTaskParameters       WatchdogTaskParams;

/**
 * @brief Initialize the watchdog task
 *
 * This function must be called prior to creating the watchdog task
 * as it sets up various aspects of the task.  This function does not
 * create the watchdog task.  It is the responsibility of the application
 * firmware to create the watchdog task after this function has been called.
 *
 * @pre None
 *
 * @param[in] parameters  pointer to a WatchdogInitParameters structure that
 *                        contains various initialization parameters specified
 *                        by the application.  This is distinct from the
 *                        rtosTaskParameters structure which is used by the
 *                        RTOS to create and start the task
 *
 * @return Error Code
 *
 * @retval E_SUCCESS     initialization was performed successfully
 * @retval E_RTOS_EVENT_GROUP_ALREADY_IN_USE Input event group already in use.
 * @retval E_RTOS_NULL_PARAMETER_SUPPLIED    Null Input parameter
 * @retval E_RTOS_EVENT_GROUP_ALREADY_IN_USE Event handle supplied is valid and
 *                                           must be deleted before creating a
 *                                           new group
 * @retval E_WDT_ERR_IRQ_CONFIG         The watchdog's IRQ or FIQ was
 *                                      enabled but no corresponding
 *                                      callback function was specified
 * @retval E_WDT_ERR_TMRSRC_INVALID     value specified for tmrsrc was invalid
 * @retval E_WDT_ERR_THRESHOLD_INVALID  value specified for threshold was
 *                                      invalid
 * @retval rtosFAIL      initialization could not be performed
 */
rtosError WatchdogTaskInit(const void * const parameters);

#endif
