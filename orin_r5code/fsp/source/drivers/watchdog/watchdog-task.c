/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stdbool.h>                       // for false, true
#include <stddef.h>                        // for NULL
#include <stdint.h>                        // for uintptr_t, uint32_t, uint8_t

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-errors.h>               // for E_RTOS_NEXT_UNBLOCK_TIME_E...
#include <osa/rtos-event-group.h>          // for rtosEventGroupSetBitsFromISR
#include <osa/rtos-task.h>                 // for rtosTaskYieldFromISR
#include <osa/rtos-values.h>               // for rtosPASS, rtosFAIL, rtosFALSE
#include <armv7-mpu.h>                     // for MPU_NTASK_REGIONS
#include <cpu/armv7-mpu.h>                 // for R5_DRACR_AP_PRIV_RW, R5_DR...
#include <cpu/armv7-mpu-safertos.h>        // for mpuParameters_t, R5MPU_CHE...
#include <cpu/sections-armv7-mpu.h>        // Immune from CT_ASSERT protection
#include <debug/abort-codes.h>             // for ABORT_CODE
#include <debug/assert.h>                  // for ASSERT, tegra_abort
#include <error/common-errors.h>           // for E_SUCCESS, error_t
#include <hw-config/vic-irqs.h>            // for INTERRUPT_WDTFIQ, INTERRUP...
#include <irq/safe-irqs.h>                 // for irq_safe_enable
#include <logger-config.h>                 // for LOG_LEVEL_INFO_ENABLE, LOG...
#include <logger/logger.h>                 // for LOG_INFO, LOG_WARN, LOG_ERROR
#include <logger/logger-data.h>            // for LOG_FL_ERROR, LOG_FL_NONE
#include <misc/attributes.h>               // for UNUSED, ALIGN, FSP__MISC__...
#include <misc/bitops.h>                   // for BIT
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, ...

/* Module-specific FSP headers */
#include <sections/sections-strings.h>     // Immune from CT_ASSERT protection
#include <watchdog/sections-watchdog.h>    // Immune from CT_ASSERT protection
#include <watchdog/watchdog-logging.h>     // for WATCHDOG_LOG_INIT, WATCHDO...
#include <watchdog/watchdog-task.h>        // for WatchdogInitParameters
#include <watchdog/watchdog.h>             // for WatchdogCheckStatus, Watch...
#include <watchdog/wdt-tegra-priv.h>       // for tegra_wdt
#include <watchdog/tegra-safe-wdt.h>       // for tegra_safe_wdt_conf, ...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_EVENT_GROUP_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_SAFERTOS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HW_CONFIG__VIC_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_DATA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_LOGGING_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WDT_TEGRA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__TEGRA_SAFE_WDT_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_9, "Approval: Bug 200532001, DR: SWE-FSP-016-SWSADR.docx")
static rtosEventGroupHandle     WatchdogSignal         SECTION_WATCHDOG_DATA;
static rtosEventGroup           WatchdogSignalMemory   SECTION_WATCHDOG_DATA;

static struct tegra_wdt_conf    wdt_conf               SECTION_WATCHDOG_INIT_DATA;

extern struct tegra_wdt         tegra_wdt_instance     SECTION_WATCHDOG_DATA;

static rtosTick                 WatchdogPetPeriod      SECTION_WATCHDOG_DATA;

static uint8_t  WatchdogTaskStack[WATCHDOG_STACK_DEPTH] SECTION_WATCHDOG_PRIV_STACKS
                                                ALIGN(WATCHDOG_STACK_DEPTH);
static uint8_t  WatchdogTCB[rtosTCB_OVERHEAD_BYTES]     SECTION_WATCHDOG_DATA;

static const char       WatchdogTask_name[] SECTION_STRINGS_RODATA = "Watchdog";

static void WatchdogTaskCode(void *parameters);

rtosTaskParameters WatchdogTaskParams SECTION_WATCHDOG_INIT_RODATA = {
    .pvTaskCode         = WatchdogTaskCode,
    .pcTaskName         = &WatchdogTask_name[0],
    .pxTCB              = WatchdogTCB,
    .pcStackBuffer      = WatchdogTaskStack,
    .uxStackDepthBytes  = WATCHDOG_STACK_DEPTH,
    .pvParameters       = NULL,
    .uxPriority         = 0,            // to be set by WatchdogTaskInit()
    .pvObject           = NULL,
    .xUsingFPU          = rtosFALSE,
    .pxMPUParameters    = NULL,         // to be set by WatchdogTaskInit()
    .uxPrivilegeLevel   = RTOS_SYSTEM_MODE,
};

/*
 * Watchdog Task's MPU Regions
 *
 * This defines the watchdog task's stack region.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_11_4, "Approval: Bug 200532002, DR: SWE-FSP-063-SWSADR.docx")
static mpuParameters_t watchdog_task_mpu_regions[MPU_NTASK_REGIONS] SECTION_MPU_DATA =
{
    [0]  = {
        .flags          = R5MPU_FL_VALID | R5MPU_FL_SIZE,
        .index          = 0,                    // filled in by WatchdogTaskInit
        .permissions    = R5MPU_CHECK_PRIV_RW,
        .access         = R5_DRACR_NORMAL_CACHED
                        | R5_DRACR_XN | R5_DRACR_AP_PRIV_RW,
        .base           = (uintptr_t)WatchdogTaskStack,
        .size           = WATCHDOG_STACK_DEPTH,
    },
};
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_9, "Approval: Bug 200532001, DR: SWE-FSP-016-SWSADR.docx",
              MISRA, DEVIATE, Rule_11_4, "Approval: Bug 200532002, DR: SWE-FSP-063-SWSADR.docx")

/*
 * Events that can be signalled
 */
#define WATCHDOG_LVL1_TIMEOUT   BIT(0)          // WDT FIQ interrupt fired
#define WATCHDOG_LVL2_TIMEOUT   BIT(1)          // WDT IRQ interrupt fired
#define WATCHDOG_PET_TIMER      BIT(2)          // time to pet the WDT

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_ERROR_TEXT static void Watchdog_fiq_callback(uint32_t status, void *data)
{
    rtosError   rc;
    rtosBool    wake;

    UNUSED(status);
    UNUSED(data);

    LOG_ERROR(WATCHDOG_LOG_LVL1_TIMEOUT, LOG_FL_ERROR, 0);

    /*
     * Indicate that the WDT FIQ occurred
     */
    rc = rtosEventGroupSetBitsFromISR(WatchdogSignal, WATCHDOG_LVL1_TIMEOUT,
                                      &wake);
    ASSERT(rc == rtosPASS);
    /*
     * XXX: TODO
     * Better error handling
     */

    rtosTaskYieldFromISR(wake);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_ERROR_TEXT static void Watchdog_irq_callback(uint32_t status, void *data)
{
    rtosError   rc;
    rtosBool    wake;

    UNUSED(status);
    UNUSED(data);

    LOG_INFO(WATCHDOG_LOG_LVL2_TIMEOUT, LOG_FL_NONE, 0);

    /*
     * Indicate the the WDT IRQ occurred
     */
    rc = rtosEventGroupSetBitsFromISR(WatchdogSignal, WATCHDOG_LVL2_TIMEOUT,
                                      &wake);
    ASSERT(rc == rtosPASS);
    /*
     * XXX: TODO
     * Better error handling
     */

    rtosTaskYieldFromISR(wake);
}

SECTION_WATCHDOG_TEXT WEAK error_t
WatchdogCallbackApplicationCheck(void)
{
    return E_SUCCESS;
}

SECTION_WATCHDOG_INIT_TEXT rtosError
WatchdogTaskInit(const void * const parameters)
{
    rtosError                   rc      = rtosPASS;
    error_t                     err;

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx", MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR: SWE-FSP-024-SWSADR.docx")
    WatchdogInitParameters      *parms  = (WatchdogInitParameters *)parameters;

    LOG_INFO(WATCHDOG_LOG_INIT, LOG_FL_START, 0);

    ASSERT(parms != NULL);

    WatchdogPetPeriod = parms->pet_period;
    WatchdogTaskParams.uxPriority = parms->task_priority;

    /*
     * Determine if the watchdog task should setup an MPU
     * region for its stack.
     */
    if (parms->watchdog_mpu) {
        watchdog_task_mpu_regions[0].index = parms->stack_priority;
        WatchdogTaskParams.pxMPUParameters = watchdog_task_mpu_regions;
    }

    rc = rtosEventGroupCreate(&WatchdogSignalMemory, &WatchdogSignal);
    if (rc != rtosPASS) {
        goto exit;
    }

    /*
     * Initialize the watchdog framework
     */
    err = WatchdogInit(parms->min_timeout,
                       parms->max_timeout,
                       parms->default_timeout);
    if (err != E_SUCCESS) {
        rc = rtosFAIL;
        goto exit;
    }

    /*
     * Set up the Watchdog HW
     */

    /*
     * Initializing the Watchdog timer Configuration register
     * NvTimer0 as source which is set for RTOS 1ms periodic timer.
     * Period set for 15 of 1msec timer
     * Enabled all irqs
     * Threshold set to default value
     */
    wdt_conf.tmrsrc = 0x0;
    wdt_conf.period = parms->hw_period;
    wdt_conf.irq_en = true;
    wdt_conf.fiq_en = true;
    wdt_conf.remoteIrq_en = true;
    wdt_conf.sys_dbg_rst_en = false;
    wdt_conf.sys_por_rst_en = false;
    wdt_conf.tsc_ref_en = false;
    wdt_conf.err_threshold = 0x7;
    wdt_conf.irq_callback = Watchdog_irq_callback;
    wdt_conf.irq_data = NULL;
    wdt_conf.fiq_callback = Watchdog_fiq_callback;
    wdt_conf.fiq_data = NULL;

    err = tegra_safe_wdt_setup(&tegra_wdt_instance.id, &wdt_conf);
    if (err != E_SUCCESS) {
        goto exit;
    }

    /*
     * Make sure that the watchdog timer will be enabled.
     */
    if (!parms->wdt_enabled) {
        goto exit;
    }

    /* Start the watchdog timer*/
    err = tegra_safe_wdt_start(&tegra_wdt_instance.id);
    if (err != E_SUCCESS) {
        goto exit;
    }

    err = irq_safe_enable(INTERRUPT_WDTFIQ);
    ASSERT(err == E_SUCCESS);
    err = irq_safe_enable(INTERRUPT_WDTIRQ);
    ASSERT(err == E_SUCCESS);

  exit:
    LOG_INFO(WATCHDOG_LOG_INIT, LOG_FL_END, rc);
    return rc;
}

/*
 * RTOS task for managing the watchdog timer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WATCHDOG_TEXT NO_RETURN static void WatchdogTaskCode(void *parameters)
{
    rtosError           rc;
    rtosEventBits       events;
    rtosEventBits       wait_events;

    UNUSED(parameters);
    while (true) {
        wait_events = (WATCHDOG_LVL1_TIMEOUT | WATCHDOG_LVL2_TIMEOUT);

        LOG_INFO(WATCHDOG_LOG_SLEEP, LOG_FL_NONE, 0);

        /*
         * Wait for the WDT to report a timeout or a specific amount
         * of time has elapsed.
         */
        rc = rtosEventGroupWaitBits(WatchdogSignal, wait_events,
                                    false, false, &events,
                                    WatchdogPetPeriod);

        LOG_INFO(WATCHDOG_LOG_WAKEUP, LOG_FL_NONE, rc, events);

        if (rc != rtosPASS) {
            if (rc == E_RTOS_NEXT_UNBLOCK_TIME_EXPIRED) {
                events |= WATCHDOG_PET_TIMER;
            } else {
                continue;
            }
        }

        /*
         * If the event was caused by FIQ or IRQ from the
         * watchdog hardware, abort to the ccplex and let
         * it deal with it.
         */
        if ((events & (WATCHDOG_LVL1_TIMEOUT | WATCHDOG_LVL2_TIMEOUT)) != 0UL) {
            tegra_abort(ABORT_CODE(WATCHDOG, 0U), events);
        }

        /*
         * So far eveything is OK.  Go check the tasks to make
         * sure that they've made progress
         */
        rc = WatchdogCheckStatus();
        if (rc != E_SUCCESS) {
            LOG_WARN(WATCHDOG_LOG_TIMEOUT, LOG_FL_ERROR, rc);
            continue;
        }

        /*
         * Check to see if the application has other checks
         * to perform
         */
        if (WatchdogCallbackApplicationCheck() != E_SUCCESS) {
            LOG_WARN(WATCHDOG_LOG_APPLICATION, LOG_FL_ERROR, 0UL);
            continue;
        }

        /*
         * Reset the watchdog timer
         */
        rc = tegra_safe_wdt_start(&tegra_wdt_instance.id);
        if (rc != E_SUCCESS) {
            LOG_WARN(WATCHDOG_LOG_TIMEOUT, LOG_FL_ERROR, rc);
            continue;
        }
    }
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

