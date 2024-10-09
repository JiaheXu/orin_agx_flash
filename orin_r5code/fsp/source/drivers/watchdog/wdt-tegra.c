/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>                   // for false, true
#include <stdint.h>                    // for uint32_t, uint8_t
#include <stddef.h>                    // IWYU pragma: keep
                                       // for NULL
#include <string.h>                    // for offsetof
                                       // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>            // for CT_ASSERT
#include <soc-common/hw-const.h>       /* Must appear before any hwinc files */

/* Hardware headers */
#include <artke_top.h>                 // for BASE_ADDRESS_TKE_TOP_WDT0, TKE...

/* Late FSP headers */
#include <osa/rtos-task.h>             // for rtosTaskEnterCritical, rtosTas...
#include <error/common-errors.h>       // for E_SUCCESS, error_t
#include <irq/safe-irqs.h>             // for in_interrupt, irq_safe_disable, irq...
#include <misc/attributes.h>           // for FSP_ALIGNOF
#include <misc/macros.h>               // for CONTAINER_OF
#include <misc/nvrm_drf.h>             // for NV_DRF_NUM, FSP__MISC__NVRM_DRF_H
#include <reg-access/reg-access.h>     // for readl_base_offset, writel_base...
#include <debug/assert.h>              // for ASSERT

/* Module-specific FSP headers */
#include <watchdog/sections-wdt.h>     // Immune from CT_ASSERT protection
#include <watchdog/watchdog-errors.h>  // for E_WDT_ERR_IRQ_CONFIG, E_WDT_ER...
#include <watchdog/wdt-tegra-priv.h>   // for tegra_wdt, tegra_wdt_id, FSP__...
#include <watchdog/tegra-safe-wdt.h>   // for tegra_safe_wdt_conf, ...


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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WATCHDOG_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WDT_TEGRA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__TEGRA_SAFE_WDT_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define TEGRA_WDT_CONFIG_0              U32_C(TKE_TOP_WDT0_WDTCR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_STATUS_0              U32_C(TKE_TOP_WDT0_WDTSR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_COMMAND_0             U32_C(TKE_TOP_WDT0_WDTCMDR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_WINDOW_CONFIG_0       U32_C(TKE_TOP_WDT0_WDTFWCR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_UNLOCK_PATTERN_0      U32_C(TKE_TOP_WDT0_WDTUR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_UNLOCK_PATTERN_VALUE  0xC45AUL
#define TEGRA_WDT_UNLOCK_CRC32          0xEDB88320UL

static inline const struct tegra_wdt *
tegra_wdt_from_id(const struct tegra_wdt_id *id)
{
    uint32_t wdtptr = fsp_c_wdtid_ptr_to_u32(id) - offsetof(struct tegra_wdt, id);
    return fsp_u32_to_c_wdt_ptr(wdtptr);
}

static inline uint32_t
tegra_wdt_readl(const struct tegra_wdt_id *id,
                uint32_t reg)
{
    return readl_base_offset(id->base_addr, reg);
}

static inline void
tegra_wdt_writel(const struct tegra_wdt_id *id,
                 uint32_t val, uint32_t reg)
{
    writel_base_offset(val, id->base_addr, reg);
}

static SECTION_WDT_TEXT void
tegra_wdt_unlock(const struct tegra_wdt_id *id)
{
    const struct tegra_wdt      *wdt_dev        = tegra_wdt_from_id(id);
    uint32_t                    val;

    if (wdt_dev->challenge_response) {
        /*
         * The Challenge is a LFSR value, present in the Unlock
         * register.
         *
         * The Response is the next value of the LFSR, that needs to
         * be written in the Unlock register.
         *
         * The LFSR uses the classical CRC-32 polynomial operating
         * in shift right (reversed) direction, that is the next
         * value is calculated as
         *
         * LFSR = (LFSR >> 1) ^ ((LFSR & 0x1) ? 0xEDB88320 : 0)
         */
        val = tegra_wdt_readl(id, TEGRA_WDT_UNLOCK_PATTERN_0);
        val = (val >> 1) ^ (((val & 1UL) != 0UL) ? TEGRA_WDT_UNLOCK_CRC32 : 0UL);
    } else {
        val = TEGRA_WDT_UNLOCK_PATTERN_VALUE;
    }

    tegra_wdt_writel(id, val, TEGRA_WDT_UNLOCK_PATTERN_0);
}

static SECTION_WDT_TEXT void
tegra_wdt_ack_internal(const struct tegra_wdt_id *id)
{
    tegra_wdt_unlock(id);
    UNUSED(tegra_safe_wdt_start(id));
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")
SECTION_WDT_TEXT error_t
tegra_safe_wdt_start(const struct tegra_wdt_id *id)
{
    error_t                     rc              = E_SUCCESS;
    const struct tegra_wdt      *wdt_dev;
    uint32_t                    val;

    if (id == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    wdt_dev = tegra_wdt_from_id(id);

    /*
     * When written to 1b, enable the counter operation, load the counter
     * with Period and starts downcounting, resets the expiration count to
     * 0 and clears all status flags.
     */
    val = NV_DRF_NUM(TKE_TOP, WDT0_WDTCMDR, StartCounter, 1UL);
    tegra_wdt_writel(id, val, TEGRA_WDT_COMMAND_0);

    if (wdt_dev->irq_callback != NULL) {
        rc = irq_safe_enable(id->irq);
        if (rc != E_SUCCESS) {
            goto out;
        }
    }
    if (wdt_dev->fiq_callback != NULL) {
        rc = irq_safe_enable(id->fiq);
        if ((rc != E_SUCCESS) && (wdt_dev->irq_callback != NULL)) {
            ASSERT(irq_safe_disable(id->irq) == E_SUCCESS);
        }
    }

  out:
    return rc;
}

SECTION_WDT_ERROR_TEXT error_t
tegra_safe_wdt_stop(const struct tegra_wdt_id *id)
{
    error_t                     rc              = E_SUCCESS;
    const struct tegra_wdt      *wdt_dev;
    uint32_t                    val;

    if (id == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    wdt_dev = tegra_wdt_from_id(id);

    /*
     * Only working if the unlock register (WDTUR{w}) has been
     * programmed before with the correct pattern. Writing to the command
     * register always clears the unlock register.
     * If written DisableCounter to 1b, while WDT counter is enabled and the
     * unlock register contains the unlock pattern, the watchdog transitions
     * back to disabled.
     */

    if (!in_interrupt()) {
        rtosTaskEnterCritical();
    }

    tegra_wdt_unlock(id);

    val = NV_DRF_NUM(TKE_TOP, WDT0_WDTCMDR, DisableCounter, 1UL);
    tegra_wdt_writel(id, val, TEGRA_WDT_COMMAND_0);

    if (wdt_dev->irq_callback != NULL) {
        rc = irq_safe_disable(id->irq);
        if (rc != E_SUCCESS) {
            goto exit;
        }
    }
    if (wdt_dev->fiq_callback != NULL) {
        rc = irq_safe_disable(id->fiq);
        if ((rc != E_SUCCESS) && (wdt_dev->irq_callback != NULL)) {
            ASSERT(irq_safe_enable(id->irq) == E_SUCCESS);
        }
    }

exit:
    if (!in_interrupt()) {
        rtosTaskExitCritical();
    }

  out:
    return rc;
}

SECTION_WDT_ERROR_TEXT error_t
tegra_safe_wdt_stop_from_isr(const struct tegra_wdt_id *id)
{
    return tegra_safe_wdt_stop(id);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WDT_ERROR_TEXT void tegra_safe_wdt_irq(void *data)
{
    error_t                     rc;
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    const struct tegra_wdt_id   *id             = data;
    const struct tegra_wdt      *wdt_dev;
    uint32_t                    status          = 0;

    ASSERT(id != NULL);

    wdt_dev = tegra_wdt_from_id(id);

    rc = tegra_safe_wdt_read_status(id, &status);
    ASSERT(rc == E_SUCCESS);

    if (wdt_dev->irq_callback != NULL) {
        if (wdt_dev->irq_ack_by_callback) {
            rc = irq_safe_disable(id->irq);
            ASSERT(rc == E_SUCCESS);
        } else {
            tegra_wdt_ack_internal(id);
        }
        wdt_dev->irq_callback(status, wdt_dev->irq_data);
    }
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WDT_ERROR_TEXT void tegra_safe_wdt_fiq(void *data)
{
    error_t                     rc;
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    const struct tegra_wdt_id   *id             = data;
    const struct tegra_wdt      *wdt_dev;
    uint32_t                    status          = 0;

    ASSERT(id != NULL);

    wdt_dev = tegra_wdt_from_id(id);

    rc = tegra_safe_wdt_read_status(id, &status);
    ASSERT(rc == E_SUCCESS);

    if (wdt_dev->fiq_callback != NULL) {
        if (wdt_dev->fiq_ack_by_callback) {
            rc = irq_safe_disable(id->fiq);
            ASSERT(rc == E_SUCCESS);
        } else {
            tegra_wdt_ack_internal(id);
        }
        wdt_dev->fiq_callback(status, wdt_dev->fiq_data);
    }
}

SECTION_WDT_TEXT error_t
tegra_safe_wdt_read_status(const struct tegra_wdt_id *id,
                           uint32_t *status)
{
    error_t             rc              = E_SUCCESS;

    if (id == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    *status = tegra_wdt_readl(id, TEGRA_WDT_STATUS_0);

  out:
    return rc;
}

SECTION_WDT_TEXT error_t
tegra_safe_wdt_ack(const struct tegra_wdt_id *id)
{
    error_t                     rc              = E_SUCCESS;
    const struct tegra_wdt      *wdt_dev;

    if (id == NULL) {
        rc = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    wdt_dev = tegra_wdt_from_id(id);

    if (!in_interrupt()) {
        rtosTaskEnterCritical();
    }

    tegra_wdt_ack_internal(id);

    if (wdt_dev->irq_ack_by_callback) {
        rc = irq_safe_enable(id->irq);
        if (rc != E_SUCCESS) {
            goto exit;
        }
    }
    if (wdt_dev->fiq_ack_by_callback) {
        rc = irq_safe_enable(id->fiq);
        if ((rc != E_SUCCESS) && wdt_dev->irq_ack_by_callback) {
            ASSERT(irq_safe_disable(id->irq) == E_SUCCESS);
        }
    }

exit:
    if (!in_interrupt()) {
        rtosTaskExitCritical();
    }

  out:
    return rc;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_WDT_INIT_TEXT error_t tegra_safe_wdt_setup(struct tegra_wdt_id *id, const struct tegra_wdt_conf *conf)
{
    struct tegra_wdt    *wdt_dev;
    uint32_t            val;
    error_t             err = E_SUCCESS;

    if ((id == NULL)
        || (conf == NULL)) {
        err = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    wdt_dev = fsp_c_wdt_ptr_to_wdt_ptr(tegra_wdt_from_id(id));

    if ((conf->tmrsrc & ~TKE_TOP_WDT0_WDTCR_0_TimerSource_DEFAULT_MASK) != 0U) {
        err = E_WDT_ERR_TMRSRC_INVALID;
        goto out;
    }

    if ((conf->err_threshold & ~TKE_TOP_WDT0_WDTCR_0_ErrorThreshold_DEFAULT_MASK) != 0UL) {
        err = E_WDT_ERR_THRESHOLD_INVALID;
        goto out;
    }

    if (conf->irq_en && (conf->irq_callback == NULL)) {
        err = E_WDT_ERR_IRQ_CONFIG;
        goto out;
    }

    if (conf->fiq_en && (conf->fiq_callback == NULL)) {
        err = E_WDT_ERR_IRQ_CONFIG;
        goto out;
    }

    wdt_dev->irq_ack_by_callback = conf->irq_en && conf->irq_ack_by_callback;
    wdt_dev->irq_callback = conf->irq_en ? conf->irq_callback : NULL;
    wdt_dev->irq_data = conf->irq_data;
    wdt_dev->fiq_ack_by_callback = conf->fiq_en && conf->fiq_ack_by_callback;
    wdt_dev->fiq_callback = conf->fiq_en ? conf->fiq_callback : NULL;
    wdt_dev->fiq_data = conf->fiq_data;

    val =
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, TimerSource, conf->tmrsrc) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, Period, conf->period) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, LocalInterruptEnable, conf->irq_en ? 1UL : 0UL) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, LocalFIQEnable, conf->fiq_en ? 1UL : 0UL) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, RemoteInterruptEnable, conf->remoteIrq_en ? 1UL : 0UL) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, SystemDebugResetEnable, conf->sys_dbg_rst_en ? 1UL : 0UL) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, SystemPOResetEnable, conf->sys_por_rst_en ? 1UL : 0UL) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, TscReferenceEnable, conf->tsc_ref_en ? 1UL : 0UL) |
        NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, ErrorThreshold, conf->err_threshold);

    tegra_wdt_writel(id, val, TEGRA_WDT_CONFIG_0);

    /* Explicitly disable windowed operation mode */
    err = tegra_safe_disable_wdt_windowed_operation(id);
    if (err != E_SUCCESS) {
        goto out;
    }

    if ((val & NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, ChallengeResponseEnable, 1UL)) != 0UL) {
        wdt_dev->challenge_response = true;
    } else {
        wdt_dev->challenge_response = false;
    }

    if (conf->irq_en) {
        err = irq_safe_set_handler(id->irq, tegra_safe_wdt_irq, (void *)id);
        if (err != E_SUCCESS) {
            goto out;
        }
    }

    if (conf->fiq_en) {
        err = irq_safe_set_handler(id->fiq, tegra_safe_wdt_fiq, (void *)id);
        if (err != E_SUCCESS) {
            goto out;
        }
    }

out:
    return err;
}

SECTION_WDT_TEXT error_t
tegra_safe_enable_wdt_windowed_operation(const struct tegra_wdt_id *id,
                                         uint8_t start_level,
                                         uint8_t start_count)
{
    uint32_t val;
    error_t  err = E_SUCCESS;

    if (id == NULL) {
        err = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    val = NV_DRF_NUM(TKE_TOP, WDT0_WDTFWCR, StartLevel, start_level) |
          NV_DRF_NUM(TKE_TOP, WDT0_WDTFWCR, StartCount, start_count);
    tegra_wdt_writel(id, val, TEGRA_WDT_WINDOW_CONFIG_0);

    val = tegra_wdt_readl(id, TEGRA_WDT_CONFIG_0);
    val |= NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, WindowedOperationEnable, 0x1UL);
    tegra_wdt_writel(id, val, TEGRA_WDT_CONFIG_0);

out:
    return err;
}

SECTION_WDT_TEXT error_t
tegra_safe_disable_wdt_windowed_operation(const struct tegra_wdt_id *id)
{
    uint32_t val;
    error_t err = E_SUCCESS;

    if (id == NULL) {
        err = E_WDT_ERR_NULL_PTR;
        goto out;
    }

    val = tegra_wdt_readl(id, TEGRA_WDT_CONFIG_0);
    val &= ~NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, WindowedOperationEnable, 0x1UL);
    tegra_wdt_writel(id, val, TEGRA_WDT_CONFIG_0);

out:
    return err;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
