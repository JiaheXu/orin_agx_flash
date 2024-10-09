/*
 * Copyright (c) 2015-2021 NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>                // for bool, true
#include <stddef.h>                 // for NULL
#include <stdint.h>                 // for uint32_t, uint64_t
                                    // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <artke_top.h>              // for TKE_TOP_SHARED_TKETSC0_0, TKE_TOP...

/* Late FSP headers */
#include <osa/rtos-task.h>          // for rtosSystemTickHandler
#include <debug/assert.h>           // for ASSERT, FSP__DEBUG__ASSERT_H
#include <error/common-errors.h>    // for E_SUCCESS
#include <irq/safe-irqs.h>          // for irq_safe_disable, irq_safe_enable, irq_safe_set_...
#include <misc/attributes.h>        // for FSP__MISC__ATTRIBUTES_H, UNUSED
#include <misc/bitops.h>            // for hilo_to_64, FSP__MISC__BITOPS_H
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <reg-access/reg-access.h>  // for readl_base_offset, writel_base_of...

/* Module-specific FSP headers */
#include <tke/sections-tke.h>       // Immune from CT_ASSERT protection
#include <tke/tke-tegra-priv.h>     // for tegra_tke_id, tegra_tke_conf, tke...
#include <tke/tke-tegra-regs.h>     // for TKE_TIMER_TMRCR_0, TKE_TIMER_TMRSR_0
#include <tke/tke-tegra.h>          // for FSP__TKE__TKE_TEGRA_H, tegra_tke_...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

static inline uint32_t
tegra_tke_timer_readl(const struct tegra_tke_id *id, uint32_t reg)
{
        return readl_base_offset(id->conf.base_addr, reg);
}

static inline void
tegra_tke_timer_writel(const struct tegra_tke_id *id, uint32_t val, uint32_t reg)
{
        writel_base_offset(val, id->conf.base_addr, reg);
}

void tegra_tke_timer_writel_non_static(const struct tegra_tke_id *id, uint32_t val, uint32_t reg)
{
        writel_base_offset(val, id->conf.base_addr, reg);
}

SECTION_TKE_TEXT void
tegra_tke_set_up_timer(struct tegra_tke_id *id, uint32_t clk_src_sel,
                       bool periodic, uint32_t divisor,
                       tegra_tke_timer_callback callback, void *data)
{
        uint32_t tmrcr;
        error_t  err;

        ASSERT(divisor != 0UL);
        id->callback = callback;
        id->data = data;

        /* select timer clock source */
        tegra_tke_timer_writel(id, clk_src_sel, TKE_TIMER_TMRCSSR_0);

        /* Disable timer, set cycle counter */
        tmrcr = (divisor - 1U) & TKE_TIMER_TMRCR_0_PTV;
        tegra_tke_timer_writel(id, tmrcr, TKE_TIMER_TMRCR_0);

        /* Clear timer interrupts. */
        tegra_tke_timer_writel(id, TKE_TIMER_TMRSR_0_INTR_CLR, TKE_TIMER_TMRSR_0);

        /* Enable timer and configure timer type. */
        tmrcr |= TKE_TIMER_TMRCR_0_EN;
        if (periodic) {
                tmrcr |= TKE_TIMER_TMRCR_0_PER;
        }
        tegra_tke_timer_writel(id, tmrcr, TKE_TIMER_TMRCR_0);

        if (id->conf.irq != TEGRA_TKE_NO_IRQ) {
                err = irq_safe_set_handler(id->conf.irq, tegra_tke_irq, (void *)id);
                ASSERT(err == E_SUCCESS);
                err = irq_safe_enable(id->conf.irq);
                ASSERT(err == E_SUCCESS);
        }
}

SECTION_TKE_TEXT void
tegra_tke_stop_timer(const struct tegra_tke_id *id)
{
        uint32_t tmrcr;
        error_t  err;

        if (id->conf.irq != TEGRA_TKE_NO_IRQ) {
                err = irq_safe_disable(id->conf.irq);
                ASSERT(err == E_SUCCESS);
        }

        tmrcr = tegra_tke_timer_readl(id, TKE_TIMER_TMRCR_0);
        tmrcr &= ~TKE_TIMER_TMRCR_0_EN;
        tegra_tke_timer_writel(id, tmrcr, TKE_TIMER_TMRCR_0);
}

SECTION_TKE_TEXT uint32_t
tegra_tke_get_pcv(const struct tegra_tke_id *id)
{
        uint32_t val;

        val = tegra_tke_timer_readl(id, TKE_TIMER_TMRSR_0);

        return val & TKE_TIMER_TMRSR_0_PCV;
}

SECTION_TKE_TEXT void
tegra_tke_enable_timer_irq(const struct tegra_tke_id *id)
{
        error_t  err;

        if (id->conf.irq != TEGRA_TKE_NO_IRQ) {
                err = irq_safe_enable(id->conf.irq);
                ASSERT(err == E_SUCCESS);
        }
}

SECTION_TKE_TEXT void
tegra_tke_disable_timer_irq(const struct tegra_tke_id *id)
{
        error_t  err;

        if (id->conf.irq != TEGRA_TKE_NO_IRQ) {
                err = irq_safe_disable(id->conf.irq);
                ASSERT(err == E_SUCCESS);
        }
}

SECTION_TKE_TEXT void
tegra_tke_clear_timer_irq(const struct tegra_tke_id *id)
{
        tegra_tke_timer_writel(id, TKE_TIMER_TMRSR_0_INTR_CLR,
                TKE_TIMER_TMRSR_0);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_TKE_TEXT void tegra_tke_irq(void *tke_id)
{
	INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
	const struct tegra_tke_id *id = (const void *)tke_id;

        tegra_tke_clear_timer_irq(id);

        if (id->callback != NULL) {
                id->callback(id->data);
        }
}

SECTION_TKE_TEXT void
tegra_tke_get_tsc(uint32_t *tsc_hi, uint32_t *tsc_lo)
{
        uint64_t tsc = tegra_tke_get_tsc64();

        INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
        *tsc_lo = LOW32(tsc);
        INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
        *tsc_hi = HI32(tsc);
}

/* TSC timestamp from 31.25 MHz clock (32 ns cycle) */
SECTION_TKE_TEXT uint32_t
tegra_tke_get_tsc32(void)
{
        return readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKETSC0_0);
}

SECTION_TKE_TEXT uint64_t
tegra_tke_get_tsc64(void)
{
        uint32_t hi0, lo, hi;

        /*
         * Here is how the code to handle rollover of the upper word:
         * Always read twice the MSW, pattern is read MSW, LSW, MSW.
         * If the two MSW are OK then it's done, otherwise read LSW a
         * second time, and keep the second set of both values
         */
        hi0 = readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKETSC1_0);
        lo = readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKETSC0_0);
        hi = readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKETSC1_0);

        if (hi0 != hi) {
                lo = readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKETSC0_0);
        }

        return hilo_to_64(hi, lo);
}

SECTION_TKE_TEXT uint32_t
tegra_tke_get_usec(void)
{
        return readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKEUSEC_0);
}

SECTION_TKE_TEXT uint32_t
tegra_tke_get_osc(void)
{
        return readl_base_offset(tke_top_base, TKE_TOP_SHARED_TKEOSC_0);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_TKE_TEXT static void tegra_tke_os_tick(void * data)
{
        UNUSED((data));

        rtosSystemTickHandler();
}

SECTION_TKE_INIT_TEXT void
tegra_tke_set_up_tick(struct tegra_tke_id *id, uint32_t clk_src,
                                uint32_t divisor)
{
        tegra_tke_set_up_timer(id, clk_src, true, divisor, tegra_tke_os_tick,
                                NULL);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
