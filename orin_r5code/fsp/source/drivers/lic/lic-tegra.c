/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>                // for bool, false
#include <stddef.h>                 // for NULL
#include <stdint.h>                 // for uint32_t, uint8_t, UINT8_MAX, UIN...
                                    // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>        // for NV_ADDRESS_MAP_LIC_BASE
#include <arintr_ctlr.h>            // for INTR_CTLR_CHANNEL0_SLICE0_IER_0...

/* Late FSP headers */
#include <error/common-errors.h>    // for E_SUCCESS, error_t
#include <misc/bitops.h>            // for BIT, bit_number, FSP__MISC__BITOPS_H
#include <reg-access/reg-access.h>  // for readl_base_offset, writel_base_of...
#include <irq/irqs.h>               // for in_interrupt, irq_enable, irq_set...
#include <irq/safe-irqs.h>          // for in_interrupt, enter_critical, exi...
#include <misc/macros.h>            // for ARRAY_SIZE,...

/* Module-specific FSP headers */
#include <lic/lic-errors.h>         // for E_LIC_ERR_NULL_PTR, E_LIC_ERR_NO_...
#include <lic/lic-tegra-priv.h>     // for tegra_lic_id, ...
#include <lic/lic-tegra.h>          // for tegra_lic_int, ...
#include <lic/sections-lic.h>       // for SECTION_LIC_TEXT, ...

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx")
    CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__IRQ__IRQS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__LIC__LIC_ERRORS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__LIC__LIC_TEGRA_PRIV_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__LIC__LIC_TEGRA_H, "Header file missing or invalid.")

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")
#define LIC_IER_REG         INTR_CTLR_CHANNEL0_SLICE0_IER_0
#define LIC_IER_SET_REG     INTR_CTLR_CHANNEL0_SLICE0_IER_SET_0
#define LIC_IER_CLR_REG     INTR_CTLR_CHANNEL0_SLICE0_IER_CLR_0
#define LIC_IDR_REG         INTR_CTLR_CHANNEL0_SLICE0_IDR_0
#define LIC_IEP_CLASS_REG   INTR_CTLR_CHANNEL0_SLICE0_IEP_CLASS_0
#define LIC_VIRQ_REG        INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0
#define LIC_VFIQ_REG        INTR_CTLR_CHANNEL0_SLICE0_VFIQ_0

#define LIC_CHAN_OFFSET \
    (INTR_CTLR_CHANNEL1_SLICE0_VIRQ_0 - INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0)
#define LIC_SLICE_OFFSET \
    (INTR_CTLR_CHANNEL0_SLICE1_VIRQ_0 - INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")
START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")

static inline uint32_t
lic_reg(uint32_t offset)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return (NV_ADDRESS_MAP_LIC_BASE + offset);
}

static inline uint32_t
lic_chan_slice_reg(uint8_t chan, uint8_t slice, uint32_t offset)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return ((chan * LIC_CHAN_OFFSET) + (slice * LIC_SLICE_OFFSET) + offset);
}

static inline uint32_t
lic_slice_reg(uint32_t reg, uint8_t slice)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return reg + (slice * LIC_SLICE_OFFSET);
}

static inline uint32_t
lic_ier_set(uint8_t chan, uint8_t slice)
{

    return lic_reg(lic_chan_slice_reg(chan, slice, LIC_IER_SET_REG));
}

static inline uint32_t
lic_ier_clr(uint8_t chan, uint8_t slice)
{

    return lic_reg(lic_chan_slice_reg(chan, slice, LIC_IER_CLR_REG));
}

static inline uint32_t
lic_iep_class(uint8_t chan, uint8_t slice)
{
    return lic_reg(lic_chan_slice_reg(chan, slice, LIC_IEP_CLASS_REG));
}

static inline uint32_t
lic_virq(uint8_t chan, uint8_t slice)
{
    return lic_reg(lic_chan_slice_reg(chan, slice, LIC_VIRQ_REG));
}

static inline uint32_t
lic_vfiq(uint8_t chan, uint8_t slice)
{
    return lic_reg(lic_chan_slice_reg(chan, slice, LIC_VFIQ_REG));
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx", MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
SECTION_LIC_TEXT void tegra_lic_irq_handler(void *data)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    const lic_irq_context_t   *ctx = (lic_irq_context_t *)data;
    const struct tegra_lic_id *id  = ctx->id;
    uint8_t                   chan = ctx->chan;
    uint32_t                  virq[LIC_MAX_SLICES];
    uint8_t                   slice = 0;
    uint32_t                  slice_map = 0;
    uint32_t                  lic_irq, irq;
    uint32_t                  mask;
    uint32_t                  irqs_left = ctx->irqs_set;
    uint32_t                  reg;
    irq_callback_fn           isr_handler;

    reg = ctx->is_vfiq ? lic_vfiq(chan, slice) : lic_virq(chan, slice);

    while (irqs_left != 0UL) {
        lic_irq = bit_number(irqs_left);
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, Example-4, DR: SWE-FSP-045-SWSADR.docx");
        irq = lic_irq + id->conf.lic_irq_base;
        slice = id->lic_map[lic_irq].slice;
        mask = BIT32_FN(id->lic_map[lic_irq].bit);
        isr_handler = id->lic_map[lic_irq].routine;

        if ((slice_map & BIT32_FN(slice)) == 0UL) {
            virq[slice] = readl(lic_slice_reg(reg, slice));
            slice_map = BIT32_FN(slice);
        }

        if ((virq[slice] & mask) != 0UL) {
            if (isr_handler != NULL) {
                isr_handler(id->lic_map[lic_irq].cb_data);
            } else {
                irq_handler(irq);
            }
        }

        irqs_left &= ~BIT32_FN(lic_irq);
    }

}

SECTION_LIC_TEXT error_t
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
tegra_lic_irq_enable(const struct tegra_lic_id *id,
                     uint32_t irq)
{
    error_t  ret = E_SUCCESS;
    uint32_t lic_irq;
    uint32_t irq_line;
    uint8_t  chan;

    if (id == NULL) {
        ret = E_LIC_NULL_PTR;
        goto out;
    }

    if (irq < id->conf.lic_irq_base) {
        ret = E_LIC_INVALID_IRQ;
        goto out;
    }

    lic_irq = irq - id->conf.lic_irq_base;
    if (lic_irq >= id->lic_map_size) {
        ret = E_LIC_INVALID_IRQ;
        goto out;
    }

    irq_line = id->lic_map[lic_irq].local_irq - (uint32_t)id->conf.local_irq_base;
    if (irq_line > id->num_lic_irq_lines) {
        ret = E_LIC_INVALID_IRQ;
        goto out;
    }
    chan = id->irq_contexts[irq_line]->chan;

    if (!in_interrupt()) {
        enter_critical();
    }

    writel(BIT32_FN(id->lic_map[lic_irq].bit),
           lic_ier_set(chan, id->lic_map[lic_irq].slice));

    if (!in_interrupt()) {
        exit_critical();
    }

out:
    return ret;
}

SECTION_LIC_TEXT error_t
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
tegra_lic_irq_disable(const struct tegra_lic_id *id,
                      uint32_t irq)
{
    error_t  ret = E_SUCCESS;
    uint32_t lic_irq;
    uint32_t irq_line;
    uint8_t  chan;

    if (id == NULL) {
        ret = E_LIC_NULL_PTR;
        goto out;
    }

    if (irq < id->conf.lic_irq_base) {
        ret = E_LIC_INVALID_IRQ;
        goto out;
    }

    lic_irq = irq - id->conf.lic_irq_base;
    if (lic_irq >= id->lic_map_size) {
        ret = E_LIC_INVALID_IRQ;
        goto out;
    }

    irq_line = id->lic_map[lic_irq].local_irq - (uint32_t)id->conf.local_irq_base;
    if (irq_line > id->num_lic_irq_lines) {
        ret = E_LIC_INVALID_IRQ;
        goto out;
    }
    chan = id->irq_contexts[irq_line]->chan;

    if (!in_interrupt()) {
        enter_critical();
    }

    writel(BIT32_FN(id->lic_map[lic_irq].bit),
           lic_ier_clr(chan, id->lic_map[lic_irq].slice));

    if (!in_interrupt()) {
        exit_critical();
    }

out:
    return ret;
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")
static error_t
tegra_lic_validate_irq_context(const struct tegra_lic_id *id)
{
    error_t  ret = E_SUCCESS;
    uint32_t i;
    uint32_t chan;
    uint8_t  max_chan;

    for (i = 0UL; i < id->num_lic_irq_lines; i += 1UL) {
        if (id->irq_contexts[i] == NULL) {
            ret = E_LIC_INVALID_PARAM;
            goto out;
        }

        chan  = id->irq_contexts[i]->chan;
        max_chan = id->conf.base_chan;
        max_chan += id->conf.num_chans;
        if ((chan < id->conf.base_chan) || (chan >= max_chan)) {
            ret = E_LIC_INVALID_PARAM;
            goto out;
        }

        id->irq_contexts[i]->irqs_set = 0UL;
    }

out:
    return ret;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")

static void
tegra_lic_configure_lic_map(const struct tegra_lic_id *id)
{
    const lic_irq_context_t *irq_ctx;
    uint32_t                i;
    uint8_t                 chan;
    uint32_t                iep;
    bool                    is_vfiq;
    uint16_t                irq_line;
    uint8_t                 slice;

    for (i = 0UL; i < id->lic_map_size; i += 1UL) {
        slice = id->lic_map[i].slice;
        irq_line = id->lic_map[i].local_irq;
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, Example-4, DR: SWE-FSP-045-SWSADR.docx");
        irq_line -= id->conf.local_irq_base;
        irq_ctx = id->irq_contexts[irq_line];
        chan  = irq_ctx->chan;
        is_vfiq = irq_ctx->is_vfiq;

        iep = readl(lic_iep_class(chan, slice));

        if (is_vfiq) {
            iep |= BIT32_FN(id->lic_map[i].bit);
        }
        writel(iep, lic_iep_class(chan, slice));

        writel(BIT32_FN(id->lic_map[i].bit), lic_ier_clr(chan, slice));

        id->irq_contexts[irq_line]->irqs_set |= BIT(i);
    }

}

static void
tegra_lic_set_irq_handlers(const struct tegra_lic_id *id)
{
    lic_irq_context_t *irq_ctx;
    uint32_t          i;
    uint32_t          local_irq;

    for (i = 0UL; i < id->num_lic_irq_lines; i += 1UL) {
        irq_ctx = id->irq_contexts[i];
        local_irq = id->conf.local_irq_base + i;

        irq_set_handler(local_irq, tegra_lic_irq_handler, (void*)irq_ctx);
        irq_enable(local_irq);
    }
}

SECTION_LIC_INIT_TEXT error_t
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
tegra_lic_init(struct tegra_lic_id *id,
               const lic_irq_t *lic_map,
               const uint32_t lic_map_size)
{
    error_t  ret = E_SUCCESS;

    if (id == NULL) {
        ret = E_LIC_NULL_PTR;
        goto out;
    }

    if (id->conf.num_chans == 0UL) {
        ret = E_LIC_NO_CHANS;
        goto out;
    }

    if ((lic_map == NULL) || (lic_map_size == 0UL)) {
        ret = E_LIC_NO_MAP;
        goto out;
    }
    id->lic_map = lic_map;
    id->lic_map_size = lic_map_size;

    if (id->num_lic_irq_lines == 0UL) {
        ret = E_LIC_NO_IRQ;
        goto out;
    }

    if (id->num_lic_irq_lines != (id->conf.num_chans * 2UL)) {
        ret = E_LIC_INVALID_PARAM;
        goto out;
    }

    ret = tegra_lic_validate_irq_context(id);
    if (ret != E_SUCCESS) {
        goto out;
    }

    tegra_lic_configure_lic_map(id);

    tegra_lic_set_irq_handlers(id);

out:
    return ret;
}

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")
