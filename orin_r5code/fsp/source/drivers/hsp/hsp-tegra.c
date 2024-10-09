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
#include <stdbool.h>                // for bool, false
#include <stddef.h>                 // for NULL
#include <stdint.h>                 // for uint32_t, uint8_t, UINT8_MAX, UIN...

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <arhsp_dbell.h>            // for HSP_DBELL_0_ENABLE_0, HSP_DBELL_0...
#include <arhsp_int.h>              // for HSP_INT_DIMENSIONING_0_nDB_SHIFT


/* Late FSP headers */
#include <osa/rtos-task.h>          // for rtosTaskEnterCritical, rtosTaskEx...
#include <osa/rtos-values.h>        // for rtosFALSE
#include <cpu/type-conversion.h>    // for fsp_c_v_ptr_to_v_ptr
#include <error/common-errors.h>    // for E_SUCCESS, error_t
#include <irq/safe-irqs.h>          // for in_interrupt, irq_safe_enable, irq_set...
#include <misc/attributes.h>        // for FSP__MISC__ATTRIBUTES_H, UNUSED
#include <misc/bitops.h>            // for BIT, bit_number, FSP__MISC__BITOPS_H
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <reg-access/reg-access.h>  // for readl_base_offset, writel_base_of...

/* Module-specific FSP headers */
#include <hsp/hsp-errors.h>          // for E_HSP_ERR_NULL_PTR, E_HSP_ERR_NO_...
#include <hsp/hsp-tegra-defs.h>      // for HSP_COMMON_SIZE, HSP_DBELL_OFFSET...
#include <hsp/hsp-tegra-priv.h>      // for tegra_hsp_id, tegra_hsp_conf, teg...
#include <hsp/hsp-tegra.h>           // for tegra_hsp_suspend_ctx, FSP__HSP__...
#include <hsp/sections-hsp.h>        // Immune from CT_ASSERT protection

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
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_DEFS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file hsp-tegra.c
 * @brief Implementation of Tegra HSP driver
 */

#define HSP_nSM_FIELD   (1U)
#define HSP_nSS_FIELD   (2U)
#define HSP_nAS_FIELD   (3U)
#define HSP_nDB_FIELD   (4U)
#define HSP_nSI_FIELD   (5U)

static inline uint32_t
hsp_ie_offset(uint32_t si_index)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return ((uint32_t)HSP_INT_IE + (uint32_t)(HSP_INT_IE_OFFSET * si_index));
}

static inline uint32_t
hsp_sm_ie_offset(uint32_t sm_index,
                 uint32_t offset)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return (HSP_COMMON_SIZE + (HSP_SHRD_MBOX_OFFSET * sm_index) + offset);
}

static inline uint32_t
hsp_sm_offset(uint32_t sm_index)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return (HSP_COMMON_SIZE + (HSP_SHRD_MBOX_OFFSET * sm_index));
}

static inline uint32_t
hsp_db_offset(uint32_t host,
              uint32_t offset)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return ((uint32_t)(HSP_DBELL_OFFSET * host) + offset);
}

static inline uint32_t
hsp_ss_offset(uint32_t ss_index,
              uint32_t offset)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return ((HSP_SHRD_SEM_OFFSET * ss_index) + offset);
}

static inline uint8_t
hsp_int_dimensioning_field(uint32_t reg_val, uint32_t field)
{
    uint32_t val = 0UL;

    switch(field) {
        case HSP_nSM_FIELD:
            val = (reg_val >> HSP_INT_DIMENSIONING_0_nSM_SHIFT) & 0xFU;
            break;
        case HSP_nSS_FIELD:
            val = (reg_val >> HSP_INT_DIMENSIONING_0_nSS_SHIFT) & 0xFU;
            break;
        case HSP_nAS_FIELD:
            val = (reg_val >> HSP_INT_DIMENSIONING_0_nAS_SHIFT) & 0xFU;
            break;
        case HSP_nDB_FIELD:
            val = (reg_val >> HSP_INT_DIMENSIONING_0_nDB_SHIFT) & 0xFU;
            break;
        case HSP_nSI_FIELD:
            val = (reg_val >> HSP_INT_DIMENSIONING_0_nSI_SHIFT) & 0xFU;
            break;
        default:
            /* Default case == NOP */
            break;
    }

    return (uint8_t)val;
}

static inline void
tegra_hsp_si_writel(const struct tegra_hsp_id *id,
                    uint32_t value)
{
    writel_base_offset(value, id->conf.base_addr,
                       hsp_ie_offset(id->conf.si_index));
}

static inline uint32_t
tegra_hsp_si_readl(const struct tegra_hsp_id *id)
{
    return readl_base_offset(id->conf.base_addr,
                             hsp_ie_offset(id->conf.si_index));
}

static inline void
tegra_hsp_sm_writel(const struct tegra_hsp_id *id,
                    uint32_t index,
                    uint32_t value)
{
    writel_base_offset(value, id->conf.base_addr, hsp_sm_offset(index));
}

static inline uint32_t
tegra_hsp_sm_readl(const struct tegra_hsp_id *id,
                   uint32_t index)
{
    return readl_base_offset(id->conf.base_addr, hsp_sm_offset(index));
}

static inline void tegra_hsp_sm_ie_writel(const struct tegra_hsp_id *id,
                                          uint32_t index,
                                          uint32_t ie_offset,
                                          uint32_t value)
{
    writel_base_offset(value, id->conf.base_addr,
                       hsp_sm_ie_offset(index, ie_offset));
}

static inline void
tegra_hsp_db_writel(const struct tegra_hsp_id *id,
                    uint32_t host,
                    uint32_t value,
                    uint32_t offset)
{
    writel_base_offset(value, id->db_base, hsp_db_offset(host, offset));
}

static inline uint32_t
tegra_hsp_db_readl(const struct tegra_hsp_id *id,
                   uint32_t host,
                   uint32_t offset)
{
    return readl_base_offset(id->db_base, hsp_db_offset(host, offset));
}

static inline void
tegra_hsp_db_self_writel(const struct tegra_hsp_id *id,
                         uint32_t value,
                         uint32_t offset)
{
    tegra_hsp_db_writel(id, id->conf.host, value, offset);
}

static inline uint32_t
tegra_hsp_db_self_readl(const struct tegra_hsp_id *id,
                        uint32_t offset)
{
    return tegra_hsp_db_readl(id, id->conf.host, offset);
}

static inline error_t tegra_hsp_ss_writel(const struct tegra_hsp_id *id,
                                          uint32_t index,
                                          uint32_t offset,
                                          uint32_t value)
{
    error_t ret;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (index >= id->n_ss) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_SHRD_SMPHR;
        goto out;
    }

    writel_base_offset(value, id->ss_base, hsp_ss_offset(index, offset));

    ret = E_SUCCESS;

out:
    return ret;
}

static inline error_t tegra_hsp_ss_readl(const struct tegra_hsp_id *id,
                                         uint32_t index,
                                         uint32_t offset,
                                         uint32_t *return_value)
{
    error_t ret;

    if ((id == NULL) || (return_value == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (index >= id->n_ss) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_SHRD_SMPHR;
        goto out;
    }

    *return_value = readl_base_offset(id->ss_base,
                                      hsp_ss_offset(index, offset));

    ret = E_SUCCESS;

out:
    return ret;
}

static inline uint32_t tegra_hsp_ir_readl(const struct tegra_hsp_id *id)
{
    return readl_base_offset(id->conf.base_addr, HSP_INT_IR_0);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_HSP_TEXT void tegra_hsp_db_irq_handler(void *data)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    const struct tegra_hsp_id *id = (const struct tegra_hsp_id *)data;
    uint32_t db_pending;
    rtosBool higher_prio_task_woken = (rtosBool) rtosFALSE;
    uint32_t source;

    db_pending = tegra_hsp_db_self_readl(id, HSP_DBELL_0_PENDING_0);
    tegra_hsp_db_self_writel(id, db_pending, HSP_DBELL_0_PENDING_0);

    while (db_pending != 0U) {
        source = bit_number(db_pending);
        db_pending &= ~BIT32_FN(source);
        id->db_callback(source, &higher_prio_task_woken);
    }

    rtosTaskYieldFromISR(higher_prio_task_woken);
}

SECTION_HSP_TEXT static error_t
tegra_hsp_sm_disable(struct tegra_hsp_id *id,
                     uint32_t sm,
                     uint32_t ie_offset)
{
    error_t ret;

    if (id->conf.si_index == (uint8_t)UINT8_MAX) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_INTR;
        goto out;
    }

    if (sm >= id->n_sm) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_MBOX;
        goto out;
    }

    if (!in_interrupt()) {
        rtosTaskEnterCritical();
    }

    if (ie_offset != 0U) {
        tegra_hsp_sm_ie_writel(id, sm, ie_offset, 0U);
    } else {
        id->sie_mask &= ~BIT(sm + HSP_INT_IE_0_mbox_empty_enable_SHIFT);
        id->sie_mask &= ~BIT(sm + HSP_INT_IE_0_mbox_full_enable_SHIFT);

        if (tegra_hsp_si_readl(id) != id->sie_mask) {
            tegra_hsp_si_writel(id, id->sie_mask);
        }
    }

    /* FIXME: sync interrupt handler here */
    id->sm[sm].callback = NULL;

    if (!in_interrupt()) {
        rtosTaskExitCritical();
    }

    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT static void
tegra_hsp_sm32_irq_handler(const struct tegra_hsp_id *id,
                           uint32_t sm)
{
    hsp_sm_data_t value;
    void          *cb_data;

    cb_data = id->sm[sm].opaque;
    value.sm.val = tegra_hsp_sm_readl(id, sm);
    id->sm[sm].callback(cb_data, &value);
}

SECTION_HSP_TEXT static void
tegra_hsp_sm_irq_handler(struct tegra_hsp_id *id,
                         uint32_t mask)
{
    uint32_t sm_mask = mask;
    uint32_t i;

    while (sm_mask != 0U) {
        i = bit_number(sm_mask);

        if (id->sm[i].callback != NULL) {
            if (id->sm[i].wide) {
                tegra_hsp_sm128_irq_handler(id, i);
            } else {
                tegra_hsp_sm32_irq_handler(id, i);
            }
        } else {
            (void)tegra_hsp_sm_disable(id, i, 0U);
        }

        sm_mask &= ~BIT(i);
    }
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_HSP_TEXT void tegra_hsp_irq_handler(void *data)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    struct tegra_hsp_id *id = (struct tegra_hsp_id *)fsp_c_v_ptr_to_v_ptr(data);
    uint32_t ie = tegra_hsp_si_readl(id);
    uint32_t ir = tegra_hsp_ir_readl(id);
    uint32_t full_mask;
    uint32_t empty_mask;

    if (ie != id->sie_mask) {
        tegra_hsp_si_writel(id, id->sie_mask);
    }

    ir &= id->sie_mask;

    full_mask = (uint32_t)(ir & HSP_INT_IR_0_mbox_full_assert_FIELD)
                                >> HSP_INT_IR_0_mbox_full_asserted_SHIFT;

    tegra_hsp_sm_irq_handler(id, full_mask);

    empty_mask = (ir & HSP_INT_IR_0_mbox_empty_asserted_FIELD)
                               >> HSP_INT_IR_0_mbox_empty_asserted_SHIFT;

    tegra_hsp_sm_irq_handler(id, empty_mask);
}

SECTION_HSP_INIT_TEXT error_t
tegra_hsp_init(struct tegra_hsp_id *id)
{
    uint32_t val;
    uint32_t i;
    error_t ret = E_SUCCESS;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->inited) {
        goto out;
    }

    val = readl_base_offset(id->conf.base_addr, HSP_INT_DIMENSIONING_0);
    id->n_sm = hsp_int_dimensioning_field(val, HSP_nSM_FIELD);
    id->n_ss = hsp_int_dimensioning_field(val, HSP_nSS_FIELD);
    id->n_as = hsp_int_dimensioning_field(val, HSP_nAS_FIELD);
    id->n_db = hsp_int_dimensioning_field(val, HSP_nDB_FIELD);
    id->n_si = hsp_int_dimensioning_field(val, HSP_nSI_FIELD);

    if (id->n_ss > 0U) {
        INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        id->ss_base = id->conf.base_addr + ((uint32_t)(1U + ((uint32_t)id->n_sm >> 1U)) << 16U);
    }

    if (id->n_db > 0U) {
        INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        id->db_base = id->conf.base_addr +
                ((uint32_t)(1U + ((uint32_t)id->n_sm >> 1U) + id->n_ss + id->n_as) << 16U);
    }

    /* default to 32-bit type shared mailboxes */
    if (id->n_sm > 0U) {
        for (i = 0UL; i < id->n_sm; i += 1UL) {
            id->sm[i].wide = false;
        }
    }

    if ((id->conf.sh_irq != UINT32_MAX) && (id->conf.si_index != (uint8_t)UINT8_MAX)) {
        /* Disable all shared interrupt sources */
        tegra_hsp_si_writel(id, id->sie_mask);
        ret = irq_safe_set_handler((uint32_t)id->conf.sh_irq, tegra_hsp_irq_handler, (void*)id);
        if (ret == E_SUCCESS) {
            ret = irq_safe_enable((uint32_t)id->conf.sh_irq);
        }
    }

    if (ret == E_SUCCESS) {
        id->inited = true;
    }

out:
    return ret;
}

SECTION_HSP_INIT_TEXT error_t
tegra_hsp_db_init(struct tegra_hsp_id *id,
                  uint32_t enabled_masters,
                  tegra_hsp_db_callback callback)
{
    error_t ret;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (callback == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_INVALID_CALLBACK;
        goto out;
    }

    UNUSED((tegra_hsp_init(id)));

    if (id->n_db == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_DB;
        goto out;
    }
    id->db_callback = callback;

    ret = irq_safe_set_handler((uint32_t)id->conf.db_irq, tegra_hsp_db_irq_handler, (void*)id);
    if (ret == E_SUCCESS) {
        ret = irq_safe_enable((uint32_t)id->conf.db_irq);
    }

    /* Enable the interrupt from <enabled_master> */
    if (ret == E_SUCCESS) {
        tegra_hsp_db_self_writel(id, enabled_masters, HSP_DBELL_0_ENABLE_0);
    }

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_produce(const struct tegra_hsp_id *id,
                     uint32_t sm,
                     uint32_t value)
{
    error_t ret;
    uint32_t reg_val;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (sm >= id->n_sm) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_MBOX;
        goto out;
    }

    reg_val = HSP_SHRD_MBOX_TAG_FIELD | value;
#if defined(HW_BUG_200395605) && (HW_BUG_200395605 == 1)
    if (reg_val == DEAD1001) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_INVALID_DATA;
        goto out;
    }
#endif

    tegra_hsp_sm_writel(id, sm, reg_val);

    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_consume(const struct tegra_hsp_id *id,
                     uint32_t sm,
                     uint32_t *data)
{
    error_t ret;

    ret = tegra_hsp_sm_peek(id, sm, data);

    if (ret == E_SUCCESS) {
        ret = tegra_hsp_sm_vacate(id, sm);
    }

    return ret;
}

SECTION_HSP_TEXT bool
tegra_hsp_sm_is_empty(const struct tegra_hsp_id *id,
                      uint32_t sm)
{
    bool ret;
    uint32_t v;

    if (id == NULL) {
        ret = false;
        goto out;
    }

    if (sm >= id->n_sm) {
        ret = false;
        goto out;
    }

    v = tegra_hsp_sm_readl(id, sm);

    ret = ((v & HSP_SHRD_MBOX_TAG_FIELD) == 0U);
out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_peek(const struct tegra_hsp_id *id,
                  uint32_t sm,
                  uint32_t *data)
{
    error_t ret;
    uint32_t v;

    if ((id == NULL) || (data == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (sm >= id->n_sm) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_MBOX;
        goto out;
    }

    v = tegra_hsp_sm_readl(id, sm);

    if ((v & HSP_SHRD_MBOX_TAG_FIELD) == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_EMPTY_MBOX;
        goto out;
    }

    *data = (v & HSP_SHRD_MBOX_DATA_FIELD);
    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_vacate(const struct tegra_hsp_id *id,
                    uint32_t sm)
{
    error_t ret;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (sm >= id->n_sm) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_MBOX;
        goto out;
    }

    tegra_hsp_sm_writel(id, sm, 0);
    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT static void
tegra_hsp_sm_enable(struct tegra_hsp_id *id,
                    uint32_t sm,
                    uint32_t ie_offset,
                    uint32_t setbit,
                    uint32_t clrbit,
                    tegra_hsp_sm_callback cb,
                    void *data)
{
    if (!in_interrupt()) {
        rtosTaskEnterCritical();
    }

    id->sm[sm].callback = cb;
    id->sm[sm].opaque = data;
    id->sie_mask |= BIT32_FN(setbit);
    id->sie_mask &= ~BIT32_FN(clrbit);

    if (id->sie_mask != tegra_hsp_si_readl(id)) {
        tegra_hsp_si_writel(id, id->sie_mask);
    }

    tegra_hsp_sm_ie_writel(id, sm, ie_offset, 1U);

    if (!in_interrupt()) {
        rtosTaskExitCritical();
    }
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_full_enable(struct tegra_hsp_id *id,
                         uint32_t sm,
                         tegra_hsp_sm_callback cb,
                         void *data)
{
    error_t  ret;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->conf.si_index == (uint8_t)UINT8_MAX) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_INTR;
        goto out;
    }

    if (sm >= id->n_sm) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_MBOX;
        goto out;
    }

    tegra_hsp_sm_enable(id, sm, HSP_SHRD_MBOX_FULL_IE_OFFSET,
                (uint32_t)HSP_INT_IE_0_mbox_full_enable_SHIFT + sm,
                (uint32_t)HSP_INT_IE_0_mbox_empty_enable_SHIFT + sm,
                cb, data);

   ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_full_disable(struct tegra_hsp_id *id,
                          uint32_t sm)
{
    INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
    return (id == NULL) ? E_HSP_ERR_NULL_PTR :
                tegra_hsp_sm_disable(id, sm, HSP_SHRD_MBOX_FULL_IE_OFFSET);
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_empty_enable(struct tegra_hsp_id *id,
                          uint32_t sm,
                          tegra_hsp_sm_callback cb,
                          void *data)
{
    error_t ret;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->conf.si_index == (uint8_t)UINT8_MAX) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_INTR;
        goto out;
    }

    if (sm >= id->n_sm) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_MBOX;
        goto out;
    }

    tegra_hsp_sm_enable(id, sm, HSP_SHRD_MBOX_EMPTY_IE_OFFSET,
                (uint32_t)HSP_INT_IE_0_mbox_empty_enable_SHIFT + sm,
                (uint32_t)HSP_INT_IE_0_mbox_full_enable_SHIFT + sm,
                cb, data);

   ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_empty_disable(struct tegra_hsp_id *id,
                           uint32_t sm)
{
    INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
    return (id == NULL) ? E_HSP_ERR_NULL_PTR :
                tegra_hsp_sm_disable(id, sm, HSP_SHRD_MBOX_EMPTY_IE_OFFSET);
}

SECTION_HSP_TEXT error_t
tegra_hsp_db_ring(const struct tegra_hsp_id *id,
                  uint32_t target)
{
    error_t ret;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->db_base == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_DB;
        goto out;
    }

    tegra_hsp_db_writel(id, target, 1, HSP_DBELL_0_TRIGGER_0);

    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_db_enable_master(const struct tegra_hsp_id *id,
                           uint32_t enabled_masters)
{
    error_t ret;
    uint32_t enable;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->db_base == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_DB;
        goto out;
    }

    if (!in_interrupt()) {
        rtosTaskEnterCritical();
    }

    enable = tegra_hsp_db_self_readl(id, HSP_DBELL_0_ENABLE_0);
    enable |= enabled_masters;
    tegra_hsp_db_self_writel(id, enable, HSP_DBELL_0_ENABLE_0);

    if (!in_interrupt()) {
        rtosTaskExitCritical();
    }

    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_db_disable_master(const struct tegra_hsp_id *id,
                            uint32_t disabled_masters)
{
    error_t ret;
    uint32_t enable;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->db_base == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NO_DB;
        goto out;
    }

    if (!in_interrupt()) {
        rtosTaskEnterCritical();
    }

    enable = tegra_hsp_db_self_readl(id, HSP_DBELL_0_ENABLE_0);
    enable &= ~disabled_masters;
    tegra_hsp_db_self_writel(id, enable, HSP_DBELL_0_ENABLE_0);

    if (!in_interrupt()) {
        rtosTaskExitCritical();
    }

    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_suspend(const struct tegra_hsp_id *id,
                  struct tegra_hsp_suspend_ctx *sctx)
{
    error_t ret;

    if ((id == NULL) || (sctx == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if ((id->conf.sh_irq != UINT32_MAX) &&
                    (id->conf.si_index != (uint8_t)UINT8_MAX)) {

        /* This statement only execute if (sctx != NULL).  Normally, we would put in a check here
         * to ensure that condition.  However, in this case, that check isn't needed due to the
         * conditional statements above.  The 'if' clause above ((id == NULL) || (sctx == NULL))
         * guarantees that (sctx != NULL), therefore, this check is not needed below.
         * Fixes MISRA Rule 14.3 violation. */
        sctx->si_enable = tegra_hsp_si_readl(id);

    }
    if (id->conf.db_irq != UINT32_MAX) {

        /* This statement only execute if (sctx != NULL).  Normally, we would put in a check here
         * to ensure that condition.  However, in this case, that check isn't needed due to the
         * conditional statements above.  The 'if' clause above ((id == NULL) || (sctx == NULL))
         * guarantees that (sctx != NULL), therefore, this check is not needed below.
         * Fixes MISRA Rule 14.3 violation. */
        sctx->db_enable = tegra_hsp_db_self_readl(id, HSP_DBELL_0_ENABLE_0);
    }
    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_resume(const struct tegra_hsp_id *id,
                 const struct tegra_hsp_suspend_ctx *sctx)
{
    error_t ret = E_SUCCESS;

    if ((id == NULL) || (sctx == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_NULL_PTR;
        goto out;
    }

    if (id->conf.db_irq != UINT32_MAX) {

        /* This statement only execute if (sctx != NULL).  Normally, we would put in a check here
         * to ensure that condition.  However, in this case, that check isn't needed due to the
         * conditional statements above.  The 'if' clause above ((id == NULL) || (sctx == NULL))
         * guarantees that (sctx != NULL), therefore, this check is not needed below.
         * Fixes MISRA Rule 14.3 violation. */
        tegra_hsp_db_self_writel(id, sctx->db_enable, HSP_DBELL_0_ENABLE_0);
        ret = irq_safe_enable((uint32_t)id->conf.db_irq);
    }
    if ((ret == E_SUCCESS) && (id->conf.sh_irq != UINT32_MAX) &&
                    (id->conf.si_index != (uint8_t)UINT8_MAX)) {

        /* This statement only execute if (sctx != NULL).  Normally, we would put in a check here
         * to ensure that condition.  However, in this case, that check isn't needed due to the
         * conditional statements above.  The 'if' clause above ((id == NULL) || (sctx == NULL))
         * guarantees that (sctx != NULL), therefore, this check is not needed below.
         * Fixes MISRA Rule 14.3 violation. */
        tegra_hsp_si_writel(id, sctx->si_enable);

        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
        ret = irq_safe_enable((uint32_t)id->conf.sh_irq);
    }

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_ss_read(const struct tegra_hsp_id *id,
                  uint32_t index,
                  uint32_t *value)
{
    return tegra_hsp_ss_readl(id, index, HSP_SHRD_SEM_STA, value);
}

SECTION_HSP_TEXT error_t
tegra_hsp_ss_set(const struct tegra_hsp_id *id,
                 uint32_t index,
                 uint32_t data)
{
    error_t ret;

    /**
     * parameter check for HW bug 200395605 (HSP IAS 7.3.1 SW WAR)
     */
#if defined(HW_BUG_200395605) && (HW_BUG_200395605 == 1)
    if ((data & HSP_SS_FORBIDDEN_BITS) != 0U) {
        ret = E_HSP_ERR_INVALID_DATA;
        goto out;
    }
#endif

    ret = tegra_hsp_ss_writel(id, index, HSP_SHRD_SEM_SET, data);

#if defined(HW_BUG_200395605) && (HW_BUG_200395605 == 1)
out:
#endif
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_ss_clear(const struct tegra_hsp_id * id,
                   uint32_t index,
                   uint32_t data)
{
    return tegra_hsp_ss_writel(id, index, HSP_SHRD_SEM_CLR, data);
}

SECTION_HSP_TEXT uint32_t
tegra_hsp_sm_count(const struct tegra_hsp_id *id)
{
    uint32_t ret;
    uint32_t val;

    if (id == NULL) {
        ret = 0U;
        goto out;
    }

    val = readl_base_offset(id->conf.base_addr, HSP_INT_DIMENSIONING_0);

    ret = hsp_int_dimensioning_field(val, HSP_nSM_FIELD);

out:
    return ret;
}

SECTION_HSP_TEXT uint32_t
tegra_hsp_ss_count(const struct tegra_hsp_id *id)
{
    uint32_t ret;
    uint32_t val;

    if (id == NULL) {
        ret = 0U;
        goto out;
    }

    val = readl_base_offset(id->conf.base_addr, HSP_INT_DIMENSIONING_0);

    ret = hsp_int_dimensioning_field(val, HSP_nSS_FIELD);

out:
    return ret;
}

SECTION_HSP_TEXT uint32_t
tegra_hsp_db_count(const struct tegra_hsp_id *id)
{
    uint32_t ret;
    uint32_t val;

    if (id == NULL) {
        ret = 0U;
        goto out;
    }

    val = readl_base_offset(id->conf.base_addr, HSP_INT_DIMENSIONING_0);

    ret = hsp_int_dimensioning_field(val, HSP_nDB_FIELD);

out:
    return ret;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
