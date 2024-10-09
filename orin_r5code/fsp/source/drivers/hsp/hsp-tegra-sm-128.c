/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <address_map_new.h>        // for NV_ADDRESS_MAP_TOP0_HSP_COMMON_SIZE
#include <arhsp_shrd_mbox.h>        // for HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0...

/* Late FSP headers */
#include <error/common-errors.h>    // for E_SUCCESS, error_t
#include <misc/bitops.h>            // for BIT, bit_number, FSP__MISC__BITOPS_H
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <reg-access/reg-access.h>  // for readl_base_offset, writel_base_of...

/* Module-specific FSP headers */
#include <hsp/hsp-errors.h>          // for E_HSP_ERR_NULL_PTR, E_HSP_ERR_NO_...
#include <hsp/hsp-tegra-defs.h>      // for HSP_COMMON_SIZE, HSP_DBELL_OFFSET...
#include <hsp/hsp-tegra-priv.h>      // for tegra_hsp_id, tegra_hsp_conf, teg...
#include <hsp/hsp-tegra.h>           // for tegra_hsp_suspend_ctx, FSP__HSP__...
#include <soc-common/hsp-tegra-sm.h> // for tegra_hsp_sm_produce_128...
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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_DEFS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_SM_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief HSP 128-bit shared mailbox registers offset
 *
 * Defines the HSP shared mailbox registers offset for the 128-bit variant.
 */
#define HSP_SM_TYPE1_TAG_OFFSET    (0x40UL)
#define HSP_SM_TYPE1_DATA0_OFFSET  (0x48UL)
#define HSP_SM_TYPE1_DATA1_OFFSET  (0x4cUL)
#define HSP_SM_TYPE1_DATA2_OFFSET  (0x50UL)
#define HSP_SM_TYPE1_DATA3_OFFSET  (0x54UL)


static inline uint32_t
hsp_sm_128_offset(uint32_t sm_index,
                  uint32_t offset)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    return (HSP_COMMON_SIZE + (HSP_SHRD_MBOX_OFFSET * sm_index) + offset);
}

static inline void
tegra_hsp_sm_write_128(const struct tegra_hsp_id *id,
                       uint32_t index,
                       hsp128_t const *value,
                       uint32_t tag)
{
    writel_base_offset(value->d0, id->conf.base_addr,
                       hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA0_OFFSET));
    writel_base_offset(value->d1, id->conf.base_addr,
                       hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA1_OFFSET));
    writel_base_offset(value->d2, id->conf.base_addr,
                       hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA2_OFFSET));
    writel_base_offset(value->d3, id->conf.base_addr,
                       hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA3_OFFSET));
    writel_base_offset(tag, id->conf.base_addr,
                       hsp_sm_128_offset(index, HSP_SM_TYPE1_TAG_OFFSET));
}

static inline void
tegra_hsp_sm_read_128(const struct tegra_hsp_id *id,
                      uint32_t index,
                      hsp128_t *data)
{
    data->d0 = readl_base_offset(id->conf.base_addr,
                          hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA0_OFFSET));
    data->d1 = readl_base_offset(id->conf.base_addr,
                          hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA1_OFFSET));
    data->d2 = readl_base_offset(id->conf.base_addr,
                          hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA2_OFFSET));
    data->d3 = readl_base_offset(id->conf.base_addr,
                          hsp_sm_128_offset(index, HSP_SM_TYPE1_DATA3_OFFSET));
    data->tag = readl_base_offset(id->conf.base_addr,
                          hsp_sm_128_offset(index, HSP_SM_TYPE1_TAG_OFFSET));
}


START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
SECTION_HSP_TEXT error_t
tegra_hsp_sm_produce_128(const struct tegra_hsp_id *id,
                         uint32_t sm,
                         hsp128_t const *value)
{
    error_t ret = E_SUCCESS;

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

    tegra_hsp_sm_write_128(id, sm, value, HSP_SHRD_MBOX_TAG_FIELD);

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_consume_128(const struct tegra_hsp_id *id,
                         uint32_t sm,
                         hsp128_t *data)
{
    error_t ret;

    ret = tegra_hsp_sm_peek_128(id, sm, data);

    if (ret == E_SUCCESS) {
        ret = tegra_hsp_sm_vacate_128(id, sm);
    }

    return ret;
}

SECTION_HSP_TEXT bool
tegra_hsp_sm_128_is_empty(const struct tegra_hsp_id *id,
                          uint32_t sm)
{
    bool     ret;
    hsp128_t data;

    if (id == NULL) {
        ret = false;
        goto out;
    }

    if (sm >= id->n_sm) {
        ret = false;
        goto out;
    }

    tegra_hsp_sm_read_128(id, sm, &data);

    ret = ((data.tag  & HSP_SHRD_MBOX_TAG_FIELD) == 0U);

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_peek_128(const struct tegra_hsp_id *id,
                      uint32_t sm,
                      hsp128_t *data)
{
    error_t  ret = E_SUCCESS;

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

    tegra_hsp_sm_read_128(id, sm, data);

    if ((data->tag & HSP_SHRD_MBOX_TAG_FIELD) == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_HSP_ERR_EMPTY_MBOX;
        goto out;
    }

out:
    return ret;
}

SECTION_HSP_TEXT error_t
tegra_hsp_sm_vacate_128(const struct tegra_hsp_id *id,
                        uint32_t sm)
{
    error_t  ret;
    hsp128_t data = {
        .d0 = 0UL,
        .d1 = 0UL,
        .d2 = 0UL,
        .d3 = 0UL,
    };

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

    tegra_hsp_sm_write_128(id, sm, &data, 0);
    ret = E_SUCCESS;

out:
    return ret;
}

SECTION_HSP_TEXT void
tegra_hsp_sm128_irq_handler(const struct tegra_hsp_id *id,
                            uint32_t sm)
{
    hsp_sm_data_t value;
    void          *cb_data;

    cb_data = id->sm[sm].opaque;
    tegra_hsp_sm_read_128(id, sm, &value.sm.data);
    id->sm[sm].callback(cb_data, &value);
}

SECTION_HSP_TEXT error_t
tegra_hsp_set_sm_type(struct tegra_hsp_id *id,
                      uint32_t sm,
                      uint32_t type)
{
    error_t  ret = E_SUCCESS;

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

    switch(type) {
        case HSP_SM_TYPE_32:
            id->sm[sm].wide = false;
            break;
        case HSP_SM_TYPE_128:
            id->sm[sm].wide = true;
            break;
        default:
            ret = E_HSP_ERR_NO_MBOX;
            break;
    }

out:
    return ret;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
