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
#include <stdint.h>                 // for uint32_t, uint8_t, uint64_t
#include <stdbool.h>                // for bool, true, false
#include <string.h>                 // for memset, NULL

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    // Must appear before any hwinc files

/* Hardware headers */
#include <araps_ast.h>              // for APS_AST_CONTROL_0, APS_AST_REGION...

/* Late FSP headers */
#include <cpu/type-conversion.h>    // for fsp_c_v_ptr_to_uptr, FSP__MISC_...
#include <debug/assert.h>           // for ASSERT, FSP__DEBUG__ASSERT_H
#include <error/common-errors.h>    // for E_SUCCESS, error_t
#include <misc/bitops.h>            // for hilo_to_64, HI32, LOW32, FSP__MIS...
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <reg-access/reg-access.h>  // for readl_base_offset, writel_base_of...

/* Module-specific FSP headers */
#include <ast/ast-errors.h>         // for E_AST_INVALID_INPUT_PARAMETERS
#include <ast/sections-ast.h>       // Immune from CT_ASSERT protection
#include <ast/tegra-ast.h>          // for ast_tegra_id, tegra_ast_region_map

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
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__AST__AST_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__AST__TEGRA_AST_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define LO_SHIFT APS_AST_REGION_0_SLAVE_BASE_LO_0_SlvBase_SHIFT

#define MAX_VMINDEX            MK_U32_CONST(15)
#define AST_INSTANCE_R5_INDEX  MK_U32_CONST(0)
#define AST_INSTANCE_DMA_INDEX MK_U32_CONST(1)

/* Special values returned from VMINDEX search */
#define VMINDEX_PHYSICAL_STREAM_ID (MAX_VMINDEX + MK_U32_CONST(1))

static inline uint32_t ast_region_base(uint32_t ast_base,
                                       uint8_t region)
{
    const uint32_t region_stride = APS_AST_REGION_1_MASK_LO_0 -
                                   APS_AST_REGION_0_MASK_LO_0;
    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    return ast_base + ((uint32_t)region * region_stride);
}

static inline uint32_t streamid_ctl_reg(uint32_t ast_base,
                                        uint8_t vmindex)
{
    uint32_t ctl_stride = APS_AST_STREAMID_CTL_1 - APS_AST_STREAMID_CTL_0;

    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    return ast_base + APS_AST_STREAMID_CTL_0 + ((uint32_t)vmindex * ctl_stride);
}

SECTION_AST_TEXT
static bool ast_is_global_locked(uint32_t ast_base)
{
    bool status = false;

    uint32_t val = readl_base_offset(ast_base, APS_AST_CONTROL_0);

    if (((val & APS_AST_CONTROL_0_Lock_FIELD) >>
        APS_AST_CONTROL_0_Lock_SHIFT) == MK_U32_CONST(0x01)) {
        status = true;
    }
    return status;
}

SECTION_AST_TEXT
bool tegra_ast_is_global_locked(void)
{
    return ast_is_global_locked(ast_tegra_id[AST_INSTANCE_R5_INDEX]) ||
           ast_is_global_locked(ast_tegra_id[AST_INSTANCE_DMA_INDEX]);
}

SECTION_AST_TEXT
static error_t ast_get_vmindex(uint32_t ast_base,
                               uint8_t stream_id,
                               bool *return_is_enabled,
                               uint8_t *VMIndex)
{
    uint32_t mask = APS_AST_STREAMID_CTL_0_READ_MASK;
    uint32_t expected_val;
    uint8_t  free_vmindex = (uint8_t)MAX_VMINDEX + 2U;
    error_t  err = E_SUCCESS;

    expected_val = (uint32_t)(stream_id & APS_AST_STREAMID_CTL_StreamID_DEFAULT_MASK) <<
                    APS_AST_STREAMID_CTL_StreamID_SHIFT;
    expected_val |= APS_AST_STREAMID_CTL_Enable_FIELD;

    for (uint8_t tmpIndex = 0; tmpIndex <= MAX_VMINDEX; tmpIndex++) {
        uint32_t val = readl(streamid_ctl_reg(ast_base, tmpIndex)) & mask;

        if (val == expected_val) {
            *return_is_enabled = true;
            free_vmindex = tmpIndex;
            goto out;
        }

        /*
         * Select last free tmpIndex in order to avoid
         * collision w/ kernel and bootloader
         */
        if ((val & APS_AST_STREAMID_CTL_Enable_FIELD) != MK_U32_CONST(0x1)) {
            free_vmindex = tmpIndex;
        }
    }

    *return_is_enabled = false;
out:
    *VMIndex = free_vmindex;
    if(free_vmindex > VMINDEX_PHYSICAL_STREAM_ID) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_VMINDEX;
    }
    return err;
}

SECTION_AST_TEXT
static error_t ast_set_streamid(uint32_t ast_base,
                                uint8_t vmindex,
                                uint8_t stream_id)
{
    uint32_t sid_ctl;
    uint32_t val;
    error_t  err = E_SUCCESS;

    /* ast_get_vmindex ensures that ast_base != 0 */
    sid_ctl = APS_AST_STREAMID_CTL_Enable_FIELD;
    sid_ctl |= ((uint32_t)(stream_id & APS_AST_STREAMID_CTL_StreamID_DEFAULT_MASK) <<
               APS_AST_STREAMID_CTL_StreamID_SHIFT);
    writel(sid_ctl, streamid_ctl_reg(ast_base, vmindex));

    /* Check that write was not blocked */
    val = readl(streamid_ctl_reg(ast_base, vmindex));
    if ((val & APS_AST_STREAMID_CTL_READ_MASK) != sid_ctl) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_VMINDEX;
    }

    return err;
}

SECTION_AST_TEXT
static error_t ast_add_streamid(uint32_t ast_base,
                                uint8_t stream_id)
{
    uint8_t VMIndex;
    error_t err = E_SUCCESS;
    bool    already_enabled;

    err = ast_get_vmindex(ast_base, stream_id, &already_enabled, &VMIndex);
    if (err != E_SUCCESS) {
        goto out;
    }

    if (already_enabled) {
        goto out;
    }

    INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
    err = ast_set_streamid(ast_base, VMIndex, stream_id);
out:
    return err;
}

SECTION_AST_TEXT
error_t tegra_ast_add_streamid(uint8_t stream_id, uint8_t instance_mask)
{
    error_t err = E_SUCCESS;
    uint8_t ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;

    if (ast_instance == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if ((instance_mask & AST_INSTANCE_R5) != 0U) {
        err = ast_add_streamid(ast_tegra_id[AST_INSTANCE_R5_INDEX], stream_id);
        if (err != E_SUCCESS) {
            goto out;
        }
    }

    if ((instance_mask & AST_INSTANCE_DMA) != 0U) {
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
        err = ast_add_streamid(ast_tegra_id[AST_INSTANCE_DMA_INDEX], stream_id);
    }
out:
    return err;
}

SECTION_AST_TEXT
static void ast_enable_region(uint32_t ast_base,
                              uint8_t region, uint8_t vmindex,
                              iova master, uint64_t slave,
                              uint64_t size)
{
    uint32_t region_base = ast_region_base(ast_base, region);
    uint32_t hi, lo;
    uint32_t snoopEnable = APS_AST_REGION_0_CONTROL_0_Snoop_DEFAULT_MASK <<
                           APS_AST_REGION_0_CONTROL_0_Snoop_SHIFT;
    uint32_t physicalStreamEnable = APS_AST_REGION_0_CONTROL_0_Physical_DEFAULT_MASK <<
                                    APS_AST_REGION_0_CONTROL_0_Physical_SHIFT;
    uint32_t tmpIndex = (uint32_t)(vmindex & APS_AST_REGION_0_CONTROL_0_VMIndex_DEFAULT_MASK) <<
                        APS_AST_REGION_0_CONTROL_0_VMIndex_SHIFT;
    uint32_t control = 0U;

    if (vmindex == VMINDEX_PHYSICAL_STREAM_ID) {
        control = snoopEnable | physicalStreamEnable;
    } else {
        control = snoopEnable | tmpIndex;
    }

    writel_base_offset(control, region_base, APS_AST_REGION_0_CONTROL_0);

    ASSERT(size != 0UL);
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    hi = HI32((uint64_t)(size - 1UL));
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    lo = (LOW32(size - 1UL) >> LO_SHIFT);

    control = (hi & APS_AST_REGION_0_MASK_HI_0_Mask_DEFAULT_MASK) <<
               APS_AST_REGION_0_MASK_HI_0_Mask_SHIFT;
    writel_base_offset(control, region_base, APS_AST_REGION_0_MASK_HI_0);

    control = (uint32_t)(lo & APS_AST_REGION_0_MASK_LO_0_Mask_DEFAULT_MASK) <<
               APS_AST_REGION_0_MASK_LO_0_Mask_SHIFT;
    writel_base_offset(control, region_base, APS_AST_REGION_0_MASK_LO_0);

    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    hi = HI32(master);
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    lo = (LOW32(master) >> LO_SHIFT);
    control = (hi & APS_AST_REGION_0_MASTER_BASE_HI_0_MastBase_DEFAULT_MASK) <<
               APS_AST_REGION_0_MASTER_BASE_HI_0_MastBase_SHIFT;
    writel_base_offset(control, region_base, APS_AST_REGION_0_MASTER_BASE_HI_0);

    control = (uint32_t)(lo & APS_AST_REGION_0_MASTER_BASE_LO_0_MastBase_DEFAULT_MASK) <<
               APS_AST_REGION_0_MASTER_BASE_LO_0_MastBase_SHIFT;
    writel_base_offset(control, region_base, APS_AST_REGION_0_MASTER_BASE_LO_0);

    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    hi = HI32(slave);
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    lo = (LOW32(slave) >> LO_SHIFT);
    control = (hi & APS_AST_REGION_0_SLAVE_BASE_HI_0_SlvBase_DEFAULT_MASK) <<
               APS_AST_REGION_0_SLAVE_BASE_HI_0_SlvBase_SHIFT;
    writel_base_offset(control, region_base, APS_AST_REGION_0_SLAVE_BASE_HI_0);

    control = (uint32_t)(lo & APS_AST_REGION_0_SLAVE_BASE_LO_0_SlvBase_DEFAULT_MASK) <<
               APS_AST_REGION_0_SLAVE_BASE_LO_0_SlvBase_SHIFT;
    control |= APS_AST_REGION_0_SLAVE_BASE_LO_0_Enable_FIELD;
    writel_base_offset(control, region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);
}


SECTION_AST_TEXT
error_t tegra_ast_enable_region(uint8_t region, uint8_t stream_id,
                                iova master, uint64_t slave,
                                uint64_t size, uint8_t instance_mask)
{
    bool     already_enabled;
    uint8_t  vmindex;
    error_t  err = E_SUCCESS;
    uint8_t  ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;
    uint64_t slave_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFF);
    uint64_t iova_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFFFF);
    uint64_t region_size_mask = MK_U64_ADDR_CONST(0xFFF);

    if ((region > MAX_REGION_INDEX) || (ast_instance == 0U) ||
        ((slave & ~slave_address_mask) != 0U) ||
        ((master & ~iova_address_mask) != 0U) ||
        (size == 0U) || ((size - 1U) > slave_address_mask) ||
        ((size & region_size_mask) != 0U) || ((size & (size - 1U)) != 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if ((((size - 1UL) & slave) != 0UL) ||
        (((size - 1UL) & master) != 0UL)) {
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    err = tegra_ast_add_streamid(stream_id, instance_mask);
    if (err != E_SUCCESS) {
        goto out;
    }

    if ((ast_instance & AST_INSTANCE_R5) != 0U) {
        err = ast_get_vmindex(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                              stream_id, &already_enabled, &vmindex);
        if (err != E_SUCCESS) {
            goto out;
        }

        if (!already_enabled) {
            /* Setting streamID should also enable the VM */
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            err = E_AST_INVALID_VMINDEX;
            goto out;
        }

        ast_enable_region(ast_tegra_id[AST_INSTANCE_R5_INDEX], region,
                          vmindex, master, slave, size);
    }

    if ((ast_instance & AST_INSTANCE_DMA) != 0U) {
        err = ast_get_vmindex(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                              stream_id, &already_enabled, &vmindex);
        if (err != E_SUCCESS) {
            goto out;
        }

        if (!already_enabled) {
            /* Setting streamID should also enable the VM */
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            err = E_AST_INVALID_VMINDEX;
            goto out;
        }

        ast_enable_region(ast_tegra_id[AST_INSTANCE_DMA_INDEX], region,
                          vmindex, master, slave, size);
    }
out:
    return err;
}

SECTION_AST_TEXT
error_t tegra_ast_disable_region(uint8_t region, uint8_t instance_mask)
{
    uint32_t region_base;
    uint32_t control;
    uint8_t ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;
    error_t err = E_SUCCESS;

    if ((region > MAX_REGION_INDEX) || (ast_instance == 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if (((instance_mask & AST_INSTANCE_R5) != 0U) &&
        (ast_tegra_id[AST_INSTANCE_R5_INDEX] != 0U)) {
        region_base = ast_region_base(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                      region);
        control = readl_base_offset(region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);
        control &= ~APS_AST_REGION_0_SLAVE_BASE_LO_0_Enable_FIELD;
        writel_base_offset(control, region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);
    }

    if (((instance_mask & AST_INSTANCE_DMA) != 0U) &&
        (ast_tegra_id[AST_INSTANCE_DMA_INDEX] != 0U)) {
        region_base = ast_region_base(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                      region);
        control = readl_base_offset(region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);
        control &= ~APS_AST_REGION_0_SLAVE_BASE_LO_0_Enable_FIELD;
        writel_base_offset(control, region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);
    }
out:
    return err;
}

SECTION_AST_TEXT
static error_t ast_get_region_stream_id(uint32_t ast_base,
                                        uint8_t region,
                                        uint8_t *streamID)
{
    uint32_t region_base = ast_region_base(ast_base, region);
    uint32_t val;
    error_t  err = E_SUCCESS;
    uint8_t  vmindex;

    val = readl_base_offset(region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);

    if ((val & APS_AST_REGION_0_SLAVE_BASE_LO_0_Enable_FIELD) != 0x01U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_REGION_DISABLED;
        goto out;
    }

    val = readl_base_offset(region_base, APS_AST_REGION_0_CONTROL_0);

    if ((val & APS_AST_REGION_0_CONTROL_0_Physical_FIELD) != 0U) {
        /*
         * Return error code indicating the request is invalid for regions
         * mapped with physical streamID
         */
        err = E_AST_INVALID_PHY_REGION_REQUEST;
        goto out;
    }

    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    vmindex = (uint8_t)((uint32_t)(val & APS_AST_REGION_0_CONTROL_0_VMIndex_FIELD) >>
               APS_AST_REGION_0_CONTROL_0_VMIndex_SHIFT);
    val = readl(streamid_ctl_reg(ast_base, vmindex));

    if ((val & APS_AST_STREAMID_CTL_0_Enable_FIELD) != 0x01U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_REGION_STREAMID_DISABLED;
        goto out;
    }

    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    *streamID = (uint8_t)((uint32_t)(val & APS_AST_STREAMID_CTL_0_StreamID_FIELD) >>
                 APS_AST_STREAMID_CTL_0_StreamID_SHIFT);
out:
    return err;
}

SECTION_AST_TEXT
error_t tegra_ast_get_region_stream_id(uint8_t region,
                                       uint8_t *streamID,
                                       uint8_t instance_mask)
{
    error_t err = E_SUCCESS;
    uint8_t ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;

    if ((ast_instance == AST_INSTANCE_R5_AND_DMA) || (ast_instance == 0U) ||
        (region > MAX_REGION_INDEX) || (streamID == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if ((ast_instance & AST_INSTANCE_R5) != 0U) {
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
        err = ast_get_region_stream_id(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                       region, streamID);
    } else {
        err = ast_get_region_stream_id(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                       region, streamID);
    }
out:
    return err;
}

SECTION_AST_TEXT
static error_t ast_get_region_mapping(uint32_t ast_base, uint8_t region,
                                      struct tegra_ast_region_map *return_region_map)
{
    uint32_t region_base = ast_region_base(ast_base, region);
    uint32_t hi, lo;
    uint64_t mask;
    error_t  err;

    hi = readl_base_offset(region_base, APS_AST_REGION_0_SLAVE_BASE_HI_0);
    lo = readl_base_offset(region_base, APS_AST_REGION_0_SLAVE_BASE_LO_0);

    if ((lo & APS_AST_REGION_0_SLAVE_BASE_LO_0_Enable_FIELD) != 0x01U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_REGION_DISABLED;
        goto out;
    }

    if (return_region_map == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    lo &= APS_AST_REGION_0_SLAVE_BASE_LO_0_SlvBase_FIELD;
    return_region_map->slave_base = hilo_to_64(hi, lo);

    hi = readl_base_offset(region_base, APS_AST_REGION_0_MASK_HI_0);
    lo = readl_base_offset(region_base, APS_AST_REGION_0_MASK_LO_0) |
               ~APS_AST_REGION_0_MASK_LO_0_READ_MASK;

    mask = hilo_to_64(hi, lo);
    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    return_region_map->size = mask + 1U;

    hi = readl_base_offset(region_base, APS_AST_REGION_0_MASTER_BASE_HI_0);
    lo = readl_base_offset(region_base, APS_AST_REGION_0_MASTER_BASE_LO_0);

    return_region_map->master_base = hilo_to_64(hi, lo);

    INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
    err = ast_get_region_stream_id(ast_base, region, &return_region_map->stream_id);
out:
    return err;
}

SECTION_AST_TEXT
error_t tegra_ast_get_region_mapping(uint8_t region,
                                     struct tegra_ast_region_map *return_region_map,
                                     uint8_t instance_mask)
{
    error_t err = E_SUCCESS;
    uint8_t ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;

    return_region_map->is_phy_region = false;

    if ((ast_instance == AST_INSTANCE_R5_AND_DMA) || (ast_instance == 0U) ||
        (region > MAX_REGION_INDEX) || (return_region_map == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    (void)memset(return_region_map, 0, sizeof(*return_region_map));

    if ((ast_instance & AST_INSTANCE_R5) != 0U) {
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
        err = ast_get_region_mapping(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                     region, return_region_map);
    } else {
        err = ast_get_region_mapping(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                     region, return_region_map);
    }
    if (err == E_AST_INVALID_PHY_REGION_REQUEST) {
        return_region_map->is_phy_region = true;
        err = E_SUCCESS;
    }
out:
   return err;
}

SECTION_AST_TEXT
iova tegra_ast_map_pointer_to_iova(const struct tegra_ast_region_map *map,
                                   const void *pointer)
{
    return tegra_ast_map_slave_to_iova(map, (uint64_t)(fsp_c_v_ptr_to_uptr(pointer)));
}

SECTION_AST_TEXT
uint64_t tegra_ast_get_local_pointer_value(const struct tegra_ast_region_map *map,
                                           const iova iova_addr)
{
    return tegra_ast_map_iova_to_slave(map, iova_addr);
}

SECTION_AST_TEXT
iova tegra_ast_map_slave_to_iova(const struct tegra_ast_region_map *map,
                                 const uint64_t slave)
{
    uint64_t mask;
    iova     addr = IOVA_NULL;
    uint64_t slave_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFF);
    uint64_t region_size_mask = MK_U64_ADDR_CONST(0xFFF);

    if ((map == NULL) ||
        (map->size == 0U) || ((map->size - 1U) > slave_address_mask) ||
        ((map->size & region_size_mask) != 0U) ||
        ((map->size & (map->size - 1U)) != 0U) ||
        ((map->slave_base & (map->size - 1U)) != 0U) ||
        ((map->master_base & (map->size - 1U)) != 0U) ||
        ((slave & ~slave_address_mask) != 0U)) {
        goto out;
    }

    mask = map->size - 1ULL;

    if ((slave & ~mask) == map->slave_base) {
        addr = (slave & mask) | map->master_base;
    }
out:
    return addr;
}

SECTION_AST_TEXT
uint64_t tegra_ast_map_iova_to_slave(const struct tegra_ast_region_map *map,
                                     const iova iova_addr)
{
    uint64_t mask;
    uint64_t slave_addr = 0ULL;
    uint64_t slave_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFF);
    uint64_t iova_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFFFF);
    uint64_t region_size_mask = MK_U64_ADDR_CONST(0xFFF);

    if ((map == NULL) ||
        (map->size == 0U) || ((map->size - 1U) > slave_address_mask) ||
        ((map->size & region_size_mask) != 0U) ||
        ((map->size & (map->size - 1U)) != 0U) ||
        ((map->slave_base & (map->size - 1U)) != 0U) ||
        ((map->master_base & (map->size - 1U)) != 0U) ||
        ((iova_addr & ~iova_address_mask) != 0U)) {
        goto out;
    }

    mask = map->size - 1ULL;

    if ((iova_addr & ~mask) == map->master_base) {
        slave_addr = (iova_addr & mask) | map->slave_base;
    }
out:
    return slave_addr;
}

SECTION_AST_TEXT
static error_t ast_set_default_vmindex(uint8_t ast_instance, uint8_t stream_id)
{
    error_t  err = E_SUCCESS;
    uint32_t control;
    bool     already_enabled;
    uint8_t  vmindex;


    if ((ast_instance & AST_INSTANCE_R5) != 0U) {
        err = ast_get_vmindex(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                              stream_id, &already_enabled, &vmindex);
        if (err != E_SUCCESS) {
            goto out;
        }
         if (!already_enabled) {
            err = ast_set_streamid(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                   vmindex, stream_id);
            if (err != E_SUCCESS){
                goto out;
            }
        }
        control = readl_base_offset(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                    APS_AST_CONTROL_0);
        control |= ((uint32_t)(vmindex & APS_AST_CONTROL_0_DefVMIndex_DEFAULT_MASK) <<
                     APS_AST_CONTROL_0_DefVMIndex_SHIFT);
        writel_base_offset(control, ast_tegra_id[AST_INSTANCE_R5_INDEX],
                           APS_AST_CONTROL_0);
    }

    if ((ast_instance & AST_INSTANCE_DMA) != 0U) {
        err = ast_get_vmindex(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                              stream_id, &already_enabled, &vmindex);
        if (err != E_SUCCESS) {
            goto out;
        }

        if (!already_enabled) {
            err = ast_set_streamid(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                   vmindex, stream_id);
            if (err != E_SUCCESS){
                goto out;
            }
        }
        control = readl_base_offset(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                    APS_AST_CONTROL_0);
        control |= ((uint32_t)(vmindex & APS_AST_CONTROL_0_DefVMIndex_DEFAULT_MASK) <<
                    APS_AST_CONTROL_0_DefVMIndex_SHIFT);
        writel_base_offset(control, ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                           APS_AST_CONTROL_0);
    }

out:
    return err;
}

SECTION_AST_TEXT
error_t tegra_ast_set_default_stream_id(uint8_t stream_id, uint8_t instance_mask,
                                        bool phy) {
    error_t  err = E_SUCCESS;
    uint8_t  ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;
    uint32_t physicalStreamEnable = APS_AST_CONTROL_0_DefPhysical_DEFAULT_MASK <<
                                    APS_AST_CONTROL_0_DefPhysical_SHIFT;
    uint32_t control;
    if (ast_instance == 0U) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if (phy) {
        /* Set DefPhysical bit in the AST_control_register */
        if ((ast_instance & AST_INSTANCE_R5) != 0U) {
            control = readl_base_offset(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                        APS_AST_CONTROL_0);
            control |= physicalStreamEnable;
            writel_base_offset(control, ast_tegra_id[AST_INSTANCE_R5_INDEX],
                               APS_AST_CONTROL_0);
        }
        if ((ast_instance & AST_INSTANCE_DMA) != 0U) {
            control = readl_base_offset(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                        APS_AST_CONTROL_0);
            control |= physicalStreamEnable;
            writel_base_offset(control, ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                               APS_AST_CONTROL_0);
        }
   } else {
        if (tegra_ast_is_global_locked() == true) {
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            err = E_AST_GLOBALLY_LOCKED;
            goto out;
        }

        err = tegra_ast_add_streamid(stream_id, ast_instance);
        if (err != E_SUCCESS) {
            goto out;
        }

        /* Clear DefPhysical bit in the AST_control_register */
        if ((ast_instance & AST_INSTANCE_R5) != 0U) {
            control = readl_base_offset(ast_tegra_id[AST_INSTANCE_R5_INDEX],
                                        APS_AST_CONTROL_0);
            control &= ~physicalStreamEnable;
            writel_base_offset(control, ast_tegra_id[AST_INSTANCE_R5_INDEX],
                               APS_AST_CONTROL_0);
        }
        if ((ast_instance & AST_INSTANCE_DMA) != 0U) {
            control = readl_base_offset(ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                                        APS_AST_CONTROL_0);
            control &= ~physicalStreamEnable;
            writel_base_offset(control, ast_tegra_id[AST_INSTANCE_DMA_INDEX],
                               APS_AST_CONTROL_0);
        }

        /* Set DefVMindex */
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
        err = ast_set_default_vmindex(ast_instance, stream_id);
    }
out:
    return err;
}

error_t tegra_ast_enable_phy_region(uint8_t region,
                                    iova master,
                                    uint64_t slave,
                                    uint64_t size,
                                    uint8_t instance_mask)
{
    error_t  err = E_SUCCESS;
    uint8_t  ast_instance = instance_mask & AST_INSTANCE_R5_AND_DMA;
    uint64_t slave_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFF);
    uint64_t iova_address_mask = MK_U64_ADDR_CONST(0xFFFFFFFFFF);
    uint64_t region_size_mask = MK_U64_ADDR_CONST(0xFFF);

    if ((region > MAX_REGION_INDEX) || (ast_instance == 0U) ||
        ((slave & ~slave_address_mask) != 0U) ||
        ((master & ~iova_address_mask) != 0U) ||
        (size == 0U) || ((size - 1U) > slave_address_mask) ||
        ((size & region_size_mask) != 0U) || ((size & (size - 1U)) != 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if ((((size - 1UL) & slave) != 0UL) ||
        (((size - 1UL) & master) != 0UL)) {
        err = E_AST_INVALID_INPUT_PARAMETERS;
        goto out;
    }

    if ((ast_instance & AST_INSTANCE_R5) != 0U) {
        if (ast_is_global_locked(ast_tegra_id[AST_INSTANCE_R5_INDEX])) {
             INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
             err = E_AST_GLOBALLY_LOCKED;
             goto out;
        }
        ast_enable_region(ast_tegra_id[AST_INSTANCE_R5_INDEX], region,
                          (uint8_t)VMINDEX_PHYSICAL_STREAM_ID, master,
                          slave, size);
    }

    if ((ast_instance & AST_INSTANCE_DMA) != 0U) {
        if (ast_is_global_locked(ast_tegra_id[AST_INSTANCE_DMA_INDEX])) {
             INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
             err = E_AST_GLOBALLY_LOCKED;
             goto out;
        }
        ast_enable_region(ast_tegra_id[AST_INSTANCE_DMA_INDEX], region,
                          (uint8_t)VMINDEX_PHYSICAL_STREAM_ID, master,
                          slave, size);
    }

out:
    return err;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
