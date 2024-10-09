/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>                   // for uint8_t, uint32_t
#include <stdbool.h>                  // for false, bool, true

/* Early FSP headers */
#include <misc/ct-assert.h>           // for CT_ASSERT
#include <soc-common/hw-const.h>      /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>          // for NV_ADDRESS_MAP_FUSE_BASE, NV_AD...

/* Late FSP headers */
#include <misc/macros.h>              // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <reg-access/reg-access.h>    // for readl, FSP__REG_ACCESS__REG_ACC...

/* Module-specific FSP headers */
#include <chipid/chip-id.h>           // for FSP__CHIPID__CHIP_ID_H, tegra_g...
#include <chipid/sections-chip-id.h>  // Immune from CT_ASSERT protection


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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CHIPID__CHIP_ID_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define MISCREG_HIDREV_0              4U

#define HIDREV_0_MAJORREV_SHIFT       4U
#define HIDREV_0_CHIPID_SHIFT         8U
#define HIDREV_0_MINORREV_SHIFT      16U
#define HIDREV_0_PLATFORM_SHIFT      20U
#define HIDREV_0_PLATFORM_SHIFT      20U
#define HIDREV_0_PLATFORM_SILICON     0U
#define HIDREV_0_PLATFORM_FPGA        2U
#define HIDREV_0_PLATFORM_VDK         8U

#define HIDREV_0_MAJORREV_MASK       ((uint32_t)0xf << HIDREV_0_MAJORREV_SHIFT)
#define HIDREV_0_CHIPID_MASK          ((uint32_t)0xff << HIDREV_0_CHIPID_SHIFT)
#define HIDREV_0_MINORREV_MASK       ((uint32_t)0xf << HIDREV_0_MINORREV_SHIFT)
#define HIDREV_0_PLATFORM_MASK       ((uint32_t)0xf << HIDREV_0_PLATFORM_SHIFT)

#define FUSE_OPT_SUBREVISION_0    0x248U
#define FUSE_OPT_SUBREVISION_SHIFT    0U
#define FUSE_OPT_SUBREVISION_MASK ((uint32_t)0xf << FUSE_OPT_SUBREVISION_SHIFT)

#define FUSE_SKU_INFO_0           0x110U
#define FUSE_SKU_INFO_SHIFT           0U
#define FUSE_SKU_INFO_MASK              ((uint32_t)0xff << FUSE_SKU_INFO_SHIFT)

/**
 * @brief ChipID information.
 *
 * @chip_id Chip ID value.
 * @major_rev Chip major revision.
 * @minor_rev Chip minor revision.
 * @sub_rev  Chip sub revision.
 * @sku_id  Chip SKU ID.
 * @platform_is_silicon Specifies whether platform is silicon or not.
 * @chip_data_initialized Specifies whether chip data is initialized or not.
 *
 */
struct chip_info {
    uint8_t chip_id;
    uint8_t major_rev;
    uint8_t minor_rev;
    uint8_t sub_rev;
    uint8_t sku_id;
    bool platform_is_silicon;
    bool platform_is_fpga;
    bool platform_is_vdk;
    bool chip_data_initialized;
};

static struct chip_info chip_data SECTION_CHIPID_DATA = {
    .platform_is_silicon = (bool)false,
    .chip_data_initialized = (bool)false,
};

static SECTION_CHIPID_INIT_TEXT void tegra_chipid_init(void)
{
    uint32_t val;

    val = readl((uint32_t)NV_ADDRESS_MAP_MISC_BASE + MISCREG_HIDREV_0);

    /* Get chip ID */
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    chip_data.chip_id = (uint8_t)((uint32_t)(val & HIDREV_0_CHIPID_MASK) >> HIDREV_0_CHIPID_SHIFT);

    /* Get Major Revision */
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    chip_data.major_rev = (uint8_t)((uint32_t)(val & HIDREV_0_MAJORREV_MASK) >> HIDREV_0_MAJORREV_SHIFT);

    /* Get Minor Revision */
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    chip_data.minor_rev = (uint8_t)((uint32_t)(val & HIDREV_0_MINORREV_MASK) >> HIDREV_0_MINORREV_SHIFT);

    if (HIDREV_0_PLATFORM_SILICON == ((uint32_t)(val & HIDREV_0_PLATFORM_MASK) >> HIDREV_0_PLATFORM_SHIFT)) {
        chip_data.platform_is_silicon = true;
    }

    if (HIDREV_0_PLATFORM_FPGA == ((uint32_t)(val & HIDREV_0_PLATFORM_MASK) >> HIDREV_0_PLATFORM_SHIFT)) {
        chip_data.platform_is_fpga = true;
    }

    if (HIDREV_0_PLATFORM_VDK == ((uint32_t)(val & HIDREV_0_PLATFORM_MASK) >> HIDREV_0_PLATFORM_SHIFT)) {
        chip_data.platform_is_vdk = true;
    }

    /* Get Sub Revision from Fuse*/
    val = readl((uint32_t)NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_SUBREVISION_0);
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    chip_data.sub_rev = (uint8_t)((val & FUSE_OPT_SUBREVISION_MASK) >> FUSE_OPT_SUBREVISION_SHIFT);

    /* Get SKU Info from Fuse*/
    val = readl((uint32_t)NV_ADDRESS_MAP_FUSE_BASE + FUSE_SKU_INFO_0);
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    chip_data.sku_id = (uint8_t)((val & FUSE_SKU_INFO_MASK) >> FUSE_SKU_INFO_SHIFT);

    chip_data.chip_data_initialized = true;
}

SECTION_CHIPID_TEXT uint8_t tegra_get_chipid(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.chip_id;
}

SECTION_CHIPID_TEXT uint8_t tegra_get_major_rev(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.major_rev;
}

SECTION_CHIPID_TEXT uint8_t tegra_get_minor_rev(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.minor_rev;
}

SECTION_CHIPID_TEXT bool tegra_platform_is_silicon(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.platform_is_silicon;
}

SECTION_CHIPID_TEXT bool tegra_platform_is_vdk(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.platform_is_vdk;
}

SECTION_CHIPID_TEXT bool tegra_platform_is_fpga(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.platform_is_fpga;
}

SECTION_CHIPID_TEXT uint8_t tegra_get_sub_rev(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.sub_rev;
}

SECTION_CHIPID_TEXT uint8_t tegra_get_sku_id(void)
{
    /* Check if chip data is initialized or not */
    if (chip_data.chip_data_initialized == false) {
        tegra_chipid_init();
    }

    return chip_data.sku_id;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
