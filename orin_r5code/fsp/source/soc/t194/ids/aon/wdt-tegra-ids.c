/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>              // for CT_ASSERT
#include <soc-common/hw-const.h>         /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>             // for NV_ADDRESS_MAP_AON_TKE_WDT_0...

/* Late FSP headers */
#include <misc/macros.h>                 // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <watchdog/sections-watchdog.h>  // Immune from CT_ASSERT protection
#include <watchdog/wdt-tegra-priv.h>     // for tegra_wdt

/* Module-specific FSP header files */
#include <processor/irqs-hw.h>           // for NV_AON_INTERRUPT_WDTFIQ, NV_...

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
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__WDT_TEGRA_PRIV_H, "Header file missing or invalid.")

extern struct tegra_wdt tegra_wdt_instance;

SECTION_WATCHDOG_DATA
struct tegra_wdt tegra_wdt_instance = {
    .id = {
        .devname        = "wdt",
        .base_addr      = NV_ADDRESS_MAP_AON_TKE_WDT_0_BASE,
        .irq            = NV_AON_INTERRUPT_WDTIRQ,
        .fiq            = NV_AON_INTERRUPT_WDTFIQ,
    },
};
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
