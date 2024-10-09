/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */
#include <stdint.h>                  // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>          // for CT_ASSERT
#include <soc-common/hw-const.h>     /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>         // for NV_ADDRESS_MAP_AON_TKE_BASE, NV_...

/* Late FSP headers */
#include <tke/sections-tke.h>        // Immune from CT_ASSERT protection
#include <tke/tke-tegra-priv.h>      // for tegra_tke_id, FSP__TKE__TKE_TEGR...

/* Module-specific FSP header files */
#include <processor/irqs-hw.h>       // for FSP__PROCESSOR__IRQS_HW_H, NV_AO...

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_PRIV_H, "Header file missing or invalid.")

const uint32_t tke_top_base = NV_ADDRESS_MAP_SCE_TKE_BASE;

SECTION_TKE_DATA
struct tegra_tke_id tegra_tke_id_timer0 = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_0_BASE,
        .irq = NV_SCE_IRQ_TIMER0,
    },
};

SECTION_TKE_DATA
struct tegra_tke_id tegra_tke_id_timer1 = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_1_BASE,
        .irq = NV_SCE_IRQ_TIMER1,
    },
};

SECTION_TKE_DATA
struct tegra_tke_id tegra_tke_id_timer2 = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_2_BASE,
        .irq = NV_SCE_IRQ_TIMER2,
    },
};

SECTION_TKE_DATA
struct tegra_tke_id tegra_tke_id_timer3 = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_3_BASE,
        .irq = NV_SCE_IRQ_TIMER3,
    },
};
