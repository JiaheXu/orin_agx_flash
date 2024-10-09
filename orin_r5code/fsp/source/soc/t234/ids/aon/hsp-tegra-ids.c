/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>                       // for UINT32_MAX, UINT8_MAX

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT
#include <soc-common/hw-const.h>          /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>              // for NV_ADDRESS_MAP_AON_HSP_BASE
#include <arhsp_shrd_mbox.h>              // for HSP_SHRD_MBOX_MBOX_0_SHRD_M...

/* Late FSP headers */
#include <hsp/hsp-tegra-priv.h>           // for tegra_hsp_id, FSP__HSP__HSP...
#include <hsp/sections-hsp.h>             // Immune from CT_ASSERT protection

/* Module-specific FSP headers */
#include <hw-config/vic-irqs.h>           // for INTERRUPT_MBOX, INTERRUPT_T...
#include <processor/irqs-lic.h>           // for AON_LIC_IRQ_TOP_HSP0
#include <soc-common/hsp-tegra-top-hw.h>  // for TEGRA_HSP_DB_SPE

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
CT_ASSERT(FSP__HSP__HSP_TEGRA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HW_CONFIG__VIC_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__HSP_TEGRA_TOP_HW_H, "Header file missing or invalid.")

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_top0 = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_TOP0_HSP_BASE,
        .host = TEGRA_HSP_DB_SPE,
        .db_irq = INTERRUPT_TOP0_HSP_DB,
        .sh_irq = UINT32_MAX,
        .si_index = UINT8_MAX,
    },
    .inited = false,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_top1 = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_TOP1_HSP_BASE,
        .db_irq = UINT32_MAX,
        .sh_irq = AON_LIC_IRQ_TOP_HSP1,
        .si_index = 4,
    },
    .inited = false,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_fsi = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_FSI_HSP_BASE,
        .db_irq = UINT32_MAX,
        .sh_irq = UINT32_MAX,
        .si_index = UINT8_MAX,
    },
    .inited = false,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_bpmp = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_BPMP_HSP_BASE,
        .db_irq = UINT32_MAX,
        .sh_irq = UINT32_MAX,
        .si_index = UINT8_MAX,
    },
    .inited = false,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_aon = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_AON_HSP_BASE,
        .db_irq = UINT32_MAX,
        .sh_irq = INTERRUPT_MBOX,
        .si_index = 0,
    },
    .inited = false,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_sce = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_HSP_BASE,
        .db_irq = UINT32_MAX,
        .sh_irq = UINT32_MAX,
        .si_index = UINT8_MAX,
    },
    .inited = false,
};
