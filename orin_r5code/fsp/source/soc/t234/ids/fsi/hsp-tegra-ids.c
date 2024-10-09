/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT
#include <soc-common/hw-const.h>          /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>              // for NV_ADDRESS_MAP_FSI_HSP_BASE
#include <arhsp_shrd_mbox.h>              // for HSP_SHRD_MBOX_MBOX_0_SHRD_M...

/* Late FSP headers */
#include <hsp/hsp-tegra-priv.h>           // for tegra_hsp_id, FSP__HSP__HSP...
#include <hsp/sections-hsp.h>             // Immune from CT_ASSERT protection
#include <processor/irqs-hw.h>

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

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_top2_hsp_tx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_TOP2_HSP_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_TOP_HSP2_SI0,
        .si_index = 0U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};
SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_top2_hsp_rx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_TOP2_HSP_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_TOP_HSP2_SI1,
        .si_index = 1U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_fsi_hsp_tx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_FSI_HSP_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_HSP_SI0,
        .si_index = 0U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_fsi_hsp_rx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_FSI_HSP_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_HSP_SI4,
        .si_index = 4U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_sce_hsp1_tx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_HSP1_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_SCE_HSP1_SI2,
        .si_index = 2U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_sce_hsp1_rx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_HSP1_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_SCE_HSP1_SI3,
        .si_index = 3U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_sce_hsp2_tx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_HSP2_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_SCE_HSP2_SI2,
        .si_index = 2U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};

SECTION_HSP_DATA
struct tegra_hsp_id tegra_hsp_id_sce_hsp2_rx = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_SCE_HSP2_BASE,
        .sh_irq = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_SCE_HSP2_SI4,
        .si_index = 4U,
    },
    .n_sm = 8U,
    .n_ss = 4U,
    .n_as = 0U,
    .n_db = 0U,
    .n_si = 8U,
};
