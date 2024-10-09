/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>                 // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <hw-config/vic-irqs.h>             // for INTERRUPT_LIC...
#include <processor/irqs-lic.h>             // for DCE_LIC_IRQ_BASE
#include <lic/lic-tegra-priv.h>             // for struct tegra_lic_id,...
#include <lic/sections-lic.h>               // Immune from CT_ASSERT

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/macros.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
    CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__HW_CONFIG__VIC_IRQS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__LIC__LIC_TEGRA_PRIV_H, "Header file missing or invalid.")

#define DCE_LIC_IRQ_LINES 4U
#define DCE_LIC_BASE_CHAN 8U
#define DCE_LIC_NUM_CHANS 2U

SECTION_LIC_DATA
struct tegra_lic_id tegra_lic_id_dce;

SECTION_LIC_DATA
lic_irq_context_t lic0_irq_context = {
    .id      = &tegra_lic_id_dce,
    .chan    = DCE_LIC_BASE_CHAN,
    .is_vfiq = false,
};

SECTION_LIC_DATA
lic_irq_context_t lic1_irq_context = {
    .id      = &tegra_lic_id_dce,
    .chan    = DCE_LIC_BASE_CHAN,
    .is_vfiq = true,
};

SECTION_LIC_DATA
lic_irq_context_t lic2_irq_context = {
    .id      = &tegra_lic_id_dce,
    .chan    = DCE_LIC_BASE_CHAN + 1U,
    .is_vfiq = false,
};

SECTION_LIC_DATA
lic_irq_context_t lic3_irq_context = {
    .id      = &tegra_lic_id_dce,
    .chan    = DCE_LIC_BASE_CHAN + 1U,
    .is_vfiq = true,
};

static lic_irq_context_t *lic_irq_contexts[DCE_LIC_IRQ_LINES] = {
    &lic0_irq_context,
    &lic1_irq_context,
    &lic2_irq_context,
    &lic3_irq_context,
};

struct tegra_lic_id tegra_lic_id_dce = {
    .conf = {
        .base_chan          = DCE_LIC_BASE_CHAN,
        .num_chans          = DCE_LIC_NUM_CHANS,
        .num_slices         = LIC_MAX_SLICES,
        .local_irq_base     = (uint16_t)INTERRUPT_LIC0,
        .lic_irq_base       = (uint16_t)LIC_IRQ_BASE,
    },
    .irq_contexts      = lic_irq_contexts,
    .num_lic_irq_lines = ARRAY_SIZE(lic_irq_contexts),
};
