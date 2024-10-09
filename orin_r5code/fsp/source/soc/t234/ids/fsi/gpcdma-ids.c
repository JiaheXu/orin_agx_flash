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

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>                 // for CT_ASSERT
#include <soc-common/hw-const.h>            /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>                // for NV_ADDRESS_MAP_AON_DMA_BASE

/* Late FSP headers */
#include <soc-common/clk-tegra-hw.h>        // for tegra_rst_aon_gpcdma
#include <gpcdma/gpcdma-priv.h>             // for struct gpcdma_id, gpcdma_channel
#include <gpcdma/sections-gpcdma.h>         // Immune from CT_ASSERT
#include <port/gpcdma-port-priv.h>          // Immune from CT_ASSERT

#include <processor/irqs-hw.h>       // for FSP__PROCESSOR__IRQS_HW_H

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
    CT_ASSERT(FSP__SOC_COMMON__CLK_TEGRA_HW_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__GPCDMA__GPCDMA_PRIV_H, "Header file missing or invalid.")

SECTION_GPCDMA_RODATA
const struct gpcdma_hw_handle gpcdma_hw_handle_fsi = {
    /* FIXME: add CLK for GPCDMA */
    .rst = 0,
};

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan0;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan1;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan2;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan3;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan4;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan5;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan6;

SECTION_GPCDMA_DATA
struct gpcdma_channel gpcdma_chan7;

SECTION_GPCDMA_DATA
struct gpcdma_id gpcdma_id_fsi = {
    .conf = {
        .base_addr = NV_ADDRESS_MAP_FSI_DMA_BASE,
        .irqs = {
            [0] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR0,
            [1] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR1,
            [2] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR2,
            [3] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR3,
            [4] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR4,
            [5] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR5,
            [6] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR6,
            [7] = NV_FSI_SPI_INTR_BASE + NV_FSI_GIC_DMA_INTR7,
        },
        .hw_handle = &gpcdma_hw_handle_fsi,
    },
    .channels = {
        [0] = &gpcdma_chan0,
        [1] = &gpcdma_chan1,
        [2] = &gpcdma_chan2,
        [3] = &gpcdma_chan3,
        [4] = &gpcdma_chan4,
        [5] = &gpcdma_chan5,
        [6] = &gpcdma_chan6,
        [7] = &gpcdma_chan7,
    },
};
