/*
 * Copyright (c) 2017-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>                  // for true
#include <stddef.h>                   // for NULL
#include <stdint.h>                   // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>           // for CT_ASSERT
#include <soc-common/hw-const.h>      /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>         // for NV_ADDRESS_MAP_SPI2_BASE

/* Late FSP headers */
#include <hw-config/vic-irqs.h>       // for INTERRUPT_SPI
#include <processor/irqs-lic.h>       // for INTERRUPT_SPIx
#include <soc-common/clk-tegra-hw.h>  // for tegra_[rst/clk]_spi2

/* Module-specific FSP header files */
#include <spi/spi-priv.h>             // for spi_hw_handle, FSP__PROCESSOR__...
#include <spi/sections-spi.h>         // for SECTION_SPI_...
#include <port/spi-port-priv.h>       // Immune from CT_ASSERT

#define INTERRUPT_SPI1  AON_LIC_IRQ_SPI1
#define INTERRUPT_SPI3  AON_LIC_IRQ_SPI3

SECTION_SPI_RODATA
static const struct spi_hw_handle spi1_hw_handle = {
    .rst = tegra_rst_spi1,
    .clk = tegra_clk_spi1,
};

SECTION_SPI_RODATA
static const struct spi_hw_handle spi2_hw_handle = {
    .rst = tegra_rst_spi2,
    .clk = tegra_clk_spi2,
};

SECTION_SPI_RODATA
static const struct spi_hw_handle spi3_hw_handle = {
    .rst = tegra_rst_spi3,
    .clk = tegra_clk_spi3,
};

SECTION_SPI_DATA
struct spi_ctlr spi_ctlr_spi1 = {
	.conf = {
		.base_addr = NV_ADDRESS_MAP_SPI1_BASE,
        .hw_handle = &spi1_hw_handle,
		.irq = INTERRUPT_SPI1,
	},
    .slave_dma_support = false,
    .en_full_duplex = false,
};

SECTION_SPI_DATA
struct spi_ctlr spi_ctlr_spi2 = {
	.conf = {
		.base_addr = NV_ADDRESS_MAP_SPI2_BASE,
        .hw_handle = &spi2_hw_handle,
		.irq = INTERRUPT_SPI,
	},
    .slave_dma_support = false,
    .en_full_duplex = false,
};

SECTION_SPI_DATA
struct spi_ctlr spi_ctlr_spi3 = {
	.conf = {
		.base_addr = NV_ADDRESS_MAP_SPI3_BASE,
        .hw_handle = &spi3_hw_handle,
		.irq = INTERRUPT_SPI3,
	},
    .slave_dma_support = false,
    .en_full_duplex = false,
};
