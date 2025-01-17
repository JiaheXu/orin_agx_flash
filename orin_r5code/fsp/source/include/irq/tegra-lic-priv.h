/*
 * Copyright (c) 2016-2020 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef IRQ__TEGRA_LIC_PRIV_H
#define IRQ__TEGRA_LIC_PRIV_H
#define FSP__IRQ__TEGRA_LIC_PRIV_H                      1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

/* Mapping of <irqs.h> numbers to LIC IRQs */
struct tegra_lic_bitslice {
	uint8_t bit;
	uint8_t slice;
};

struct tegra_lic_id {
	uint8_t base_channel;
	uint8_t num_channels;
	uint8_t num_slices;
	uint16_t local_irq;
	uint16_t lic_irq_base;
	uint16_t lic_map_size;
	const struct tegra_lic_bitslice *lic_map;
};

#define TEGRA_LIC_BITSLICE(lic) \
	{ \
	.bit = (NV_ADDRESS_MAP_ ## lic ## _INTR_ID) % 32, \
	.slice = (NV_ADDRESS_MAP_ ## lic ## _INTR_ID) / 32, \
	}

#endif	/* FSP_IRQ_TEGRA_LIC_PRIV_H */
