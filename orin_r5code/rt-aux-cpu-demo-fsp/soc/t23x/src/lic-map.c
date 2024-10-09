/*
 * Copyright (c) 2020-2022, NVIDIA CORPORATION. All rights reserved.
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

#include <stdint.h>

#include <address_map_new.h>

#include <error/common-errors.h>
#include <lic/lic-tegra.h>
#include <processor/irqs-lic.h>
#include <hw-config/vic-irqs.h>

#include <lic-map.h>
#include <spe-errors.h>

#define LIC_IRQ_OFFSET(_irq)  (AON_LIC_IRQ_ ## _irq - AON_LIC_IRQ_BASE)

#define LIC_IRQ_MAPPING(_local_irq, _global_irq, _irq_line) \
    [LIC_IRQ_OFFSET(_local_irq)] = TEGRA_LIC_INTERRUPT(_irq_line, _global_irq)

#define LIC_IRQ_MAPPING_H(_local_irq, _global_irq, _irq_line, _isr, _isr_data) \
    [LIC_IRQ_OFFSET(_local_irq)] = \
            TEGRA_LIC_INTERRUPT_H(_irq_line, _global_irq, _isr, _isr_data)

static const lic_irq_t aon_lic_map[] = {
    LIC_IRQ_MAPPING(TOP_HSP1, TOP_HSP1_SHARED_4, INTERRUPT_LIC0),
};

error_t
get_aon_lic_map(const lic_irq_t **lic_map,
                uint32_t *lic_map_size)
{
    error_t ret = E_SUCCESS;

    if (lic_map == NULL || lic_map_size == NULL) {
        ret = E_SPE_ERR_NULL_PTR;
        goto out;
    }

    *lic_map = aon_lic_map;
    *lic_map_size = ARRAY_SIZE(aon_lic_map);

out:
    return ret;
}
