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

#include <error/common-errors.h>
#include <lic/lic-tegra.h>
#include <processor/irqs-lic.h>
#include <debug/print.h>

#include <spe-lic.h>
#include <lic-map.h>
#include <err-hook.h>

void
spe_lic_enable(const uint32_t irq)
{
    error_t ret;

    ret = tegra_lic_irq_enable(&tegra_lic_id_aon, irq);
    if (ret != E_SUCCESS) {
        error_hookf("spe_lic_enable(%u) failed!\r\n", (unsigned int)irq);
    }
}

void
spe_lic_disable(const uint32_t irq)
{
    error_t ret;

    ret = tegra_lic_irq_disable(&tegra_lic_id_aon, irq);
    if (ret != E_SUCCESS) {
        error_hookf("spe_lic_disable(%u) failed!\r\n", (unsigned int)irq);
    }
}

error_t
spe_lic_init(void)
{
    const lic_irq_t *lic_map;
    uint32_t        lic_map_size;
    error_t         ret;

    ret = get_aon_lic_map(&lic_map, &lic_map_size);
    if (ret != E_SUCCESS) {
        error_hook("get_aon_lic_map() failed!\r\n");
        goto out;
    }

    ret = tegra_lic_init(&tegra_lic_id_aon,
                         lic_map,
                         lic_map_size);

out:
    return ret;
}
