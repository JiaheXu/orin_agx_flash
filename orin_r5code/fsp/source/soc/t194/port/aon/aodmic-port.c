/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdio.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <chipid/chip-id.h>
#include <clk/clk-tegra.h>
#include <soc-common/clk-tegra-hw.h>

/* Module-specific FSP headers */
#include <aodmic/aodmic-errors.h>
#include <aodmic/sections-aodmic.h>     // for SECTION_AODMIC_...
#include <aodmic/tegra-aodmic-priv.h>
#include <port/aodmic-port.h>
#include <port/aodmic-port-priv.h>

SECTION_AODMIC_RODATA
const struct aodmic_port_clk_rst aodmic_port_clk_rst_aon = {
    .clk = tegra_clk_aodmic,
    .rst = tegra_rst_aodmic,
};

SECTION_AODMIC_INIT_TEXT error_t
aodmic_port_clock_reset_config(const struct aodmic_port_clk_rst *hw_handle)
{
    error_t ret_val = E_SUCCESS;

    if (hw_handle == NULL) {
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    if(!tegra_platform_is_fpga()) {
        /* Enable AODMIC clock */
        tegra_clk_enable(hw_handle->clk);
        tegra_clk_reset_pulse(hw_handle->rst, 2);
    }

out:
    return ret_val;
}

SECTION_AODMIC_TEXT error_t
aodmic_port_clock_disable(const struct aodmic_port_clk_rst *hw_handle)
{
    error_t ret_val = E_SUCCESS;

    if (hw_handle == NULL) {
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    tegra_clk_disable(hw_handle->clk);

out:
    return ret_val;
}

SECTION_AODMIC_INIT_TEXT error_t
aodmic_port_clock_set_rate(const struct aodmic_port_clk_rst *hw_handle,
                           uint32_t sample_rate)
{
    int32_t status;
    error_t ret_val = E_SUCCESS;

    if (hw_handle == NULL) {
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    status = tegra_clk_set_rate(hw_handle->clk, sample_rate);
    if (status) {
        ret_val = E_AODMIC_CLK_RATE_ERROR;
    }

out:
    return ret_val;
}

