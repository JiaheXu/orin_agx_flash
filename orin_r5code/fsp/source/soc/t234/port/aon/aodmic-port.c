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
#include <stdint.h>                   // for uint8_t, uintptr_t, int32_t
#include <stdio.h>                    // for NULL
                                      // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>           // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>      // for MODULE_ID_AODMIC, E_SUCCESS
#include <chipid/chip-id.h>           // for tegra_platform_is_fpga
#include <clk/clk-tegra.h>            // for tegra_clk_disable, tegra_clk_en...
#include <misc/macros.h>              // for END_RFD_BLOCK, START_RFD_BLOCK
#include <soc-common/clk-tegra-hw.h>  // for tegra_clk_enable, ...

/* Module-specific FSP headers */
#include <aodmic/aodmic-errors.h>     // for E_AODMIC_NULL_POINTER, E_AODMIC...
#include <aodmic/sections-aodmic.h>   // for SECTION_AODMIC_...
#include <port/aodmic-port.h>         // for aodmic_port_clock_reset_config, ...
#include <port/aodmic-port-priv.h>    // for aodmic_port_clk_rst

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CHIPID__CHIP_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__CLK_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__AODMIC__AODMIC_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PORT__AODMIC_PORT_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

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
