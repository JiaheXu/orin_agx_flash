/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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
#include <misc/ct-assert.h>

/* Hardware headers */
#include <mach/address_map_new_hex_misra.h>

/* Late FSP headers */
#include <misc/bitops.h>
#include <clk.h>
#include <mach/reset.h>
#include <bpmp/irq.h>
#undef ARRAY_SIZE
#undef DIV_ROUND_UP

/* Module-specific FSP header files */
#include <processor/i2c-tegra-hw.h>
#include <soc-common/i2c-dependencies.h>

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
CT_ASSERT(FSP__PROCESSOR__I2C_TEGRA_HW_H, "Header file missing or invalid.")

const struct i2c_hw_handle t194_i2c_ctrl5 = {
    .ctrl_id = TEGRA194_I2C5,
    .base = NV_ADDRESS_MAP_I2C5_BASE,
    .clk_id = TEGRA194_CLK_I2C5,
    .rst_id = TEGRA194_RESET_I2C5,
    .irq_no = BPMP_INT_SOCPWRI2C,
    .is_multimaster = false,
};

const struct i2c_hw_handle t194_i2c_ctrl10 = {
    .ctrl_id = TEGRA194_I2C10,
    .base = NV_ADDRESS_MAP_I2C10_BASE,
    .clk_id = TEGRA194_CLK_I2C10,
    .rst_id = TEGRA194_RESET_I2C10,
    .irq_no = UINT32_MAX, /* Invalid Interrupt number. Uses same lane as I2C5 */
    .is_multimaster = false,
};

const struct i2c_hw_handle *i2c_get_hw_handle(uint32_t ctrl_id)
{
    const struct i2c_hw_handle *hi2c_hw;

    switch (ctrl_id) {
    case TEGRA194_I2C5:
        hi2c_hw = &t194_i2c_ctrl5;
        break;
    case TEGRA194_I2C10:
        hi2c_hw = &t194_i2c_ctrl10;
        break;
    default:
        hi2c_hw = NULL;
        break;
    }

    return hi2c_hw;
}