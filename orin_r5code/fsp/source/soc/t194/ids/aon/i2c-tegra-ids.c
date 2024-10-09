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
#include <stdbool.h>                  // for true
#include <stddef.h>                   // for NULL
#include <stdint.h>                   // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>           // for CT_ASSERT
#include <soc-common/hw-const.h>      /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>          // for NV_ADDRESS_MAP_I2C2_BASE, NV_AD...

/* Late FSP headers */
#include <hw-config/vic-irqs.h>       // for FSP__HW_CONFIG__VIC_IRQS_H, INT...

/* Module-specific FSP header files */
#include <processor/i2c-tegra-hw.h>   // for i2c_hw_handle, FSP__PROCESSOR__...
#include <soc-common/clk-tegra-hw.h>  // for FSP__SOC_COMMON__CLK_TEGRA_HW_H
#include <soc-common/i2c-defs.h>      // for TEGRA194_I2C2, TEGRA194_I2C8

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
CT_ASSERT(FSP__HW_CONFIG__VIC_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__I2C_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__CLK_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEFS_H, "Header file missing or invalid.")

static const struct i2c_hw_handle t194_i2c_ctrl2 = {
    .ctrl_id = TEGRA194_I2C2,
    .base = NV_ADDRESS_MAP_I2C2_BASE,
    .clk_data = tegra_clk_i2c2,
    .rst_data = tegra_rst_i2c2,
    .irq_no = INTERRUPT_I2C2,
    .is_multimaster = (bool)true,
};

static const struct i2c_hw_handle t194_i2c_ctrl8 = {
    .ctrl_id = TEGRA194_I2C8,
    .base = NV_ADDRESS_MAP_I2C8_BASE,
    .clk_data = tegra_clk_i2c8,
    .rst_data = tegra_rst_i2c8,
    .irq_no = INTERRUPT_I2C3,
    .is_multimaster = (bool)true,
};

const struct i2c_hw_handle *i2c_get_hw_handle(uint32_t ctrl_id)
{
    struct i2c_hw_handle const *hi2c_hw;

    switch(ctrl_id) {
    case TEGRA194_I2C2:
        hi2c_hw = &t194_i2c_ctrl2;
        break;
    case TEGRA194_I2C8:
        hi2c_hw = &t194_i2c_ctrl8;
        break;
    default:
        hi2c_hw = NULL;
        break;
    }

    return hi2c_hw;
}
