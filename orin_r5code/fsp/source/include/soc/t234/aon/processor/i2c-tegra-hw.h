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

#ifndef I2C_TEGRA_HW_H
#define I2C_TEGRA_HW_H
#define FSP__PROCESSOR__I2C_TEGRA_HW_H                  1

/* Compiler headers */
#include <stdbool.h>              // for bool
#include <stdint.h>               // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>          // for BIT

/* Module-specific FSP headers */
#include <soc-common/i2c-defs.h>  // for FSP__SOC_COMMON__I2C_DEFS_H, TEGRA1...

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
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEFS_H, "Header file missing or invalid.")

struct i2c_hw_handle;

struct i2c_hw_handle {
    const uint32_t ctrl_id;
    const uint32_t base;
    const struct tegra_clk *clk_data;
    const struct tegra_rst *rst_data;
    const uint32_t irq_no;
    bool is_multimaster;
};

const struct i2c_hw_handle *i2c_get_hw_handle(uint32_t ctrl_id);

/* @brief BIT vector for Valid i2c ids for SPE cluster. */
#define VALID_I2C_IDS   (BIT(TEGRA234_I2C2) | BIT(TEGRA234_I2C8))

#endif
