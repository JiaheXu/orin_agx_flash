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
#include <stdint.h>                                  // for uint32_t, ...

/* Early FSP headers */
#include <misc/ct-assert.h>                          // for CT_ASSERT

/* Hardware headers */
#include <address_map_new.h>                         // for NV_ADDRESS_MAP_AON_GPIO_0_BASE, NV_ADDRESS_MAP_GPIO_CTL0_BASE
#include <argpio_sw.h>                               // for GPIO_A_ENABLE_CONFIG_00_0, ....

/* Late FSP headers */
#include <misc/macros.h>                             // for ARRAY_SIZE

/* Module-specific FSP headers */
#include <gpio/tegra-gpio-priv.h>                    // for struct gpio_irq_handler

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPIO__TEGRA_GPIO_PRIV_H, "Header file missing or invalid.")

static const uint32_t tegra_fsi_gpio_ctl_0_bases[] = {
    GPIO_U_ENABLE_CONFIG_00_0,
    GPIO_V_ENABLE_CONFIG_00_0,
};

static const uint32_t tegra_fsi_gpio_ctl_1_bases[] = {
    GPIO_S_ENABLE_CONFIG_00_0,
    GPIO_T_ENABLE_CONFIG_00_0,
//    GPIO_W_ENABLE_CONFIG_00_0,
};

struct tegra_gpio_id tegra_fsi_gpio_ctl_0_id = {
    .base_addr         = NV_ADDRESS_MAP_FSI_GPIO_CTL0_BASE,
    .bank_count        = ARRAY_SIZE(tegra_fsi_gpio_ctl_0_bases),
    .bank_bases        = tegra_fsi_gpio_ctl_0_bases
};

struct tegra_gpio_id tegra_fsi_gpio_ctl_1_id = {
    .base_addr         = NV_ADDRESS_MAP_FSI_GPIO_CTL1_BASE,
    .bank_count        = ARRAY_SIZE(tegra_fsi_gpio_ctl_1_bases),
    .bank_bases        = tegra_fsi_gpio_ctl_1_bases
};

struct tegra_gpio_id *const chips[] = {
    &tegra_fsi_gpio_ctl_0_id,
    &tegra_fsi_gpio_ctl_1_id
};
