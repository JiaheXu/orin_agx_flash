/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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
#include <processor/tegra-gpio-hw-params.h>          // for TEGRA_MAIN_GPIO_ID0_IRQ, TEGRA_GPIO_IRQ_STATUS_REG, ...

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
CT_ASSERT(FSP__PROCESSOR__GPIO_TEGRA_HW_PARAMS_H, "Header file missing or invalid.")

#define TEGRA_GPIO_IRQ_STATUS_REG       (GPIO_N_INTERRUPT_STATUS_G4_0 - \
                                         GPIO_N_ENABLE_CONFIG_00_0)

static const uint32_t tegra_gpio_main_bases[] = {
    GPIO_A_ENABLE_CONFIG_00_0,
    GPIO_B_ENABLE_CONFIG_00_0,
    GPIO_C_ENABLE_CONFIG_00_0,
    GPIO_D_ENABLE_CONFIG_00_0,
    GPIO_E_ENABLE_CONFIG_00_0,
    GPIO_F_ENABLE_CONFIG_00_0,
    GPIO_G_ENABLE_CONFIG_00_0,
    GPIO_H_ENABLE_CONFIG_00_0,
    GPIO_I_ENABLE_CONFIG_00_0,
    GPIO_J_ENABLE_CONFIG_00_0,
    GPIO_K_ENABLE_CONFIG_00_0,
    GPIO_L_ENABLE_CONFIG_00_0,
    GPIO_M_ENABLE_CONFIG_00_0,
    GPIO_N_ENABLE_CONFIG_00_0,
    GPIO_P_ENABLE_CONFIG_00_0,
    GPIO_Q_ENABLE_CONFIG_00_0,
    GPIO_R_ENABLE_CONFIG_00_0,
    GPIO_X_ENABLE_CONFIG_00_0,
    GPIO_Y_ENABLE_CONFIG_00_0,
    GPIO_Z_ENABLE_CONFIG_00_0,
    GPIO_AC_ENABLE_CONFIG_00_0,
    GPIO_AD_ENABLE_CONFIG_00_0,
    GPIO_AE_ENABLE_CONFIG_00_0,
    GPIO_AF_ENABLE_CONFIG_00_0,
    GPIO_AG_ENABLE_CONFIG_00_0,
};

static struct gpio_irq_handler tegra_gpio_main_handlers[
                               ARRAY_SIZE(tegra_gpio_main_bases) *
                               GPIOS_PER_BANK];

static uint8_t main_bank_irq_status[ARRAY_SIZE(tegra_gpio_main_bases)] = {0};

static uint32_t tegra_main_gpio_id_irqs[] = {
    TEGRA_MAIN_GPIO_ID0_IRQ,
    TEGRA_MAIN_GPIO_ID1_IRQ,
    TEGRA_MAIN_GPIO_ID2_IRQ,
    TEGRA_MAIN_GPIO_ID3_IRQ,
    TEGRA_MAIN_GPIO_ID4_IRQ,
    TEGRA_MAIN_GPIO_ID5_IRQ,
};

struct tegra_gpio_id tegra_gpio_id_main = {
    .base_addr         = NV_ADDRESS_MAP_GPIO_CTL_BASE,
    .bank_count        = ARRAY_SIZE(tegra_gpio_main_bases),
    .bank_bases        = tegra_gpio_main_bases,
    .irqs              = tegra_main_gpio_id_irqs,
    .nirqs             = ARRAY_SIZE(tegra_main_gpio_id_irqs),
    .irq_handlers      = tegra_gpio_main_handlers,
    .irq_status_offset = TEGRA_GPIO_IRQ_STATUS_REG,
    .bank_irq_status   = main_bank_irq_status,
    .isr_status        = true,
};

