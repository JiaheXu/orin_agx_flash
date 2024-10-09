/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */
#include <stdint.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */
#include <soc-common/tegra-gpio-hw.h>                   // for tegra_gpio_id_main
#include <processor/gpio-tegra.h>

/* Module-specific FSP headers */
#include <gpio/tegra-gpio-priv.h>                       /* for struct tegra_gpio_id */

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

    CT_ASSERT(FSP__GPIO__TEGRA_GPIO_PRIV_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__PROCESSOR__GPIO_TEGRA_HW_H, "Header file missing or invalid.")

struct tegra_gpio_id * tegra_gpio_id_dce[MAX_GPIO_CONTROLLERS] = {
    &tegra_gpio_id_main,
    &tegra_gpio_id_aon,
};
