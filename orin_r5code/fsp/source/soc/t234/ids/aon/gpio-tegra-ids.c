/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */

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

/* Main GPIO controller */
extern struct tegra_gpio_id tegra_gpio_id_main;

/* AON GPIO controller */
extern struct tegra_gpio_id tegra_gpio_id_aon;

struct tegra_gpio_id *const chips[] = {
    &tegra_gpio_id_main,
    &tegra_gpio_id_aon
};
