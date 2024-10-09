/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef I2C__I2C_CONFIG_H
#define I2C__I2C_CONFIG_H
#define FSP__I2C__I2C_CONFIG_H                          1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

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

/**
 * @brief I2C controller and bus configuration data.
 *
 * @ctrl_clk_rate  Controller clock rate in Hz. +
 *                    Valid range - <100000, 4294967295>
 * @bus_clk_rate   Bus clock rate in Hz. +
 *                    Standard Mode  - 100000 +
 *                    Fast Mode      - 400000 +
 *                    Fast Mode Plus - 1000000
 * @tlow           Interface timing prod setting parameter in us. +
 *                    Valid ranges: +
 *                    Standard Mode  - <4, 4294967295> +
 *                    Fast Mode      - <0, 4294967295> +
 *                    Fast Mode Plus - <0, 4294967295>
 * @thigh          Interface timing prod setting parameter in us. +
 *                    Valid Ranges: +
 *                    Standard Mode  - <4, 4294967295> +
 *                    Fast Mode      - <0, 4294967295> +
 *                    Fast Mode Plus - <0, 4294967295>
 * @hs_master_code High speed master code. +
 *                    Valid range - <0, 4294967295>
 */
struct i2c_config_data
{
    uint32_t ctrl_clk_rate;
    uint32_t bus_clk_rate;
#ifdef CUSTOM_TIMING_PARAMS
    uint32_t thigh;
    uint32_t tlow;
#endif
    uint32_t hs_master_code;
};

#endif        /* FSP_I2C_I2C_CONFIG_H */
