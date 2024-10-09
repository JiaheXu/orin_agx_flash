/* Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef FSI__CAR_PLL_H
#define FSI__CAR_PLL_H
#define FSP__FSI__CAR_PLL_H             1

/* Compiler headers */
#include <stdint.h>                     // for uint8_t, uint64_t, uint32_t
#include <stdbool.h>                    // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <error/common-errors.h>        // for error_t

/* Module-specific FSP headers */
#include <car/car-pll.h>                // for car_pll_spec, ...


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
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_PLL_H, "Header file missing or invalid.")

/**
 * @brief FSI CAR PLL voltage/frequency configuration:
 *  Use the defines to select the frequency for the specific
 *  voltage when calling fsi_pll_init.
 */
#define FSI_CAR_PLL_550mV_633MHz        0U
#define FSI_CAR_PLL_600mV_897MHz        1U
#define FSI_CAR_PLL_670mV_1300MHz       2U
#define FSI_CAR_PLL_720mV_1300MHz       3U
#define FSI_CAR_PLL_850mV_1610MHz       4U
#define FSI_CAR_PLL_940mV_1610MHz       5U

#define FSI_CAR_PLL_550mV_550MHz        6U
#define FSI_CAR_PLL_600mV_780MHz        7U
#define FSI_CAR_PLL_670mV_1130MHz       8U
#define FSI_CAR_PLL_720mV_1130MHz       9U
#define FSI_CAR_PLL_850mV_1400MHz       10U
#define FSI_CAR_PLL_940mV_1400MHz       11U

#define FSI_CAR_PLL_0mV_0MHz_NO_INIT    12U


/**
 * @brief PLL register configuration values typically used to
 *        write directly to PLL registers. This can be used
 *        during PLL initialization or fault recovery.
 */
struct fsi_pll_init_cfg {
    int64_t hz;
    uint8_t divm;
    uint8_t divp;
    uint8_t divn;
    uint16_t divn_frac;
    uint16_t frac_step;
    uint32_t reg_misc_2;
    uint32_t reg_misc_3;
    uint32_t reg_misc_4;
    uint32_t reg_misc_5;
    uint32_t reg_misc_6;
};

/**
 * @brief Read FSI PLL enable status.
 *
 * @return True or false.
 */
bool fsi_pll_is_enabled(void);

/**
 * @brief Enable FSI PLL.
 *
 * DEPRECATED! Use fsi_clk_enable with FSI_CLK_ID_PLL instead.
 *
 * @return E_SUCCESS
 *         E_CAR_PLL_NO_LOCK
 */
error_t fsi_pll_enable(void);

/**
 * @brief Disable FSI PLL.
 *
 * DEPRECATED! Use fsi_clk_disable with FSI_CLK_ID_PLL instead.
 *
 * @return E_SUCCESS
 */
error_t fsi_pll_disable(void);

/**
 * @brief Set FSI PLL frequency.
 *
 * DEPRECATED! Use fsi_clk_set_rate with FSI_CLK_ID_PLL instead.
 *
 * @param hz_out Frequency in hertz.
 * @return E_SUCCESS
 *         E_CAR_INVALID_PARAM
 */
error_t fsi_pll_set_rate(int64_t hz_out);

/**
 * @brief Get FSI PLL frequency.
 *
 * DEPRECATED! Use fsi_clk_get_rate with FSI_CLK_ID_PLL instead.
 *
 * @param hz_out The pointer to where the frequency will be
 *               written.
 * @return E_SUCCESS
 */
error_t fsi_pll_get_rate(int64_t *hz_out);

/**
 * @brief Used to read the current car_pll_spec structure
 *        of values.
 *
 * DEPRECATED! This is a debug/test feature.
 *
 * @param spec Pointer to the car_pll_spec structure that will
 *             be populated with the current values.
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 */
error_t fsi_pll_spec_get(struct car_pll_spec *spec);

/**
 * @brief Used to provide an alternate car_pll_spec structure of
 *        values.
 *
 * DEPRECATED! This is a debug/test feature.
 *
 * @param spec Pointer to the car_pll_spec structure. If this
 *             pointer is NULL then the default car_pll_spec
 *             structure will be used. This is the mechanism to
 *             reset to default values.
 * @return E_SUCCESS
 */
error_t fsi_pll_spec_set(struct car_pll_spec *spec);

/**
 * @brief Used to provide a table of alternate car_pll_cfg
 *        structure values.
 *
 * DEPRECATED! This is a debug/test feature.
 *
 * @param cfgs Pointer to the car_pll_cfg structure table. If
 *             this pointer is NULL then the default car_pll_cfg
 *             structure table will be used. This is the
 *             mechanism to reset to the default table.
 * @param cfgs_n The number of entries in the table. If this
 *               value is 0 the car_pll_cfg feature is disabled.
 * @return E_SUCCESS
 */
error_t fsi_pll_cfgs_set(struct car_pll_cfg *cfgs, uint32_t cfgs_n);

/**
 * @brief Read the PLL registers to populate the
 *        fsi_pll_init_cfg structure. Typically used for a
 *        read-modify-write to the PLL.
 *
 * DEPRECATED! This is a debug/test feature.
 *
 * @param cfg Pointer to the fsi_pll_init_cfg structure.
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 *         E_CAR_PLL_NO_INIT
 *         E_CAR_PLL_NO_LOCK
 */
error_t fsi_pll_init_cfg_rd(struct fsi_pll_init_cfg *cfg);

/**
 * @brief Write the fsi_pll_init_cfg values directly to the PLL
 *        registers. This can be used during PLL initialization
 *        or fault recovery.
 *
 * DEPRECATED! This is a debug/test feature.
 *
 * @param cfg Pointer to the fsi_pll_init_cfg structure.
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 *         E_CAR_PLL_NO_INIT
 *         E_CAR_PLL_NO_LOCK
 */
error_t fsi_pll_init_cfg_wr(struct fsi_pll_init_cfg *cfg);

/**
 * @brief Initialize FSI PLL.
 *
 * DEPRECATED! Just start using fsi_clk_ functions with
 *             FSI_CLK_ID_PLL instead.
 *
 * @param pll_vf The PLL Voltage/Frequency selector.
 * @return E_SUCCESS
 *         E_CAR_INVALID_PARAM
 */
error_t fsi_pll_init(uint32_t pll_vf);

#endif /* FSI__CAR_PLL_H */

