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

#ifndef FSI__CAR_CLK_H
#define FSI__CAR_CLK_H
#define FSP__FSI__CAR_CLK_H             1

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
#include <car/car-clk.h>                // for clock functions, ...


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
CT_ASSERT(FSP__CAR__CAR_CLK_H, "Header file missing or invalid.")


#define Hz                              (1LL)
#define kHz                             (Hz * 1000)
#define MHz                             (kHz * 1000)
#define GHz                             (MHz * 1000)

#define FSI_CLK_XTAL_HZ                 40000000ULL /* 40Mhz */
#define FSI_CLK_VREFRO_HZ               50000000ULL /* 50Mhz */

#define FSI_CLK_ID_XTAL                 0U
#define FSI_CLK_ID_PLL                  1U
#define FSI_CLK_ID_CHSM_CPU             2U
#define FSI_CLK_ID_CSITE                3U
#define FSI_CLK_ID_FABRIC_CLK           4U
#define FSI_CLK_ID_FABRIC_PCLK          5U
#define FSI_CLK_ID_JTAG                 6U
#define FSI_CLK_ID_LA                   7U
#define FSI_CLK_ID_MC                   8U
#define FSI_CLK_ID_SE                   9U
#define FSI_CLK_ID_SE_FR                10U
#define FSI_CLK_ID_SPI0                 11U
#define FSI_CLK_ID_SRAM_CLK             12U
#define FSI_CLK_ID_SRAM_PCLK            13U
#define FSI_CLK_ID_TS                   14U
#define FSI_CLK_ID_UART0                15U
#define FSI_CLK_ID_N                    16U

extern const struct car_clk_inf fsi_clk_infs[FSI_CLK_ID_N];

/**
 * @brief Enable a clock.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @return E_SUCCESS
 *         E_CAR_INVALID_CLOCK_ID
 */
error_t fsi_clk_enable(uint32_t clk_id);

/**
 * @brief Disable a clock.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @return E_SUCCESS
 *         E_CAR_INVALID_CLOCK_ID
 */
error_t fsi_clk_disable(uint32_t clk_id);

/**
 * @brief Get the clock enable status.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @return True or false.
 */
bool fsi_clk_is_enabled(uint32_t clk_id);

/**
 * @brief Set the clock rate.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @param hz_out The requested clock rate in hertz.
 * @return E_SUCCESS
 *         E_CAR_INVALID_CLOCK_ID
 */
error_t fsi_clk_set_rate(uint32_t clk_id, int64_t hz_out);

/**
 * @brief Get the clock rate.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @param hz_out The pointer to put the clock rate in hertz.
 * @return E_SUCCESS
 *         E_CAR_INVALID_CLOCK_ID
 */
error_t fsi_clk_get_rate(uint32_t clk_id, int64_t *hz_out);


// Bring-up specific functions:

bool fsi_clk_fmon_is_enabled(void);
void fsi_clk_fmon_enable(void);
void fsi_clk_fmon_disable(void);

/**
 * @brief Set divider.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @param div The div value.
 * @return E_SUCCESS
 *         E_CAR_INVALID_CLOCK_ID
 */
error_t fsi_clk_set_div(uint32_t clk_id, uint32_t div);

/**
 * @brief  Get divider.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @param div The pointer to put the divider value.
 * @return E_SUCCESS
 *         E_CAR_INVALID_CLOCK_ID
 */
error_t fsi_clk_get_div(uint32_t clk_id, uint32_t *div);

/**
 * @brief Get CAR error_t string.
 *
 * @param err The error_t returned from a CAR function.
 * @return The string describing the error_t code.
 */
const char *fsi_car_get_err_str(error_t err);

#endif /* FSI__CAR_CLK_H */

