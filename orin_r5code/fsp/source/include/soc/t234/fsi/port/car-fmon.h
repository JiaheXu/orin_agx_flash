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

#ifndef FSI__CAR_FMON_H
#define FSI__CAR_FMON_H
#define FSP__FSI__CAR_FMON_H            1

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


/* FMON IDs */
#define FSI_FMON_ID_PLL             0U
#define FSI_FMON_ID_CHSM            1U
#define FSI_FMON_ID_FABRIC          2U
#define FSI_FMON_ID_SPI             3U
#define FSI_FMON_ID_TS              4U
#define FSI_FMON_ID_XTAL            5U
#define FSI_FMON_ID_N               6U

extern const struct car_fmon_inf fsi_fmon_infs[FSI_FMON_ID_N];

/* Counter offsets to clk_count member of fmon_clk_count structure */
#define FSI_FMON_CLK_COUNT_MIN      (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_MIN)
#define FSI_FMON_CLK_COUNT_MAX      (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_MAX)
#define FSI_FMON_CLK_COUNT_LATEST   (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_COUNT_0_Latest_Count)
#define FSI_FMON_CLK_COUNT_1        (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_COUNT_1)
#define FSI_FMON_CLK_COUNT_2        (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_COUNT_2)
#define FSI_FMON_CLK_COUNT_3        (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_COUNT_3)
#define FSI_FMON_CLK_COUNT_EARLIEST (CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_COUNT_4_Earliest_Count)

struct fsi_fmon_cnts {
    uint32_t ref_win;
    uint32_t ref_win_dc_chk;
    uint32_t cnt_min;
    uint32_t cnt_max;
    uint32_t cnt_0_latest;
    uint32_t cnt_1;
    uint32_t cnt_2;
    uint32_t cnt_3;
    uint32_t cnt_4_earliest;
};


/**
 * @brief Calculate clock frequency from frequency monitor
 *        counters.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @param hz The pointer to put the frequency hertz.
 * @return E_SUCCESS
 *         E_CAR_FMON_NO_DATA
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t fsi_fmon_clk_hz_calc(uint32_t clk_id, uint64_t *hz);

/**
 * @brief Frequency Monitor (FMON) enabled?.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @retval  true/false
 */
bool fsi_fmon_is_enabled(uint32_t clk_id);

/**
 * @brief Frequency Monitor (FMON) startup.
 *
 * DEPRECATED! This is handled automatically by the FSP CAR
 *             driver.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @return E_SUCCESS
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t fsi_fmon_startup(uint32_t clk_id);

/**
 * @brief Frequency Monitor (FMON) shutdown.
 *
 * DEPRECATED! This is handled automatically by the FSP CAR
 *             driver.
 *
 * @param clk_id The FSI_CLK_ID_? define that is used to identify
 *               a clock.
 * @return E_SUCCESS
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t fsi_fmon_shutdown(uint32_t clk_id);

/**
 * @brief  Initialize frequency monitors
 *
 * DEPRECATED!
 *
 * @return E_SUCCESS
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t fsi_fmon_init(void);

#endif /* FSI__CAR_FMON_H */

