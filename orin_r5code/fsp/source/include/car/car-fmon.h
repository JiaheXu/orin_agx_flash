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

#ifndef CAR__CAR_FMON_H
#define CAR__CAR_FMON_H
#define FSP__CAR__CAR_FMON_H            1

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
#include <car/car-clk.h>                // for clock structures, ...


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


/**
 * @file car/car-fmon.h
 * @brief Structures that are internal to the CAR driver
 */

#define CAR_FMON_RDWR_VERIFY_N          (3)


/**
 * @brief Structure defining frequency monitor (FMON)
 *        information
 */
struct car_fmon_count_inf {
    uint8_t cnt_sel_min;
    uint8_t cnt_sel_max;
    uint8_t cnt_sel_0;
    uint8_t cnt_sel_n;
    uint8_t cnt_sel_min_i;
    uint8_t cnt_sel_max_i;
    uint8_t cnt_sel_0_i;
    uint8_t cnt_ref_win_i;
    uint8_t cnt_ref_win_dc_chk_i;
};

struct car_fmon_inf {
    const struct car_fmon_count_inf *cnt;
    const struct car_clk_inf *clk;
    uint64_t clk_ref;
    uint64_t clk_err;
    uint32_t reg_thr_hi;
    uint32_t reg_thr_hi_msk;
    uint32_t reg_thr_lo;
    uint32_t reg_thr_lo_msk;
    uint32_t reg_ref_win_cnt;
    uint32_t reg_ref_win_cnt_msk;
    uint32_t reg_ref_win_dc_chk_cnt;
    uint32_t reg_ref_win_dc_chk_cnt_msk;
    uint32_t reg_flt_actn;
    uint32_t reg_flt_actn_msk;
    uint32_t reg_cfg;
    uint32_t reg_cfg_msk_rprt;
    uint32_t reg_cfg_msk_ref_en;
    uint32_t reg_cfg_msk_cntr_en;
    uint32_t reg_cfg_msk_en;
    uint32_t reg_en_sts;
    uint32_t reg_en_sts_msk;
    uint32_t reg_flt_sts;
    uint32_t reg_flt_sts_msk_clr;
    uint32_t reg_flt_sts_msk_ro;
    uint32_t reg_data_ctrl;
    uint32_t reg_data_ctrl_msk_sel;
    uint32_t reg_data_ctrl_msk_en;
    uint32_t reg_data_sts_done;
    uint32_t reg_data_sts_done_msk;
    uint32_t reg_data_sts_sel;
    uint32_t reg_data_sts_sel_msk;
    uint32_t reg_data_val;
    uint32_t reg_data_val_msk;
    uint32_t reg_cntr_ctrl;
    uint32_t reg_cntr_ctrl_msk_hld;
    uint32_t reg_cntr_ctrl_msk_clr;
};


/**
 * @brief Read frequency monitor select counters.
 *
 * @param fmon Pointer to car_fmon_inf structure.
 * @param cnts Pointer to write counter data to.
 *             There must be enough memory for all of the
 *             car_fmon_inf->cnt_sel_* counter data + 1.
 *             Optional: Set cnts to NULL to reset the
 *             counters without reading them (returns
 *             E_SUCCESS).
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 *         E_CAR_FMON_NO_DATA
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t car_fmon_counts(const struct car_fmon_inf *fmon, uint32_t *cnts);

/**
 * @brief Calculate the input clock frequency (the frequency
 *        that FMON is monitoring).
 *
 * @param fmon Pointer to car_fmon_inf structure.
 * @param cnts Pointer to write select counter data to.
 *             There must be enough memory for all of the
 *             car_fmon_inf->cnt_sel_* counter data + 1.
 *             Optional: Set cnts to NULL to reset the
 *             counters without reading them (returns
 *             E_SUCCESS).
 * @param hz Pointer to write the calculated frequency to.
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 *         E_CAR_FMON_NO_DATA
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t car_fmon_clk_hz_calc(const struct car_fmon_inf *fmon, uint32_t *cnts, uint64_t *hz);

/**
 * @brief Get frequency monitor (FMON) default hardware timeout.
 *
 * The function returns the amount of time in microseconds that
 * should be allowed for the FMON hardware to process the
 * programming.
 *
 * @param fmon Pointer to car_fmon_inf structure.
 * @return Timeout in microseconds.
 */
uint64_t car_fmon_timeout_us(const struct car_fmon_inf *fmon);

/**
 * @brief Frequency Monitor (FMON) enabled?.
 *
 * @retval  true/false
 */
bool car_fmon_is_enabled(const struct car_fmon_inf *fmon);

/**
 * @brief Read Frequency Monitor (FMON) enable status register
 *        (reg_en_sts).
 *
 * @param fmon Pointer to car_fmon_inf structure.
 * @return FMON enable status register data.
 */
uint32_t car_fmon_enable_status(const struct car_fmon_inf *fmon);

/**
 * @brief  Frequency Monitor (FMON) startup.
 *
 * @param fmon Pointer to car_fmon_inf structure.
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 */
error_t car_fmon_startup(const struct car_fmon_inf *fmon);

/**
 * @brief  Frequency Monitor (FMON) shutdown.
 *
 * @param fmon Pointer to car_fmon_inf structure.
 * @return E_SUCCESS
 *         E_CAR_NULL_PTR
 *         E_CAR_FMON_HW_LOGIC
 *         E_CAR_FMON_HW_TIMEOUT
 *         E_CAR_FMON_FAULT
 */
error_t car_fmon_shutdown(const struct car_fmon_inf *fmon);

#endif /* CAR__CAR_FMON_H */

