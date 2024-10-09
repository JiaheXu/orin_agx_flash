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

/* Compiler headers */
#include <stddef.h>                     // for NULL
#include <stdbool.h>                    // for bool, false
#include <stdint.h>                     // for uint32_t, uint8_t, UINT8_MAX, UIN...

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <soc-common/hw-const.h>        /* Must appear before any hwinc files */

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for E_SUCCESS, error_t
#include <misc/bitops.h>                // for BIT, bit_number, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for register access functions...
#include <car/car-math.h>               // for safe math functions...
#include <car/car-clk.h>                // for clock structures...
#include <car/car-fmon.h>               // for frequency monitor structures...
#include <port/car-port.h>              // for platform functions...

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_REG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_MATH_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_CLK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_FMON_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PORT__CAR_PORT_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
static error_t car_fmon_clk_hz_get(const struct car_fmon_inf *fmon, uint64_t *hz)
{
    error_t ret = E_CAR_NULL_PTR;

    if ((fmon->clk != NULL) && (hz != NULL)) {
        ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        if ((fmon->clk->op_rate != NULL) && (fmon->clk->op_rate->clk_hz_get != NULL)) {
            ret = fmon->clk->op_rate->clk_hz_get(fmon->clk, (int64_t *)hz);
        }
    }
    return ret;
}

SECTION_CAR_TEXT
bool car_fmon_is_enabled(const struct car_fmon_inf *fmon)
{
    uint32_t val;
    bool ret = false;

    if (fmon != NULL) {
        val = car_reg_rd_val(fmon->reg_cfg, fmon->reg_cfg_msk_en);
        if (val != 0U) {
            ret = true;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
uint32_t car_fmon_enable_status(const struct car_fmon_inf *fmon)
{
    uint32_t data = 0U;

    if (fmon != NULL) {
        data = car_reg_rd(fmon->reg_en_sts);
        data &= fmon->reg_en_sts_msk;
    }
    return data;
}

SECTION_CAR_TEXT
uint64_t car_fmon_timeout_us(const struct car_fmon_inf *fmon)
{
    uint64_t delay_us = 1ULL;
    uint64_t hz;
    error_t ret;

    if (fmon != NULL) {
        ret = car_fmon_clk_hz_get(fmon, &hz);
        if (ret == E_SUCCESS) {
            hz *= 6ULL;
            hz += (fmon->clk_ref << 2);
            hz += (fmon->clk_err << 2);
            hz = 1000000ULL / hz; /* Hz => us */
            if (delay_us < hz) {
                delay_us = hz;
            }
        }
    }
    return delay_us;
}

SECTION_CAR_TEXT
static void car_fmon_delay_us(const struct car_fmon_inf *fmon)
{
    car_port_udelay(car_fmon_timeout_us(fmon));
}

SECTION_CAR_TEXT
static error_t car_fmon_reg_rdwr_verify(uint32_t reg, uint32_t mask, uint32_t val, uint64_t to_us)
{
    uint64_t ticks_start;
    uint64_t ticks_us;
    uint32_t i;
    error_t ret = E_CAR_FMON_HW_TIMEOUT;

    for (i = 0; i < CAR_FMON_RDWR_VERIFY_N; i = i + 1U) {
        car_reg_rdwr_val(reg, mask, val);
        ticks_start = car_port_get_time_ticks();
        do {
            if (car_reg_rd_val(reg, mask) == val) {
                ret = E_SUCCESS;
                goto out;
            }

            ticks_us = car_port_get_time_delta_us(ticks_start);
        } while (ticks_us < to_us);
    }

out:
    return ret;
}

SECTION_CAR_TEXT
static error_t car_fmon_count_load_en(const struct car_fmon_inf *fmon, bool enable)
{
    uint64_t ticks_start;
    uint64_t ticks_us;
    error_t ret = E_CAR_NULL_PTR;

    if (fmon != NULL) {
        if (enable == true) {
            /* load_data = 0x1 */
            car_reg_rdwr_val(fmon->reg_data_ctrl, fmon->reg_data_ctrl_msk_en, fmon->reg_data_ctrl_msk_en);
        } else {
            /* load_data = 0x0 && register_count_select = 0x0 */
            car_reg_wr(fmon->reg_data_ctrl, 0);
        }
        /* wait for load_data_done */
        ret = E_CAR_FMON_HW_TIMEOUT;
        ticks_start = car_port_get_time_ticks();
        do {
            ticks_us = car_port_get_time_delta_us(ticks_start);
            if (car_reg_rd_val(fmon->reg_data_sts_done, fmon->reg_data_sts_done_msk)) {
                ret = E_SUCCESS;
                break;
            }

            if (ticks_us > 3ULL) {
                break;
            }

            car_port_udelay(1ULL);
        } while (true);
    }

    return ret;
}

SECTION_CAR_TEXT
static error_t car_fmon_count_load(const struct car_fmon_inf *fmon,
                                   uint32_t *cnt, uint32_t sel)
{
    error_t ret;

    /* load_data = 0x0 */
    ret = car_fmon_count_load_en(fmon, false);
    if (ret == E_SUCCESS) {
        /* register count select */
        car_reg_rdwr_val(fmon->reg_data_ctrl, fmon->reg_data_ctrl_msk_sel, sel);
        /* load_data = 0x1 */
        ret = car_fmon_count_load_en(fmon, true);
        if (ret == E_SUCCESS) {
            /* check load_data_sel_status */
            if ((car_reg_rd_val(fmon->reg_data_sts_sel, fmon->reg_data_sts_sel_msk) == sel)) {
                /* read load data val */
                *cnt = car_reg_rd(fmon->reg_data_val);
                /* load_data = 0x0 */
                ret = car_fmon_count_load_en(fmon, false);
            } else {
                ret = E_CAR_FMON_HW_LOGIC;
            }
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t car_fmon_counts(const struct car_fmon_inf *fmon, uint32_t *cnts)
{
    uint32_t i;
    error_t ret = E_CAR_NULL_PTR;

    if (fmon != NULL) {
        if (car_fmon_is_enabled(fmon) == true) {
            if (cnts == NULL) {
                /* caller can clear counters without reading them */
                ret = E_SUCCESS;
                goto clear;
            }

            /* freeze counter storage */
            car_reg_rdwr_val(fmon->reg_cntr_ctrl, fmon->reg_cntr_ctrl_msk_hld, fmon->reg_cntr_ctrl_msk_hld);

            for (i = 0; i < fmon->cnt->cnt_sel_n; i = i + 1U) {
                ret = car_fmon_count_load(fmon, &cnts[fmon->cnt->cnt_sel_0_i + i], fmon->cnt->cnt_sel_0 + i);
                if (ret != E_SUCCESS) {
                    goto clear;
                }
            }

            ret = car_fmon_count_load(fmon, &cnts[fmon->cnt->cnt_sel_min_i], fmon->cnt->cnt_sel_min);
            ret |= car_fmon_count_load(fmon, &cnts[fmon->cnt->cnt_sel_max_i], fmon->cnt->cnt_sel_max);
            cnts[fmon->cnt->cnt_ref_win_i] = car_reg_rd_val(fmon->reg_ref_win_cnt,
                                                            fmon->reg_ref_win_cnt_msk);
            cnts[fmon->cnt->cnt_ref_win_dc_chk_i] = car_reg_rd_val(fmon->reg_ref_win_dc_chk_cnt,
                                                                   fmon->reg_ref_win_dc_chk_cnt_msk);
clear:
            /* clear counts */
            car_reg_rdwr_val(fmon->reg_cntr_ctrl, fmon->reg_cntr_ctrl_msk_clr, fmon->reg_cntr_ctrl_msk_clr);
            /* unfreeze counter storage */
            car_reg_rdwr_val(fmon->reg_cntr_ctrl, fmon->reg_cntr_ctrl_msk_hld, 0);
        } else {
            ret = E_CAR_FMON_NO_DATA;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t car_fmon_clk_hz_calc(const struct car_fmon_inf *fmon, uint32_t *cnts, uint64_t *hz)
{
    int64_t cnt;
    int64_t ref_win;
    uint32_t i;
    uint32_t n;
    error_t ret = car_fmon_counts(fmon, cnts);

    if ((ret == E_SUCCESS) && (cnts != NULL)) {
        cnt = 0;
        n = 0;
        for (i = 0; i < fmon->cnt->cnt_sel_n; i = i + 1U) {
            if ((cnts[fmon->cnt->cnt_sel_0_i + i] != 0) && (cnts[fmon->cnt->cnt_sel_0_i + i] != UINT32_MAX)) {
                cnt += (int64_t)cnts[fmon->cnt->cnt_sel_0_i + i];
                n = n + 1U;
            }
        }

        if (n > 0) {
            cnt *= (int64_t)fmon->clk_ref;
            ref_win = (int64_t)cnts[fmon->cnt->cnt_ref_win_i];
            ref_win *= (int64_t)n;
            *hz = (uint64_t)car_div_s64(cnt, ref_win);
        } else {
            ret = E_CAR_FMON_NO_DATA;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t car_fmon_shutdown(const struct car_fmon_inf *fmon)
{
    uint32_t data;
    error_t ret = E_CAR_NULL_PTR;

    if (fmon != NULL) {
        data = (fmon->reg_cfg_msk_rprt |
                fmon->reg_cfg_msk_ref_en |
                fmon->reg_cfg_msk_cntr_en |
                fmon->reg_cfg_msk_en);
        car_reg_rdwr_val(fmon->reg_cfg, data, 0);
        car_fmon_delay_us(fmon);
        data = car_fmon_enable_status(fmon);
        if (data == 0U) {
            ret = E_SUCCESS;
        } else {
            ret = E_CAR_FMON_HW_LOGIC;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t car_fmon_startup(const struct car_fmon_inf *fmon)
{
    uint64_t delay_us;
    uint32_t thr_lo = 0U;
    uint32_t thr_hi = UINT32_MAX;
    uint32_t data;
    error_t ret;

    ret = car_fmon_shutdown(fmon);
    if (ret == E_SUCCESS) {
        delay_us = car_fmon_timeout_us(fmon);
        /* monitoring window period */
        ret = car_fmon_reg_rdwr_verify(fmon->reg_thr_lo, fmon->reg_thr_lo_msk, thr_lo, delay_us);
        ret |= car_fmon_reg_rdwr_verify(fmon->reg_thr_hi, fmon->reg_thr_hi_msk, thr_hi, delay_us);
        /* fault action */
        ret |= car_fmon_reg_rdwr_verify(fmon->reg_flt_actn, fmon->reg_flt_actn_msk, fmon->reg_flt_actn_msk, delay_us);
        /* configuration */
        car_reg_rdwr_val(fmon->reg_cfg, fmon->reg_cfg_msk_rprt, fmon->reg_cfg_msk_rprt);
        car_reg_rdwr_val(fmon->reg_cfg, fmon->reg_cfg_msk_ref_en, fmon->reg_cfg_msk_ref_en);
        car_reg_rdwr_val(fmon->reg_cfg, fmon->reg_cfg_msk_cntr_en, fmon->reg_cfg_msk_cntr_en);
        car_reg_rdwr_val(fmon->reg_cfg, fmon->reg_cfg_msk_en, fmon->reg_cfg_msk_en);
        /* verify */
        data = (fmon->reg_cfg_msk_rprt |
                fmon->reg_cfg_msk_ref_en |
                fmon->reg_cfg_msk_cntr_en |
                fmon->reg_cfg_msk_en);
        if ((car_reg_rd(fmon->reg_cfg) & data) != data) {
            ret = E_CAR_FMON_HW_LOGIC;
            goto out;
        }

        /* monitor enable status with timeout */
        car_port_udelay(delay_us);
        data = car_fmon_enable_status(fmon);
        if (data != fmon->reg_en_sts_msk) {
            ret = E_CAR_FMON_HW_LOGIC;
            goto out;
        }

        /* test for fault */
        data = car_reg_rd(fmon->reg_flt_sts);
        data &= fmon->reg_flt_sts_msk_ro;
        if (data != 0U) {
            ret = E_CAR_FMON_FAULT;
        }
    }
out:
    return ret;
}

