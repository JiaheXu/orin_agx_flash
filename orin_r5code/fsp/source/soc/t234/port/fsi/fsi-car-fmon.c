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
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <arclk_rst.h>              // for CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0...
#include <arintr_ctlr.h>            // for INTR_CTLR_CHANNEL0_SLICE0_IER_0...

/* Late FSP headers */
#include <misc/macros.h>            // for ARRAY_SIZE,...
#include <error/common-errors.h>    // for E_SUCCESS, error_t
#include <misc/bitops.h>            // for BIT, bit_number, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */
#include <car/car-sections.h>       // for SECTION_CAR_TEXT ...
#include <car/car-errors.h>         // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-fmon.h>           // for core FMON functions...
#include <port/car-port.h>              // for platform functions...
#include <port/car-clk.h>           // for clock frequency defines, ...
#include <port/car-fmon.h>          // for FMON API functions ...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_FMON_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PORT__CAR_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_FMON_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_CLK_H, "Header file missing or invalid.")


SECTION_CAR_RODATA
const struct car_fmon_count_inf fsi_fmon_count_inf = {
    .cnt_sel_min                = FSI_FMON_CLK_COUNT_MIN,
    .cnt_sel_max                = FSI_FMON_CLK_COUNT_MAX,
    .cnt_sel_0                  = FSI_FMON_CLK_COUNT_LATEST,
    .cnt_sel_n                  = (FSI_FMON_CLK_COUNT_EARLIEST - FSI_FMON_CLK_COUNT_LATEST) + 1,
    .cnt_sel_min_i              = (FSI_FMON_CLK_COUNT_MIN + 1),
    .cnt_sel_max_i              = (FSI_FMON_CLK_COUNT_MAX + 1),
    .cnt_sel_0_i                = (FSI_FMON_CLK_COUNT_LATEST + 1),
    .cnt_ref_win_i              = 0,
    .cnt_ref_win_dc_chk_i       = 1,
};

const struct car_fmon_inf fsi_fmon_infs[] = {
    [FSI_FMON_ID_PLL] = {
        .clk                    = &fsi_clk_infs[FSI_CLK_ID_PLL],
        .clk_ref                = FSI_CLK_XTAL_HZ,
        .clk_err                = FSI_CLK_XTAL_HZ,
        .reg_thr_hi             = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_PLL_FSI_0,
        .reg_thr_hi_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_PLL_FSI_0_PLL_FSI_FMON_COUNT_THRESH_HIGH_FIELD,
        .reg_thr_lo             = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_PLL_FSI_0,
        .reg_thr_lo_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_PLL_FSI_0_PLL_FSI_FMON_COUNT_THRESH_LOW_FIELD,
        .reg_ref_win_cnt        = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_PLL_FSI_0,
        .reg_ref_win_cnt_msk    = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_PLL_FSI_0_PLL_FSI_FMON_REF_WINDOW_COUNT_FIELD,
        .reg_ref_win_dc_chk_cnt = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_PLL_FSI_0,
        .reg_ref_win_dc_chk_cnt_msk = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_PLL_FSI_0_PLL_FSI_FMON_REF_WINDOW_DC_CHECK_COUNT_FIELD,
        .reg_flt_actn           = CLK_RST_CONTROLLER_FMON_FAULT_ACTION_PLL_FSI_0,
        .reg_flt_actn_msk       = 0,
        .reg_cfg                = CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0,
        .reg_cfg_msk_rprt       = (CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_REPORT_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_REPORT_LOW_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_REPORT_HIGH_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_REPORT_DC_FAULT_VIOL_FIELD),
        .reg_cfg_msk_ref_en     = CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_REF_CLK_WINDOW_EN_FIELD,
        .reg_cfg_msk_cntr_en    = CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_ENABLE_FMON_COUNTER_FIELD,
        .reg_cfg_msk_en         = CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0_PLL_FSI_FMON_ENABLE_FIELD,
        .reg_en_sts             = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_PLL_FSI_0,
        .reg_en_sts_msk         = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_PLL_FSI_0_READ_MASK,
        .reg_flt_sts            = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0,
        .reg_flt_sts_msk_clr    = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0_PLL_FSI_FMON_FAULT_CLEAR_FIELD,
        .reg_flt_sts_msk_ro     = (CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0_PLL_FSI_FMON_FAULT_OUT_STATUS_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0_PLL_FSI_FMON_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0_PLL_FSI_FMON_COUNT_HIGHER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0_PLL_FSI_FMON_COUNT_LOWER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_PLL_FSI_0_PLL_FSI_FMON_DC_FAULT_FIELD),
        .reg_data_ctrl          = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0,
        .reg_data_ctrl_msk_sel  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_FIELD,
        .reg_data_ctrl_msk_en   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_FIELD,
        .reg_data_sts_done      = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_PLL_FSI_0,
        .reg_data_sts_done_msk  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_DONE_FIELD,
        .reg_data_sts_sel       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_PLL_FSI_0,
        .reg_data_sts_sel_msk   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_SEL_STATUS_FIELD,
        .reg_data_val           = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_PLL_FSI_0,
        .reg_data_val_msk       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_PLL_FSI_0_PLL_FSI_FMON_LOAD_DATA_VAL_FIELD,
        .reg_cntr_ctrl          = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_PLL_FSI_0,
        .reg_cntr_ctrl_msk_hld  = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_PLL_FSI_0_PLL_FSI_FMON_HOLD_COUNT_SAMPLES_FIELD,
        .reg_cntr_ctrl_msk_clr  = (CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_PLL_FSI_0_PLL_FSI_FMON_MIN_MAX_LASTN_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_PLL_FSI_0_PLL_FSI_FMON_MASK_MIN_MAX_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_PLL_FSI_0_PLL_FSI_FMON_CLEAR_FMON_COUNTER_FIELD),
        .cnt                    = &fsi_fmon_count_inf,
    },
    [FSI_FMON_ID_CHSM] = {
        .clk                    = &fsi_clk_infs[FSI_CLK_ID_CHSM_CPU],
        .clk_ref                = FSI_CLK_XTAL_HZ,
        .clk_err                = FSI_CLK_XTAL_HZ,
        .reg_thr_hi             = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_CHSM_CPU_0,
        .reg_thr_hi_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_COUNT_THRESH_HIGH_FIELD,
        .reg_thr_lo             = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_CHSM_CPU_0,
        .reg_thr_lo_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_COUNT_THRESH_LOW_FIELD,
        .reg_ref_win_cnt        = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_CHSM_CPU_0,
        .reg_ref_win_cnt_msk    = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REF_WINDOW_COUNT_FIELD,
        .reg_ref_win_dc_chk_cnt = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_CHSM_CPU_0,
        .reg_ref_win_dc_chk_cnt_msk = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REF_WINDOW_DC_CHECK_COUNT_FIELD,
        .reg_flt_actn           = CLK_RST_CONTROLLER_FMON_FAULT_ACTION_FSI_CHSM_CPU_0,
        .reg_flt_actn_msk       = 0,
        .reg_cfg                = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0,
        .reg_cfg_msk_rprt       = (CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REPORT_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REPORT_LOW_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REPORT_HIGH_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REPORT_DC_FAULT_VIOL_FIELD),
        .reg_cfg_msk_ref_en     = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_REF_CLK_WINDOW_EN_FIELD,
        .reg_cfg_msk_cntr_en    = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_ENABLE_FMON_COUNTER_FIELD,
        .reg_cfg_msk_en         = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_ENABLE_FIELD,
        .reg_en_sts             = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_CHSM_CPU_0,
        .reg_en_sts_msk         = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_CHSM_CPU_0_READ_MASK,
        .reg_flt_sts            = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0,
        .reg_flt_sts_msk_clr    = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_FAULT_CLEAR_FIELD,
        .reg_flt_sts_msk_ro     = (CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_FAULT_OUT_STATUS_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_COUNT_HIGHER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_COUNT_LOWER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_DC_FAULT_FIELD),
        .reg_data_ctrl          = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_CHSM_CPU_0,
        .reg_data_ctrl_msk_sel  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_LOAD_DATA_SEL_FIELD,
        .reg_data_ctrl_msk_en   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_LOAD_DATA_FIELD,
        .reg_data_sts_done      = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_CHSM_CPU_0,
        .reg_data_sts_done_msk  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_LOAD_DATA_DONE_FIELD,
        .reg_data_sts_sel       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_CHSM_CPU_0,
        .reg_data_sts_sel_msk   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_LOAD_DATA_SEL_STATUS_FIELD,
        .reg_data_val           = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_CHSM_CPU_0,
        .reg_data_val_msk       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_LOAD_DATA_VAL_FIELD,
        .reg_cntr_ctrl          = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_CHSM_CPU_0,
        .reg_cntr_ctrl_msk_hld  = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_HOLD_COUNT_SAMPLES_FIELD,
        .reg_cntr_ctrl_msk_clr  = (CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_MIN_MAX_LASTN_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_MASK_MIN_MAX_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_CHSM_CPU_0_FSI_CHSM_CPU_FMON_CLEAR_FMON_COUNTER_FIELD),
        .cnt                    = &fsi_fmon_count_inf,
    },
    [FSI_FMON_ID_FABRIC] = {
        .clk                    = &fsi_clk_infs[FSI_CLK_ID_FABRIC_CLK],
        .clk_ref                = FSI_CLK_XTAL_HZ,
        .clk_err                = FSI_CLK_XTAL_HZ,
        .reg_thr_hi             = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_FABRIC_0,
        .reg_thr_hi_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_FABRIC_0_FSI_FABRIC_FMON_COUNT_THRESH_HIGH_FIELD,
        .reg_thr_lo             = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_FABRIC_0,
        .reg_thr_lo_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_FABRIC_0_FSI_FABRIC_FMON_COUNT_THRESH_LOW_FIELD,
        .reg_ref_win_cnt        = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_FABRIC_0,
        .reg_ref_win_cnt_msk    = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_FABRIC_0_FSI_FABRIC_FMON_REF_WINDOW_COUNT_FIELD,
        .reg_ref_win_dc_chk_cnt = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_FABRIC_0,
        .reg_ref_win_dc_chk_cnt_msk = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_FABRIC_0_FSI_FABRIC_FMON_REF_WINDOW_DC_CHECK_COUNT_FIELD,
        .reg_flt_actn           = CLK_RST_CONTROLLER_FMON_FAULT_ACTION_FSI_FABRIC_0,
        .reg_flt_actn_msk       = 0,
        .reg_cfg                = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0,
        .reg_cfg_msk_rprt       = (CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_REPORT_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_REPORT_LOW_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_REPORT_HIGH_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_REPORT_DC_FAULT_VIOL_FIELD),
        .reg_cfg_msk_ref_en     = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_REF_CLK_WINDOW_EN_FIELD,
        .reg_cfg_msk_cntr_en    = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_ENABLE_FMON_COUNTER_FIELD,
        .reg_cfg_msk_en         = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_FABRIC_0_FSI_FABRIC_FMON_ENABLE_FIELD,
        .reg_en_sts             = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_FABRIC_0,
        .reg_en_sts_msk         = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_FABRIC_0_READ_MASK,
        .reg_flt_sts            = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0,
        .reg_flt_sts_msk_clr    = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0_FSI_FABRIC_FMON_FAULT_CLEAR_FIELD,
        .reg_flt_sts_msk_ro     = (CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0_FSI_FABRIC_FMON_FAULT_OUT_STATUS_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0_FSI_FABRIC_FMON_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0_FSI_FABRIC_FMON_COUNT_HIGHER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0_FSI_FABRIC_FMON_COUNT_LOWER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_FABRIC_0_FSI_FABRIC_FMON_DC_FAULT_FIELD),
        .reg_data_ctrl          = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_FABRIC_0,
        .reg_data_ctrl_msk_sel  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_FABRIC_0_FSI_FABRIC_FMON_LOAD_DATA_SEL_FIELD,
        .reg_data_ctrl_msk_en   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_FABRIC_0_FSI_FABRIC_FMON_LOAD_DATA_FIELD,
        .reg_data_sts_done      = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_FABRIC_0,
        .reg_data_sts_done_msk  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_FABRIC_0_FSI_FABRIC_FMON_LOAD_DATA_DONE_FIELD,
        .reg_data_sts_sel       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_FABRIC_0,
        .reg_data_sts_sel_msk   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_FABRIC_0_FSI_FABRIC_FMON_LOAD_DATA_SEL_STATUS_FIELD,
        .reg_data_val           = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_FABRIC_0,
        .reg_data_val_msk       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_FABRIC_0_FSI_FABRIC_FMON_LOAD_DATA_VAL_FIELD,
        .reg_cntr_ctrl          = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_FABRIC_0,
        .reg_cntr_ctrl_msk_hld  = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_FABRIC_0_FSI_FABRIC_FMON_HOLD_COUNT_SAMPLES_FIELD,
        .reg_cntr_ctrl_msk_clr  = (CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_FABRIC_0_FSI_FABRIC_FMON_MIN_MAX_LASTN_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_FABRIC_0_FSI_FABRIC_FMON_MASK_MIN_MAX_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_FABRIC_0_FSI_FABRIC_FMON_CLEAR_FMON_COUNTER_FIELD),
        .cnt                    = &fsi_fmon_count_inf,
    },
    [FSI_FMON_ID_SPI] = {
        .clk                    = &fsi_clk_infs[FSI_CLK_ID_SPI0],
        .clk_ref                = FSI_CLK_XTAL_HZ,
        .clk_err                = FSI_CLK_XTAL_HZ,
        .reg_thr_hi             = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_SPI_0,
        .reg_thr_hi_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_SPI_0_FSI_SPI_FMON_COUNT_THRESH_HIGH_FIELD,
        .reg_thr_lo             = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_SPI_0,
        .reg_thr_lo_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_SPI_0_FSI_SPI_FMON_COUNT_THRESH_LOW_FIELD,
        .reg_ref_win_cnt        = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_SPI_0,
        .reg_ref_win_cnt_msk    = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_SPI_0_FSI_SPI_FMON_REF_WINDOW_COUNT_FIELD,
        .reg_ref_win_dc_chk_cnt = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_SPI_0,
        .reg_ref_win_dc_chk_cnt_msk = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_SPI_0_FSI_SPI_FMON_REF_WINDOW_DC_CHECK_COUNT_FIELD,
        .reg_flt_actn           = CLK_RST_CONTROLLER_FMON_FAULT_ACTION_FSI_SPI_0,
        .reg_flt_actn_msk       = 0,
        .reg_cfg                = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0,
        .reg_cfg_msk_rprt       = (CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_REPORT_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_REPORT_LOW_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_REPORT_HIGH_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_REPORT_DC_FAULT_VIOL_FIELD),
        .reg_cfg_msk_ref_en     = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_REF_CLK_WINDOW_EN_FIELD,
        .reg_cfg_msk_cntr_en    = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_ENABLE_FMON_COUNTER_FIELD,
        .reg_cfg_msk_en         = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_SPI_0_FSI_SPI_FMON_ENABLE_FIELD,
        .reg_en_sts             = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_SPI_0,
        .reg_en_sts_msk         = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_SPI_0_READ_MASK,
        .reg_flt_sts            = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0,
        .reg_flt_sts_msk_clr    = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0_FSI_SPI_FMON_FAULT_CLEAR_FIELD,
        .reg_flt_sts_msk_ro     = (CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0_FSI_SPI_FMON_FAULT_OUT_STATUS_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0_FSI_SPI_FMON_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0_FSI_SPI_FMON_COUNT_HIGHER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0_FSI_SPI_FMON_COUNT_LOWER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_SPI_0_FSI_SPI_FMON_DC_FAULT_FIELD),
        .reg_data_ctrl          = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_SPI_0,
        .reg_data_ctrl_msk_sel  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_SPI_0_FSI_SPI_FMON_LOAD_DATA_SEL_FIELD,
        .reg_data_ctrl_msk_en   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_SPI_0_FSI_SPI_FMON_LOAD_DATA_FIELD,
        .reg_data_sts_done      = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_SPI_0,
        .reg_data_sts_done_msk  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_SPI_0_FSI_SPI_FMON_LOAD_DATA_DONE_FIELD,
        .reg_data_sts_sel       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_SPI_0,
        .reg_data_sts_sel_msk   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_SPI_0_FSI_SPI_FMON_LOAD_DATA_SEL_STATUS_FIELD,
        .reg_data_val           = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_SPI_0,
        .reg_data_val_msk       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_SPI_0_FSI_SPI_FMON_LOAD_DATA_VAL_FIELD,
        .reg_cntr_ctrl          = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_SPI_0,
        .reg_cntr_ctrl_msk_hld  = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_SPI_0_FSI_SPI_FMON_HOLD_COUNT_SAMPLES_FIELD,
        .reg_cntr_ctrl_msk_clr  = (CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_SPI_0_FSI_SPI_FMON_MIN_MAX_LASTN_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_SPI_0_FSI_SPI_FMON_MASK_MIN_MAX_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_SPI_0_FSI_SPI_FMON_CLEAR_FMON_COUNTER_FIELD),
        .cnt                    = &fsi_fmon_count_inf,
    },
    [FSI_FMON_ID_TS] = {
        .clk                    = &fsi_clk_infs[FSI_CLK_ID_TS],
        .clk_ref                = FSI_CLK_XTAL_HZ,
        .clk_err                = FSI_CLK_XTAL_HZ,
        .reg_thr_hi             = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_TS_0,
        .reg_thr_hi_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_TS_0_FSI_TS_FMON_COUNT_THRESH_HIGH_FIELD,
        .reg_thr_lo             = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_TS_0,
        .reg_thr_lo_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_TS_0_FSI_TS_FMON_COUNT_THRESH_LOW_FIELD,
        .reg_ref_win_cnt        = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_TS_0,
        .reg_ref_win_cnt_msk    = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_TS_0_FSI_TS_FMON_REF_WINDOW_COUNT_FIELD,
        .reg_ref_win_dc_chk_cnt = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_TS_0,
        .reg_ref_win_dc_chk_cnt_msk = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_TS_0_FSI_TS_FMON_REF_WINDOW_DC_CHECK_COUNT_FIELD,
        .reg_flt_actn           = CLK_RST_CONTROLLER_FMON_FAULT_ACTION_FSI_TS_0,
        .reg_flt_actn_msk       = 0,
        .reg_cfg                = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0,
        .reg_cfg_msk_rprt       = (CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_REPORT_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_REPORT_LOW_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_REPORT_HIGH_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_REPORT_DC_FAULT_VIOL_FIELD),
        .reg_cfg_msk_ref_en     = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_REF_CLK_WINDOW_EN_FIELD,
        .reg_cfg_msk_cntr_en    = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_ENABLE_FMON_COUNTER_FIELD,
        .reg_cfg_msk_en         = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_TS_0_FSI_TS_FMON_ENABLE_FIELD,
        .reg_en_sts             = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_TS_0,
        .reg_en_sts_msk         = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_TS_0_READ_MASK,
        .reg_flt_sts            = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0,
        .reg_flt_sts_msk_clr    = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0_FSI_TS_FMON_FAULT_CLEAR_FIELD,
        .reg_flt_sts_msk_ro     = (CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0_FSI_TS_FMON_FAULT_OUT_STATUS_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0_FSI_TS_FMON_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0_FSI_TS_FMON_COUNT_HIGHER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0_FSI_TS_FMON_COUNT_LOWER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_TS_0_FSI_TS_FMON_DC_FAULT_FIELD),
        .reg_data_ctrl          = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_TS_0,
        .reg_data_ctrl_msk_sel  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_TS_0_FSI_TS_FMON_LOAD_DATA_SEL_FIELD,
        .reg_data_ctrl_msk_en   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_TS_0_FSI_TS_FMON_LOAD_DATA_FIELD,
        .reg_data_sts_done      = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_TS_0,
        .reg_data_sts_done_msk  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_TS_0_FSI_TS_FMON_LOAD_DATA_DONE_FIELD,
        .reg_data_sts_sel       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_TS_0,
        .reg_data_sts_sel_msk   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_TS_0_FSI_TS_FMON_LOAD_DATA_SEL_STATUS_FIELD,
        .reg_data_val           = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_TS_0,
        .reg_data_val_msk       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_TS_0_FSI_TS_FMON_LOAD_DATA_VAL_FIELD,
        .reg_cntr_ctrl          = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_TS_0,
        .reg_cntr_ctrl_msk_hld  = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_TS_0_FSI_TS_FMON_HOLD_COUNT_SAMPLES_FIELD,
        .reg_cntr_ctrl_msk_clr  = (CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_TS_0_FSI_TS_FMON_MIN_MAX_LASTN_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_TS_0_FSI_TS_FMON_MASK_MIN_MAX_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_TS_0_FSI_TS_FMON_CLEAR_FMON_COUNTER_FIELD),
        .cnt                    = &fsi_fmon_count_inf,
    },
    [FSI_FMON_ID_XTAL] = {
        .clk                    = &fsi_clk_infs[FSI_CLK_ID_XTAL],
        .clk_ref                = FSI_CLK_VREFRO_HZ,
        .clk_err                = FSI_CLK_VREFRO_HZ,
        .reg_thr_hi             = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_XTAL_0,
        .reg_thr_hi_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_HIGH_FSI_XTAL_0_FSI_XTAL_FMON_COUNT_THRESH_HIGH_FIELD,
        .reg_thr_lo             = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_XTAL_0,
        .reg_thr_lo_msk         = CLK_RST_CONTROLLER_FMON_THRESHOLD_LOW_FSI_XTAL_0_FSI_XTAL_FMON_COUNT_THRESH_LOW_FIELD,
        .reg_ref_win_cnt        = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_XTAL_0,
        .reg_ref_win_cnt_msk    = CLK_RST_CONTROLLER_FMON_REF_WINDOW_COUNT_FSI_XTAL_0_FSI_XTAL_FMON_REF_WINDOW_COUNT_FIELD,
        .reg_ref_win_dc_chk_cnt = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_XTAL_0,
        .reg_ref_win_dc_chk_cnt_msk = CLK_RST_CONTROLLER_FMON_REF_WINDOW_DC_CHECK_COUNT_FSI_XTAL_0_FSI_XTAL_FMON_REF_WINDOW_DC_CHECK_COUNT_FIELD,
        .reg_flt_actn           = CLK_RST_CONTROLLER_FMON_FAULT_ACTION_FSI_XTAL_0,
        .reg_flt_actn_msk       = 0,
        .reg_cfg                = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0,
        .reg_cfg_msk_rprt       = (CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_REPORT_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_REPORT_LOW_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_REPORT_HIGH_THRESH_VIOL_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_REPORT_DC_FAULT_VIOL_FIELD),
        .reg_cfg_msk_ref_en     = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_REF_CLK_WINDOW_EN_FIELD,
        .reg_cfg_msk_cntr_en    = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_ENABLE_FMON_COUNTER_FIELD,
        .reg_cfg_msk_en         = CLK_RST_CONTROLLER_FMON_CONFIG_FSI_XTAL_0_FSI_XTAL_FMON_ENABLE_FIELD,
        .reg_en_sts             = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_XTAL_0,
        .reg_en_sts_msk         = CLK_RST_CONTROLLER_FMON_ENABLE_STATUS_FSI_XTAL_0_READ_MASK,
        .reg_flt_sts            = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0,
        .reg_flt_sts_msk_clr    = CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0_FSI_XTAL_FMON_FAULT_CLEAR_FIELD,
        .reg_flt_sts_msk_ro     = (CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0_FSI_XTAL_FMON_FAULT_OUT_STATUS_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0_FSI_XTAL_FMON_OVERFLOW_ERROR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0_FSI_XTAL_FMON_COUNT_HIGHER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0_FSI_XTAL_FMON_COUNT_LOWER_THRESH_HIGH_FAULT_FIELD |
                                   CLK_RST_CONTROLLER_FMON_FAULT_STATUS_FSI_XTAL_0_FSI_XTAL_FMON_DC_FAULT_FIELD),
        .reg_data_ctrl          = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_XTAL_0,
        .reg_data_ctrl_msk_sel  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_XTAL_0_FSI_XTAL_FMON_LOAD_DATA_SEL_FIELD,
        .reg_data_ctrl_msk_en   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_CTRL_FSI_XTAL_0_FSI_XTAL_FMON_LOAD_DATA_FIELD,
        .reg_data_sts_done      = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_XTAL_0,
        .reg_data_sts_done_msk  = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_0_FSI_XTAL_0_FSI_XTAL_FMON_LOAD_DATA_DONE_FIELD,
        .reg_data_sts_sel       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_XTAL_0,
        .reg_data_sts_sel_msk   = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_1_FSI_XTAL_0_FSI_XTAL_FMON_LOAD_DATA_SEL_STATUS_FIELD,
        .reg_data_val           = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_XTAL_0,
        .reg_data_val_msk       = CLK_RST_CONTROLLER_FMON_LOAD_DATA_STATUS_2_FSI_XTAL_0_FSI_XTAL_FMON_LOAD_DATA_VAL_FIELD,
        .reg_cntr_ctrl          = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_XTAL_0,
        .reg_cntr_ctrl_msk_hld  = CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_XTAL_0_FSI_XTAL_FMON_HOLD_COUNT_SAMPLES_FIELD,
        .reg_cntr_ctrl_msk_clr  = (CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_XTAL_0_FSI_XTAL_FMON_MIN_MAX_LASTN_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_XTAL_0_FSI_XTAL_FMON_MASK_MIN_MAX_COUNT_CLEAR_FIELD |
                                   CLK_RST_CONTROLLER_FMON_CLEAR_COUNTER_FSI_XTAL_0_FSI_XTAL_FMON_CLEAR_FMON_COUNTER_FIELD),
        .cnt                    = &fsi_fmon_count_inf,
    },
};

SECTION_CAR_DATA
static uint32_t fsi_clk_counts[sizeof(struct fsi_fmon_cnts)];

SECTION_CAR_TEXT
static struct car_fmon_inf *fsi_fmon_inf_get(uint32_t clk_id)
{
    struct car_fmon_inf *fmon = NULL;

    if (clk_id < ARRAY_SIZE(fsi_clk_infs)) {
        fmon = (struct car_fmon_inf *)fsi_clk_infs[clk_id].fmon;
    }
    return fmon;
}

bool fsi_fmon_is_enabled(uint32_t clk_id)
{
    struct car_fmon_inf *fmon = fsi_fmon_inf_get(clk_id);
    bool ret = false;

    if (fmon != NULL) {
        ret = car_fmon_is_enabled(fmon);
    }
    return ret;
}

error_t fsi_fmon_clk_hz_calc(uint32_t clk_id, uint64_t *hz)
{
    struct car_fmon_inf *fmon = fsi_fmon_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (fmon != NULL) {
        ret = car_fmon_clk_hz_calc(fmon, fsi_clk_counts, hz);
    }
    return ret;
}

error_t fsi_fmon_shutdown(uint32_t clk_id)
{
    struct car_fmon_inf *fmon = fsi_fmon_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (fmon != NULL) {
        ret = car_fmon_shutdown(fmon);
    }
    return ret;
}

error_t fsi_fmon_startup(uint32_t clk_id)
{
    struct car_fmon_inf *fmon = fsi_fmon_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (fmon != NULL) {
        ret = car_fmon_startup(fmon);
    }
    return ret;
}

error_t fsi_fmon_init(void)
{
    uint64_t ticks_start;
    uint64_t ticks_us;
    uint64_t delay_us;
    uint32_t data;
    uint32_t i;
    error_t ret = E_SUCCESS;

    ticks_start = car_port_get_time_ticks();
    i = 0;
    do {
        ticks_us = car_port_get_time_delta_us(ticks_start);
        data = car_fmon_enable_status(&fsi_fmon_infs[i]);
        if (data != 0U) {
            delay_us = car_fmon_timeout_us(&fsi_fmon_infs[i]);
            if (delay_us > ticks_us) {
                delay_us -= ticks_us;
                car_port_udelay(delay_us);
            } else {
                ret = E_CAR_FMON_HW_TIMEOUT;
                break;
            }
        } else {
            ret = car_fmon_counts(&fsi_fmon_infs[i], fsi_clk_counts);
            if (ret == E_SUCCESS) {
                if ((fsi_clk_counts[fsi_fmon_infs[i].cnt->cnt_sel_min_i] == UINT32_MAX) &&
                    (fsi_clk_counts[fsi_fmon_infs[i].cnt->cnt_sel_max_i] == 0U)) {
                    ret = E_CAR_FMON_HW_LOGIC;
                    break;
                } else {
                    i++;
                }
            } else {
                break;
            }
        }
    } while (i < FSI_FMON_ID_N);

    return ret;
}

