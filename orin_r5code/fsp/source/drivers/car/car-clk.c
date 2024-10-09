/* Copyright (c) 2020-2021 NVIDIA CORPORATION.  All rights reserved.
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

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for E_SUCCESS, error_t

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for register access functions...
#include <car/car-clk.h>                // for declarations...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_REG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_CLK_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
static bool car_clk_op_is_enabled(const struct car_clk_inf *clk)
{
    bool ret = false;

    if (clk != NULL) {
        if ((car_reg_rd(clk->en_reg) & clk->en_msk) != 0U)
            ret = true;
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_clk_op_enable_bit(const struct car_clk_inf *clk)
{
    error_t ret = E_CAR_NULL_PTR;

    if (clk != NULL) {
        if (clk->en_msk == 0U) {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        } else {
            car_reg_rdwr(clk->en_reg, clk->en_msk, clk->en_msk);
            ret = E_SUCCESS;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_clk_op_disable_bit(const struct car_clk_inf *clk)
{
    error_t ret = E_CAR_NULL_PTR;

    if (clk != NULL) {
        if (clk->en_msk == 0U) {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        } else {
            car_reg_rdwr(clk->en_reg, 0U, clk->en_msk);
            ret = E_SUCCESS;
        }
    }
    return ret;
}

SECTION_CAR_RODATA
const struct car_clk_op_able car_clk_op_able_bit = {
    .clk_is_enabled = car_clk_op_is_enabled,
    .clk_enable = car_clk_op_enable_bit,
    .clk_disable = car_clk_op_disable_bit,
};


SECTION_CAR_TEXT
static error_t car_clk_op_enable_reg(const struct car_clk_inf *clk)
{
    error_t ret = E_CAR_NULL_PTR;

    if (clk != NULL) {
        if (clk->en_msk == 0U) {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        } else {
            car_reg_wr(clk->en_reg + 0x04U, clk->en_msk);
            ret = E_SUCCESS;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_clk_op_disable_reg(const struct car_clk_inf *clk)
{
    error_t ret = E_CAR_NULL_PTR;

    if (clk != NULL) {
        if (clk->en_msk == 0U) {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        } else {
            car_reg_wr(clk->en_reg + 0x08U, clk->en_msk);
            ret = E_SUCCESS;
        }
    }
    return ret;
}

SECTION_CAR_RODATA
const struct car_clk_op_able car_clk_op_able_reg = {
    .clk_is_enabled = car_clk_op_is_enabled,
    .clk_enable = car_clk_op_enable_reg,
    .clk_disable = car_clk_op_disable_reg,
};



SECTION_CAR_TEXT
static error_t clk_get_hz_in(const struct car_clk_inf *clk, int64_t *hz_in)
{
    error_t ret = E_CAR_NULL_PTR;

    if ((clk != NULL) && (hz_in != NULL)) {
        if ((clk->pclk != NULL) && (clk->pclk->op_rate != NULL) && (clk->pclk->op_rate->clk_hz_get != NULL)) {
            ret = clk->pclk->op_rate->clk_hz_get((struct car_clk_inf *)clk->pclk, hz_in);
        } else {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_clk_op_hz_get(const struct car_clk_inf *clk, int64_t *hz_out)
{
    int64_t hz_in;
    uint32_t divider;
    error_t ret = E_CAR_NULL_PTR;

    if ((clk != NULL) && (hz_out != NULL)) {
        ret = clk_get_hz_in(clk, &hz_in);
        if (ret == E_SUCCESS) {
            if ((clk->op_div != NULL) && (clk->op_div->clk_div_calc_out != NULL)) {
                divider = clk->op_div->clk_div_calc_out(clk);
            } else {
                divider = car_reg_rd_val(clk->div_reg, clk->div_msk);
                divider = divider + 1U;
            }
            *hz_out = hz_in / divider;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_clk_op_hz_set(const struct car_clk_inf *clk, int64_t hz_out)
{
    int64_t hz_in;
    uint32_t divider;
    error_t ret = E_CAR_NULL_PTR;

    if (clk != NULL) {
        if ((clk->hz_max != 0ULL) && (hz_out > clk->hz_max)) {
            ret = E_CAR_CLOCK_HZ_NOT_SUPPORTED;
        } else if ((clk->hz_min != 0ULL) && (hz_out < clk->hz_min)) {
            ret = E_CAR_CLOCK_HZ_NOT_SUPPORTED;
        } else {
            ret = clk_get_hz_in(clk, &hz_in);
            if (ret == E_SUCCESS) {
                if (hz_out != 0LL) {
                    if ((clk->op_div != NULL) && (clk->op_div->clk_div_calc_in != NULL)) {
                        divider = clk->op_div->clk_div_calc_in(hz_in, hz_out);
                    } else {
                        divider = hz_in / hz_out;
                        if (divider > 0) {
                            divider = divider - 1U;
                        }
                    }
                    car_reg_rdwr_val(clk->div_reg, clk->div_msk, divider);
                } else {
                    ret = E_CAR_CLOCK_HZ_NOT_SUPPORTED;
                }
            }
        }
    }
    return ret;
}

SECTION_CAR_RODATA
const struct car_clk_op_rate car_clk_op_rate_dflt = {
    .clk_hz_get = car_clk_op_hz_get,
    .clk_hz_set = car_clk_op_hz_set,
};
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

