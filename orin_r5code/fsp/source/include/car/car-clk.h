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

#ifndef CAR__CAR_CLK_H
#define CAR__CAR_CLK_H
#define FSP__CAR__CAR_CLK_H             1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for error_t

/* Module-specific FSP headers */
#include <car/car-fmon.h>               // for frequency monitor structures...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_FMON_H, "Header file missing or invalid.")

/**
 * @file car/car-clk.h
 * @brief Structures that are internal to the CAR driver
 */

struct car_clk_inf;
/**
 * @brief Structure defining clock enable/disable operations.
 *
 * @clk_is_enabled  returns clock enable status
 * @clk_enable  function to enable clock.
 * @clk_disable  function to disable clock.
 */
struct car_clk_op_able {
    bool (*clk_is_enabled)(const struct car_clk_inf *clk);
    error_t (*clk_enable)(const struct car_clk_inf *clk);
    error_t (*clk_disable)(const struct car_clk_inf *clk);
};

/**
 * @brief Structure defining clock rate operations.
 *
 * @clk_hz_set  function to set clock rate.
 * @clk_hz_get  function to get clock rate.
 */
struct car_clk_op_rate {
    error_t (*clk_hz_set)(const struct car_clk_inf *clk, int64_t hz_out);
    error_t (*clk_hz_get)(const struct car_clk_inf *clk, int64_t *hz_out);
};

/**
 * @brief Structure defining clock div calc operations.
 *
 * @clk_div_calc_in   function to calculate divisor set to reg.
 * @clk_div_calc_out  function to calculate divisor get from reg.
 */
struct car_clk_op_div {
    uint32_t (*clk_div_calc_in) (int64_t hz_in, int64_t hz_out);
    uint32_t (*clk_div_calc_out)(const struct car_clk_inf *clk);
};


extern const struct car_clk_op_able car_clk_op_able_bit;
extern const struct car_clk_op_able car_clk_op_able_reg;
extern const struct car_clk_op_rate car_clk_op_rate_dflt;

/**
 * @brief Structure defining clock information
 *
 * @fmon  frequency monitor
 * @pclk  parent clock
 * @op_able  functions to enable/disable the clock
 * @op_rate  functions to set/get clock rate
 * @op_div   functions to calculate device clock divisor
 * @rate_min  minimum rate allowed
 * @rate_max  maximum rate allowed
 * @en_reg  clock enable/disable register
 * @en_msk  clock enable/disable mask
 * @div_reg  clock divisor register
 * @div_msk  clock divisor mask
 */
struct car_clk_inf {
    const struct car_fmon_inf *fmon;
    const struct car_clk_inf *pclk;
    const struct car_clk_op_able *op_able;
    const struct car_clk_op_rate *op_rate;
    const struct car_clk_op_div *op_div;
    int64_t hz_min;
    int64_t hz_max;
    uint32_t en_reg;
    uint32_t en_msk;
    uint32_t div_reg;
    uint32_t div_msk;
};

/**
 * @brief Define a clock
 *
 * Fills the car_clk_inf ormation structure with the info
 * associated with the clock.
 */
#define CLK_INF(_i, _oa, _er, _em, _dr, _dm,    \
                _dv, _or, _hn, _hx, _pc, _fm)   \
    [_i] = {                                    \
        .op_able = _oa,                         \
        .en_reg = _er,                          \
        .en_msk = _em,                          \
        .div_reg = _dr,                         \
        .div_msk = _dm,                         \
        .op_div = _dv,                          \
        .op_rate = _or,                         \
        .hz_min = _hn,                          \
        .hz_max = _hx,                          \
        .pclk = _pc,                            \
        .fmon = _fm,                            \
    }
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* CAR__CAR_CLK_H */

