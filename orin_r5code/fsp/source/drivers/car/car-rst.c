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

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for E_SUCCESS, error_t

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for register access functions...
#include <car/car-rst.h>                // for reset declarations...

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
CT_ASSERT(FSP__CAR__CAR_RST_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
static bool car_rst_op_is_asserted(const struct car_rst_inf *rst)
{
    bool ret = false;

    if (rst != NULL) {
        if ((car_reg_rd(rst->reg) & rst->msk) != 0U)
            ret = true;
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_rst_op_assert_bit(const struct car_rst_inf *rst)
{
    error_t ret = E_CAR_INVALID_RESET_ID;

    if (rst != NULL) {
        car_reg_rdwr(rst->reg, rst->msk, rst->msk);
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_rst_op_deassert_bit(const struct car_rst_inf *rst)
{
    error_t ret = E_CAR_INVALID_RESET_ID;

    if (rst != NULL) {
        car_reg_rdwr(rst->reg, 0U, rst->msk);
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_RODATA
const struct car_rst_op car_rst_op_bit = {
    .rst_is_asserted = car_rst_op_is_asserted,
    .rst_assert = car_rst_op_assert_bit,
    .rst_deassert = car_rst_op_deassert_bit,
};


SECTION_CAR_TEXT
static error_t car_rst_op_assert_reg(const struct car_rst_inf *rst)
{
    error_t ret = E_CAR_INVALID_RESET_ID;

    if (rst != NULL) {
        car_reg_wr(rst->reg + 0x04U, rst->msk);
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_rst_op_deassert_reg(const struct car_rst_inf *rst)
{
    error_t ret = E_CAR_INVALID_RESET_ID;

    if (rst != NULL) {
        car_reg_wr(rst->reg + 0x08U, rst->msk);
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_RODATA
const struct car_rst_op car_rst_op_reg = {
    .rst_is_asserted = car_rst_op_is_asserted,
    .rst_assert = car_rst_op_assert_reg,
    .rst_deassert = car_rst_op_deassert_reg,
};
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

