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

#ifndef CAR__CAR_RST_H
#define CAR__CAR_RST_H
#define FSP__CAR__CAR_RST_H             1

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

/**
 * @file car/car-rst.h
 * @brief Structures that are internal to the CAR driver
 */

/**
 * @brief Structure defining reset information
 *
 * @ops   Reset interface operations.
 * @label Reset signal name as defined by hardware.
 * @flags Boolean details about the reset signal.
 * @reg   The reset's main register offset in CAR.
 * @bit   The bit position that controls the reset.
 */
struct car_rst_inf {
    const struct car_rst_op *op;
    uint32_t flags;
    uint32_t reg;
    uint32_t msk;
};

/**
 * @brief Structure defining reset operations
 *
 * @rst_assert   Function to assert the reset.
 * @rst_deassert Function to deassert the reset.
 */
struct car_rst_op {
    bool (*rst_is_asserted)(const struct car_rst_inf *rst);
    error_t (*rst_assert)(const struct car_rst_inf *rst);
    error_t (*rst_deassert)(const struct car_rst_inf *rst);
};

extern const struct car_rst_op car_rst_op_bit;
extern const struct car_rst_op car_rst_op_reg;

/**
 * @brief Define a bit controlled reset signal
 *
 * Fills the reset structure with the info associated with a bit
 * controlled reset.
 */
#define CAR_RST_TYPE_B(_i, _f, _r, _m)  \
    [_i] = {                            \
        .op = &car_rst_op_bit,          \
        .flags = _f,                    \
        .reg = _r,                      \
        .msk = _m,                      \
    }

/**
 * @brief Define a register controlled reset signal
 *
 * Fills the reset structure with the info associated with a
 * register controlled reset.
 */
#define CAR_RST_TYPE_R(_i, _f, _r, _m)  \
    [_i] = {                            \
        .op = &car_rst_op_reg,          \
        .flags = _f,                    \
        .reg = _r,                      \
        .msk = _m,                      \
    }
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* CAR__CAR_RST_H */

