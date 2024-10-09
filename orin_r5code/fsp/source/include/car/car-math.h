/* Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CAR__CAR_MATH_H
#define CAR__CAR_MATH_H
#define FSP__CAR__CAR_MATH_H            1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...

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


/**
 * WAR: CAR_FALSE can be used with do { ... } while (CAR_FALSE)
 * loops instead of while(0) or while(false), both of which seem
 * to confuse coverity to report rule 14.4 violations (The
 * condition expression "0" does not have an essentially boolean
 * type).
 */
#define CAR_FALSE                       (0 == 1)
#define CAR_TRUE                        (!CAR_FALSE)

/**
 * CAR_RET_UNUSED is for ignoring function return values in a
 * way that we can later simply grep for them. This is
 * acceptable way of defining explicitly a function return value
 * as uninteresting for MISRA-C Rule 17.7.
 */
#define CAR_RET_UNUSED(...) \
    do {                    \
       (void) __VA_ARGS__ ; \
    } while(CAR_FALSE)

int64_t car_add_s64(int64_t a, int64_t b);
int64_t car_sub_s64(int64_t a, int64_t b);
int64_t car_mul_s64(int64_t a, int64_t b);
int64_t car_div_s64(int64_t a, int64_t b);
int64_t car_div64_round(int64_t n, int64_t d);
int64_t car_range_s64(int64_t x, int64_t x_min, int64_t x_max);
int32_t car_abs_s32(int32_t x);
int64_t car_abs_s64(int64_t x);
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* CAR__CAR_MATH_H */


