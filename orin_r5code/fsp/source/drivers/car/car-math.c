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
#include <misc/math_secure.h>           // for the math

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-math.h>               // for declarations...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MATH_SECURE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_MATH_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
int64_t car_add_s64(int64_t a, int64_t b)
{
    int64_t c = (int64_t)sadd_u64(a, b, OVERFLOW_ACTION_0);

    return c;
}

SECTION_CAR_TEXT
int64_t car_sub_s64(int64_t a, int64_t b)
{
    int64_t c = (int64_t)ssub_u64(a, b, OVERFLOW_ACTION_0);

    return c;
}

SECTION_CAR_TEXT
int64_t car_mul_s64(int64_t a, int64_t b)
{
    int64_t c = (int64_t)smul_u64(a, b, OVERFLOW_ACTION_0);

    return c;
}

SECTION_CAR_TEXT
int64_t car_div_s64(int64_t a, int64_t b)
{
    int64_t c = 0LL;

    /* Even though a division by 0 would trigger a fatal error already,
     * add a check for divisor == 0 here to inform coverity we really check
     * for this case. This avoids an INT32-C violation.
     */
    if ((b == 0) || ((a == INT64_MIN) && (b == -1LL))) {
        /* nothing */
    } else {
        c = a / b;
    }
    return c;
}

SECTION_CAR_TEXT
/* divide unsigned numbers by rounding to nearest integer */
int64_t car_div64_round(int64_t n, int64_t d)
{
    int64_t ret;

    /* Simplest way to do round-to-nearest division with unsigned
     * numbers in C would be:
     *  return (n + d/2)/d;
     *
     * But this does not work with very large numbers (i.e. n+d/2
     * will overflow when both numbers are close to INT64_MAX)
     *
     * Hence compute same with more elborate algorithm:
     */
    ret = n / d;
    if ((n % d) != 0) {
        if ((n % d) >= car_add_s64(d / 2, d % 2)) {
            ret = car_add_s64(ret, 1LL);
        }
    }
    return ret;
}

SECTION_CAR_TEXT
int64_t car_range_s64(int64_t x, int64_t x_min, int64_t x_max)
{
    if ((x_min != 0LL) && (x < x_min)) {
        x = x_min;
    } else if ((x_max != 0LL) && (x > x_max)) {
        x = x_max;
    }
    return x;
}

SECTION_CAR_TEXT
int32_t car_abs_s32(int32_t x)
{
    return (x >= 0) ? (uint32_t)x : (0U - (uint32_t)x);
}

SECTION_CAR_TEXT
int64_t car_abs_s64(int64_t x)
{
    return (x >= 0) ? (uint64_t)x : (0U - (uint64_t)x);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

