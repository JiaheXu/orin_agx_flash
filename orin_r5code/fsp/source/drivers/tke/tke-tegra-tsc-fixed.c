/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdint.h>                 // for uint64_t, uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <ospl/rtos-port.h>         // for rtosTICK_RATE_MS
#include <misc/bitops.h>            // for hilo_to_64, FSP__MISC__BITOPS_H
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <tke/sections-tke.h>       // Immune from CT_ASSERT protection
#include <tke/tke-tegra.h>          // for tegra_tke_get_tsc64, FSP__TKE__TK...


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
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/*
 * Number of nanoseconds per tic of TSC
 */
#define TEGRA_TKE_TSC_NS_PER_TIC        32U

/**
 * div_by_1000()        - return value divided by 1000
 *
 * @value:      64-bit value that is to be divided by 1000
 *
 * This function will divide a 64-bit value by 1000.  It will do it so
 * as not to pull in the general 64-bit divide code.
 *
 * Return Value:
 *      64-bit value representing the input divided by 1000.
 */
SECTION_TKE_TEXT static uint64_t
div_by_1000(const uint64_t value)
{
    uint32_t    value_hi;
    uint32_t    value_lo;
    uint64_t    result;

    /*
     * There isn't a 64-bit divide on the R5 and to avoid
     * pulling in a lot of library code, a bit of a "cheat"
     * is being done here.
     *
     * The idea is that if value fits in 32-bits then perform
     * a 32-bit divide and expand it back to 64-bits (where it
     * is known that the upper 32-bits will be 0).
     *
     * If the upper 32-bits of value are not zero, then approximate
     * the divide with some shifts and adds.  Since the result is
     * being used by timer functions, super high accuracy is not
     * absolutely required.
     */
    INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
    value_hi = HI32(value);
    if (value_hi == 0U) {
        INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-559, DR: SWE-FSP-048-SWSADR.docx");
        value_lo = LOW32(value);
        value_lo /= 1000U;
        result = PACK64(value_lo, value_hi);
    } else {
        /*
         * Approximate dividing by 1000
         *
         * Division is really an approximation and is really
         * just multiplication by the reciprocal (e.g. x / 1000
         * is equivalent to x * 1/1000).  So what the code is
         * doing is multiplying the value by 1/1000 (or the
         * approximation thereof).
         *
         * The series for x/1000 (for at least to the accuracy we
         * need) is:
         * x/1000 ~= x/2^10 + x/2^16 + x/2^17 + x/2^21 + x/2^24
         *
         * It could argued that for the timing needs that only the
         * first term is sufficient but computing the extra terms
         * is not expensive and results in a closer approximation.
         */
    START_RFD_BLOCK(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        result = value >> 10;
        result += value >> 16;
        result += value >> 17;
        result += value >> 21;
        result += value >> 24;
        END_RFD_BLOCK(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    }

    return result;
}

/**
 * tegra_tke_convert_usecs_to_ticks()      - Convert a microsecond value to ticks
 *
 * @value:              a value in microseconds to be converted
 *
 * This function will covert a value that is represented in microseconds and
 * convert that to the equivalent number of ticks (with lower resolution of
 * course).
 *
 * Return Values:
 *      number of ticks that represents the value
 */
SECTION_TKE_TEXT uint64_t
tegra_tke_convert_usecs_to_ticks(const uint64_t value)
{
    uint64_t    ticks;

    ticks = div_by_1000(value) / rtosTICK_RATE_MS;

    return ticks;
}

/* TSC timestamp at nanosecond resolution */
SECTION_TKE_TEXT uint64_t
tegra_tke_get_tsc_ns(void)
{
        /* TSC (cntfid0) is usually 31.25MHz, cntperiod is 32 ns */
        return tegra_tke_get_tsc64() * TEGRA_TKE_TSC_NS_PER_TIC;
}

/* Convert TSC timestamp to nanosecond resolution */
SECTION_TKE_TEXT uint64_t
tegra_tke_tsc_to_ns(uint64_t tsc)
{
        /* TSC is 31.25MHz = 32 ns */
        INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        return tsc * TEGRA_TKE_TSC_NS_PER_TIC;
}

SECTION_TKE_TEXT uint64_t
tegra_tke_get_elapsed_usec(uint32_t prev_tsc_hi, uint32_t prev_tsc_low)
{
        uint64_t result, curr_tsc, prev_tsc;

        curr_tsc = tegra_tke_get_tsc64();
        prev_tsc = hilo_to_64(prev_tsc_hi, prev_tsc_low);

        /* cntfid0 holds the base frequency of the tsc.
         * This will always be the frequency tsc runs.
         * There is no plan to change it for tegra
         */
        INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        result = div_by_1000((curr_tsc - prev_tsc) * TEGRA_TKE_TSC_NS_PER_TIC);

        return result;
}

/**
 * tegra_tke_get_elapsed_usecs64()  - Return elapsed microseconds
 *
 * @start:      value of TSC at the start of the period of when
 *              the elapsed time is being computed.
 *
 * Return Value:
 *      the number of microseconds that have elapsed between when
 *      the start time was taken and this function called.  This is
 *      a 64-bit value.
 */
SECTION_TKE_TEXT uint64_t
tegra_tke_get_elapsed_usecs64(const uint64_t start)
{
    uint64_t    elapsed;
    uint64_t    now;
    uint64_t    elapsed_tsc;

    now = tegra_tke_get_tsc64();

    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    elapsed_tsc = now - start;

    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    elapsed = div_by_1000(elapsed_tsc * TEGRA_TKE_TSC_NS_PER_TIC);

    return elapsed;
}

SECTION_TKE_INIT_TEXT void
tegra_tsc_init(void)
{
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
