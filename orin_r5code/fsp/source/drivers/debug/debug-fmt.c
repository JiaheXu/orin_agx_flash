/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>               // for bool, false, true
#include <stdint.h>                // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>        // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>           // for ARRAY_SIZE

/* Module-specific FSP headers */
#include <debug/debug-fmt.h>       // for FSP__DEBUG__DEBUG_FMT_H, dbg_putdec
#include <debug/debug-uart.h>      // for dbg_putc, dbg_puts, FSP__DEBUG__DE...
#include <debug/sections-debug.h>  // for SECTION_DEBUG_TEXT, SECTION_DEBUG_...

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
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__DEBUG_FMT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__DEBUG_UART_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

SECTION_DEBUG_TEXT static char
nibble_to_hex(uint32_t n)
{
    if (n < 10)
        return '0' + n;
    return 'a' + n - 10;
}

SECTION_DEBUG_TEXT void
dbg_puthex8(uint32_t n)
{
    dbg_puts("0x");
    dbg_putc(nibble_to_hex((n >> 28) & 0xf));
    dbg_putc(nibble_to_hex((n >> 24) & 0xf));
    dbg_putc(nibble_to_hex((n >> 20) & 0xf));
    dbg_putc(nibble_to_hex((n >> 16) & 0xf));
    dbg_putc(nibble_to_hex((n >> 12) & 0xf));
    dbg_putc(nibble_to_hex((n >>  8) & 0xf));
    dbg_putc(nibble_to_hex((n >>  4) & 0xf));
    dbg_putc(nibble_to_hex( n        & 0xf));
}

/* This lookup table avoids the integer division (div /= 10) */
SECTION_DEBUG_DATA
static uint32_t divisors[] =
{
    1U,
    10U,
    100U,
    1000U,
    10000U,
    100000U,
    1000000U,
    10000000U,
    100000000U,
    1000000000U
};

SECTION_DEBUG_TEXT void
dbg_putdec(uint32_t n)
{
    int32_t i;
    bool keep_print = false;
    uint32_t div;
    uint32_t res;

    if (n == 0) {
        dbg_putc('0');
        return;
    }

    for (i = ARRAY_SIZE(divisors) - 1; i >= 0; i--) {
        div = divisors[i];
        if (n >= div) {
            keep_print = true;
            res = n / div;
            dbg_putc(res + '0');
            n -= res * div;
        } else if (keep_print) {
            dbg_putc('0');
        }
    }
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
