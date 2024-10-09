/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdint.h>                // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>        // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>   // for E_SUCCESS, error_t
#include <misc/macros.h>           // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <processor/debug-hw.h>    // for DEBUG_UART, FSP__PROCESSOR__DEBUG_...

/* Module-specific FSP headers */
#include <debug/assert.h>          // for ASSERT, FSP__DEBUG__ASSERT_H
#include <debug/debug-uart.h>      // for FSP__DEBUG__DEBUG_UART_H, dbg_flush
#include <debug/sections-debug.h>  // Immune from CT_ASSERT protection


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
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__DEBUG_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__DEBUG_UART_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

extern const struct dbg_uart_ops dbg_standalone_uart_ops;
extern const struct dbg_uart_ops dbg_comb_uart_ops;

/**
 *
 * Defaults to standalone uart ops to not break the builds
 * for existing clients. At some point, this needs to be
 * revisited.
 */
static struct dbg_uart_ops *ops = NULL;
static void *uart_ctlr = NULL;

static inline struct dbg_uart_ops *c_ptr_to_nc_ptr(const struct dbg_uart_ops *input)
{
    union cast_c_ptr_to_nc_ptr {
        const struct dbg_uart_ops *input;
        struct dbg_uart_ops *output;
    };
    union cast_c_ptr_to_nc_ptr t;

    t.input = input;

    return t.output;
}

#if defined(DEBUG_COMB_UART)
static inline void *c_tcu_ptr_to_v_ptr(const struct comb_uart_id *input)
{
    union cast_c_tcu_ptr_to_v_ptr {
        const struct comb_uart_id* input;
        void *output;
    };
    union cast_c_tcu_ptr_to_v_ptr t;

    t.input = input;

    return t.output;
}
#endif

SECTION_DEBUG_TEXT void
dbg_init_uart_ops(uint32_t uart_mode)
{
    switch (uart_mode) {
        case UART_MODE_TCU:
            ops = c_ptr_to_nc_ptr(&dbg_comb_uart_ops);
#if defined(DEBUG_COMB_UART)
            uart_ctlr = c_tcu_ptr_to_v_ptr(&DEBUG_COMB_UART);
#endif
            break;
        case UART_MODE_STANDALONE:
        default:
            ops = c_ptr_to_nc_ptr(&dbg_standalone_uart_ops);
            uart_ctlr = &DEBUG_UART;;
            break;
    }
}


SECTION_DEBUG_TEXT void
dbg_init(void)
{
    error_t rc = -1;

    /* default to standalone if not registered */
    if (ops == NULL) {
        ops = c_ptr_to_nc_ptr(&dbg_standalone_uart_ops);
        uart_ctlr = &DEBUG_UART;;
    }

    if (ops->init != NULL) {
        rc = ops->init(uart_ctlr);
    }

    ASSERT(rc == E_SUCCESS);
}

SECTION_DEBUG_TEXT void
dbg_putc(char c)
{
    ops->putc(uart_ctlr, c);
}

SECTION_DEBUG_TEXT void
dbg_putd(const char *d, uint32_t len)
{
    ops->putd(uart_ctlr, d, len);
}

SECTION_DEBUG_TEXT void
dbg_puts(const char *s)
{
    ops->puts(uart_ctlr, s);
}

SECTION_DEBUG_TEXT void
dbg_flush(void)
{
    ops->flush(uart_ctlr);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
