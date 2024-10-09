/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <uart/tegra-uart.h>       // for tegra_uart_flush_tx_hw_fifo, tegra...
#include <comb-uart/comb-uart.h>   // for comb_uart_...

/* Module-specific FSP headers */
#include <debug/debug-uart.h>      // for dbg_uart_ops...
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
CT_ASSERT(FSP__UART__TEGRA_UART_H, "Header file missing or invalid.")
CT_ASSERT(FSP__COMB_UART__COMB_UART_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

static const struct tegra_uart_conf debug_uart_conf SECTION_DEBUG_DATA =
{
    .parity         = TEGRA_UART_NO_PARITY,
    .stop_bits      = TEGRA_UART_STOP_BITS_1,
    .data_bits      = TEGRA_UART_DATA_BITS_8,
    .baud           = 115200U,
};

static inline error_t
dbg_uart_init(void *data)
{
    struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)data;

    return tegra_uart_init_hw(ctlr, &debug_uart_conf);
}

static inline void
dbg_uart_putc(void *data,
              const char ch)
{
    const struct tegra_uart_ctlr *ctlr = (const struct tegra_uart_ctlr *)data;

    tegra_uart_write_char(ctlr, ch);
}

static inline void
dbg_uart_puts(void *data,
              const char *s)
{
    const struct tegra_uart_ctlr *ctlr = (const struct tegra_uart_ctlr *)data;

    tegra_uart_write_string(ctlr, s);
}

static inline void
dbg_uart_putd(void *data,
              const char *d,
              uint32_t len)
{
    const struct tegra_uart_ctlr *ctlr = (const struct tegra_uart_ctlr *)data;

    tegra_uart_write(ctlr, d, len);
}

static inline void
dbg_uart_tx_flush(void *data)
{
    const struct tegra_uart_ctlr *ctlr = (const struct tegra_uart_ctlr *)data;

    tegra_uart_flush_tx_hw_fifo(ctlr);
}

const struct dbg_uart_ops dbg_standalone_uart_ops = {
    .init  =  &dbg_uart_init,
    .putc  =  &dbg_uart_putc,
    .puts  =  &dbg_uart_puts,
    .putd  =  &dbg_uart_putd,
    .flush =  &dbg_uart_tx_flush,
};

static inline error_t
dbg_comb_uart_init(void *data)
{
    const struct comb_uart_id *id = (const struct comb_uart_id *)data;

    return comb_uart_init(id);
}

static inline void
dbg_comb_uart_putc(void *data,
                   const char ch)
{
    const struct comb_uart_id *id = (const struct comb_uart_id *)data;

    comb_uart_putc(id, ch);
}

static inline void
dbg_comb_uart_puts(void *data,
                   const char *s)
{
    const struct comb_uart_id *id = (const struct comb_uart_id *)data;

    comb_uart_puts(id, s);
}

static inline void
dbg_comb_uart_putd(void *data,
                   const char *d,
                   uint32_t len)
{
    const struct comb_uart_id *id = (const struct comb_uart_id *)data;

    comb_uart_send(id, d, len);
}

static inline void
dbg_comb_uart_tx_flush(void *data)
{
    const struct comb_uart_id *id = (const struct comb_uart_id *)data;

    comb_uart_tx_flush(id);
}

const struct dbg_uart_ops dbg_comb_uart_ops = {
    .init  =  &dbg_comb_uart_init,
    .putc  =  &dbg_comb_uart_putc,
    .putd  =  &dbg_comb_uart_putd,
    .puts  =  &dbg_comb_uart_puts,
    .flush =  &dbg_comb_uart_tx_flush,
};

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
