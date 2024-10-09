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

#ifndef TEGRA_UART_H
#define TEGRA_UART_H
#define FSP__UART__TEGRA_UART_H        1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

typedef uint8_t         tegra_uart_parity_t;
#define TEGRA_UART_NO_PARITY    ((tegra_uart_parity_t)1U)
#define TEGRA_UART_ODD_PARITY   ((tegra_uart_parity_t)2U)
#define TEGRA_UART_EVEN_PARITY  ((tegra_uart_parity_t)3U)

typedef uint8_t         tegra_uart_stop_bits_t;
#define TEGRA_UART_STOP_BITS_1  ((tegra_uart_stop_bits_t)1U)
#define TEGRA_UART_STOP_BITS_2  ((tegra_uart_stop_bits_t)2U)

typedef uint8_t         tegra_uart_data_bits_t;
#define TEGRA_UART_DATA_BITS_5  ((tegra_uart_data_bits_t)1U)
#define TEGRA_UART_DATA_BITS_6  ((tegra_uart_data_bits_t)2U)
#define TEGRA_UART_DATA_BITS_7  ((tegra_uart_data_bits_t)3U)
#define TEGRA_UART_DATA_BITS_8  ((tegra_uart_data_bits_t)4U)

struct tegra_uart_conf {
    tegra_uart_parity_t         parity;
    tegra_uart_stop_bits_t      stop_bits;
    tegra_uart_data_bits_t      data_bits;
    uint32_t                    baud;
};

// IWYU pragma: no_forward_declare tegra_uart_ctlr
struct tegra_uart_ctlr;

error_t tegra_uart_init_hw(struct tegra_uart_ctlr * const ctlr,
                           const struct tegra_uart_conf * const conf);
void tegra_uart_flush_tx_hw_fifo(const struct tegra_uart_ctlr * const ctlr);
void tegra_uart_write_char(const struct tegra_uart_ctlr * const ctlr,
                           const char c);
char tegra_uart_read_char(const struct tegra_uart_ctlr * const ctlr);
void tegra_uart_write(const struct tegra_uart_ctlr * const ctlr,
                      const char *buf,
                      uint32_t count);
void tegra_uart_write_string(const struct tegra_uart_ctlr * const ctlr,
                             const char *buf);

#endif
