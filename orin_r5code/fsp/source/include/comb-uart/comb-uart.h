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

#ifndef COMB_UART__COMB_UART_H
#define COMB_UART__COMB_UART_H
#define FSP__COMB_UART__COMB_UART_H                      1

/**
 * @file comb-uart/comb-uart.h
 * @brief functions for performing combined uart operations.
 */

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for error_t, FSP__ERROR__COMMON...
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/*
 * Declaration for comb_uart_id that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
struct comb_uart_id;

/**
 * @brief output a string via the combined UART protocol.
 *
 * @pre comb_uart_init() has been called
 *
 * @param[in] id   combined uart context
 * @param[in] s    string to output
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_COMB_UART_NULL_PTR             invalid id paramter passed
 * @retval E_COMB_UART_TIMEDOUT             server timed out
 */
void comb_uart_puts(const struct comb_uart_id *id,
                    const char *s);

/**
 * @brief output data via the combined UART protocol.
 *
 * @pre comb_uart_init() has been called
 *
 * @param[in] id   combined uart context
 * @param[in] d    data buffer to output
 * @param[in] len  len of the data buffer
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_COMB_UART_NULL_PTR             invalid id paramter passed
 * @retval E_COMB_UART_TIMEDOUT             server timed out
 */
void comb_uart_send(const struct comb_uart_id *id,
                    const char *d,
                    const uint32_t len);

/**
 * @brief output a char via the combined UART protocol.
 *
 * @pre comb_uart_init() has been called
 *
 * @param[in] id   combined uart context
 * @param[in] ch   character to output
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_COMB_UART_NULL_PTR             invalid id paramter passed
 * @retval E_COMB_UART_TIMEDOUT             server timed out
 */
error_t comb_uart_putc(const struct comb_uart_id *id,
                       const char ch);

/**
 * @brief tx flush operation via the combined UART protocol.
 *
 * @pre comb_uart_init() has been called
 *
 * @param[in] id   combined uart context
 * @param[in] wait flag to indicate wait until the tx buffer is flushed
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_COMB_UART_NULL_PTR             invalid id paramter passed
 * @retval E_COMB_UART_TIMEDOUT             server timed out
 */
error_t comb_uart_tx_flush(const struct comb_uart_id *id);

/**
 * @brief global initialization of the COMB_UART context.
 *
 * @pre None
 *
 * @param[in] id combined uart context
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_COMB_UART_NULL_PTR             invalid id paramter passed
 * @retval E_COMB_UART_TIMEDOUT             server timed out
 */
error_t comb_uart_init(const struct comb_uart_id *id);

#endif
