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

#ifndef DEBUG__DEBUG_UART_H
#define DEBUG__DEBUG_UART_H
#define FSP__DEBUG__DEBUG_UART_H        1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for error_t

/* Module-specific FSP headers */

#define UART_MODE_STANDALONE 1U
#define UART_MODE_TCU        2U

/**
 * @brief simple debug uart operations structure
 *
 * Describes the minimal debug UART controller operations.
 *
 * @init    callback to initialize the uart controller context.
 * @putc    callback to write a char to the UART controller tx fifo.
 * @putd    callback to write an amount of data to the UART controller tx fifo.
 * @puts    callback to write a string to the UART controller tx fifo.
 * @flush   callback to flush the uart controller tx fifo.
 */
struct dbg_uart_ops {
    error_t (*init)     (void *ctlr);
    void    (*putc)     (void *ctlr, const char ch);
    void    (*putd)     (void *ctlr, const char *d, uint32_t len);
    void    (*puts)     (void *ctlr, const char *s);
    void    (*flush)    (void *ctlr);
};

void dbg_init_uart_ops(uint32_t uart_mode);
void dbg_init(void);
void dbg_putc(char c);
void dbg_putd(const char *d, uint32_t len);
void dbg_puts(const char *s);
void dbg_flush(void);

#endif
