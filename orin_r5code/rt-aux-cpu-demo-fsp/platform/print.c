/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdarg.h>            // for va_end, va_start
#include <stdint.h>            // for uint32_t
#include <stdio.h>             // for vsnprintf

/* local rtos headers */
#include <osa/rtos-task.h>
#include <osa/rtos-semaphore.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <irq/irqs.h>

/* Module-specific FSP headers */
#include <debug/printf-isr.h>
#include <debug/print.h>

/* local headers */
#include <spe-uart.h>
#include <tcu.h>

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
    CT_ASSERT(FSP__DEBUG__PRINT_H, "Header file missing or invalid.")

#define PRINTF_BUFSIZE     128UL

static uint32_t print_log_level = PRINT_LEVEL_INFO;

#if defined(USE_GCC) && (USE_GCC == 1)
static int vprintf_msg(const char *fmt, va_list ap)
    __attribute__ ((format (__printf__, 1, 0)));
#endif

static int print_msg(const char* msg_buf, int len, bool from_isr)
{
    uint32_t written = 0UL;

    if (from_isr) {
        spe_uart_write_now(msg_buf, len);
        written = len;
        goto out;
    }
    enter_critical();
    spe_uart_write_now(msg_buf, len);
    written = len;
    exit_critical();

out:
    return written;
}

int32_t vprintf_isr(const char *fmt, va_list ap)
{
    char msg[PRINTF_ISR_BUFSIZE];
    int ret;

    ret = vsnprintf(msg, PRINTF_ISR_BUFSIZE, fmt, ap);

    if (ret > 0)
    {
        ret = (ret > PRINTF_ISR_BUFSIZE) ? PRINTF_ISR_BUFSIZE : ret;
        if (is_tcu_enabled()) {
            tcu_print_msg(msg, ret, true);
        } else {
            print_msg(msg, ret, true);
        }
    }

    return ret;
}

int32_t printf_isr(const char *fmt, ...)
{
    va_list args;
    int ret;

    va_start(args, fmt);
    ret = vprintf_isr(fmt, args);
    va_end(args);

    return ret;
}

static int vprintf_msg(const char *fmt, va_list ap)
{
    char msg[PRINTF_BUFSIZE];
    int ret;

    ret = vsnprintf(msg, PRINTF_BUFSIZE, fmt, ap);

    if (ret > 0)
    {
        ret = (ret > PRINTF_BUFSIZE) ? PRINTF_BUFSIZE : ret;
        if (is_tcu_enabled()) {
            tcu_print_msg(msg, ret, false);
        } else {
            print_msg(msg, ret, false);
        }
    }

    return ret;
}

void print_message(uint32_t level,
                   const char *format,
                   ...)
{
    va_list args;

    if (level <= print_log_level) {
        va_start(args, format);
        if (in_interrupt()) {
            vprintf_isr(format, args);
        } else {
            vprintf_msg(format, args);
        }
        va_end(args);
    }
}

void set_print_log_level(const uint32_t level)
{
    print_log_level = level;
}

uint32_t get_print_log_level(void)
{
    return print_log_level;
}
