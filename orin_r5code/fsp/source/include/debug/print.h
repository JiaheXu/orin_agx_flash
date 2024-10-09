/* Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef DEBUG__PRINT_H
#define DEBUG__PRINT_H
#define FSP__DEBUG__PRINT_H                             1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

#define PRINT_LEVEL_ALWAYS         0UL
#define PRINT_LEVEL_ERROR          1UL
#define PRINT_LEVEL_WARNING        2UL
#define PRINT_LEVEL_INFO           3UL
#define PRINT_LEVEL_DEBUG          4UL
#define PRINT_LEVEL_HIGHEST        5UL

#if defined(USE_GCC) && (USE_GCC == 1)
void print_message(uint32_t level,
                   const char *format,
                   ...)
    __attribute__ ((format (__printf__, 2, 3)));

#else
void print_message(uint32_t level,
                   const char *format,
                   ...);
#endif

void set_print_log_level(const uint32_t level);
uint32_t get_print_log_level(void);

#define pr_warn(...)    \
    print_message(PRINT_LEVEL_WARNING, __VA_ARGS__)

#define pr_error(...)   \
    print_message(PRINT_LEVEL_ERROR, __VA_ARGS__)

#define pr_info(...)    \
    print_message(PRINT_LEVEL_INFO, __VA_ARGS__)

#define pr_debug(...)   \
    print_message(PRINT_LEVEL_DEBUG, __VA_ARGS__)

#endif /* DEBUG__PRINT_H */
