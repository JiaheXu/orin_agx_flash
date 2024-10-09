/*
 * Copyright (c) 2014-2020 NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>

/* Early FSP headers */
#include <misc/macros.h>

/* Hardware headers */

/* Late FSP headers */

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

#pragma message "content of SIZE_MAX: " STR(SIZE_MAX)
#pragma message "content of INT_MAX: " STR(INT_MAX)
#pragma message "content of INT32_MAX: " STR(INT32_MAX)
#pragma message "content of UINT_MAX: " STR(UINT_MAX)
#pragma message "content of UINT32_MAX: " STR(UINT32_MAX)
#pragma message "content of LONG_MAX: " STR(LONG_MAX)
#pragma message "content of size_t: " STR(__SIZE_TYPE__)
#pragma message "content of sizeof(uint32_t): " STR(sizeof(uint32_t))
#pragma message "content of sizeof(unsigned long): " STR(sizeof(unsigned long))
#pragma message "content of sizeof(void *()): " STR(sizeof(void *()))
#pragma message "content of __LONG_MAX__: " STR(__LONG_MAX__)
#pragma message "content of UINTPTR_MAX: " STR(UINTPTR_MAX)
