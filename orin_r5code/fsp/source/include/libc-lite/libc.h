/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef LIBC__LIBC_H
#define LIBC__LIBC_H
#define FSP__LIBC__LIBC_H                             1
#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>
// We don't have libc in libfsp. This file exposes a handful of libc-like utility functions

// memops.c
void *memcpy(void *dst, const void *src, size_t count);
int memcmp(const void *s1, const void *s2, size_t n);
void *memset(void *s, int c, size_t n);

// sleep.c
uint64_t delay(unsigned cycles);
void usleep(unsigned uSec);

// string.c
size_t strnlen(const char *s, size_t n);

// vprintfmt.c
int vprintfmt(void (*putch)(int, void*), void *putdat, const char* pFormat, va_list ap);

// freestanding things
__attribute__( ( always_inline ) ) static inline int isprint(int c)
{
    return c >= ' ' && c <= '~';
}

#endif // LIBNVRISCV_LIBC_H
