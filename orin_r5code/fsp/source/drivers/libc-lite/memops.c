/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dst, const void *src, size_t count)
{
    char *d=dst;
    const char *s = src;
    while (count--)
        *d++ = *s++;

    return dst;
}

int memcmp(const void *s1, const void *s2, size_t n)
{
    const unsigned char *d=s1;
    const unsigned char *s = s2;
    while (n--)
    {
        int x = (*s) - (*d);
        if (x)
            return x;
        d++;
        s++;
    }
    return 0;
}

void *memset(void *s, int c, size_t n)
{
    unsigned char *p = s;
    while (n--)
        *p++ = (uint8_t)c;
    return s;
}
