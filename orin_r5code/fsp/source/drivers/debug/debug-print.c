/*
 * Copyright (c) 2014-2022, NVIDIA CORPORATION.  All rights reserved.
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
#include <ctype.h>                      // for isdigit
#include <stdarg.h>                     // for va_arg, va_list, va_end, va_s...
#include <stdbool.h>                    // for bool, true, false
#include <stdint.h>                     // for uint32_t, int32_t, uint64_t
#include <string.h>                     // for size_t, NULL, memset, memcpy

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/armv7-regs.h>             // for disable_interrupts, restore_i...
#include <irq/safe-irqs.h>
#include <misc/bitops.h>                // for BIT, FSP__MISC__BITOPS_H
#include <misc/macros.h>                // for ARRAY_SIZE

/* Module-specific FSP headers */
#include <debug/assert.h>               // for ASSERT, FSP__DEBUG__ASSERT_H
#include <debug/debug-print.h>          // for dbg_printf, dbg_snprintf, dbg...
#include <debug/debug-uart.h>           // for dbg_puts, FSP__DEBUG__DEBUG_U...
#include <debug/sections-debug.h>       // Immune from CT_ASSERT protection

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
CT_ASSERT(FSP__CPU__ARMV7_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_SYS_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__DEBUG_UART_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define DBG_PRINT_BUFF_LEN              160

typedef uint32_t        format_flags_t;

#define DBG_FMT_FL_ALTERNATE            ((format_flags_t)BIT(0))
#define DBG_FMT_FL_ZERO_PAD             ((format_flags_t)BIT(1))
#define DBG_FMT_FL_LEFT_ADJUST          ((format_flags_t)BIT(2))
#define DBG_FMT_FL_BLANK_POSITIVE       ((format_flags_t)BIT(3))
#define DBG_FMT_FL_ALWAYS_SIGN          ((format_flags_t)BIT(4))
#define DBG_FMT_FL_LENGTH_CHAR          ((format_flags_t)BIT(5))
#define DBG_FMT_FL_LENGTH_SHORT         ((format_flags_t)BIT(6))
#define DBG_FMT_FL_LENGTH_LONG          ((format_flags_t)BIT(7))
#define DBG_FMT_FL_LENGTH_LONG_LONG     ((format_flags_t)BIT(8))

#define DBG_FMT_FL_LENGTH_MASK          (DBG_FMT_FL_LENGTH_CHAR                 \
                                         | DBG_FMT_FL_LENGTH_SHORT              \
                                         | DBG_FMT_FL_LENGTH_LONG               \
                                         | DBG_FMT_FL_LENGTH_LONG_LONG)


typedef union {
    int64_t             sint64;
    uint64_t            uint64;
    int32_t             sint32;
    uint32_t            uint32;
    int16_t             sint16;
    uint16_t            uint16;
    int8_t              sint8;
    uint8_t             uint8;
    char                ch;
    char                *string;
} format_value_t;

/* This lookup table avoids the integer division (div /= 10) */
static const uint32_t divisors[]        SECTION_DEBUG_RODATA =
{
    1000000000U,
    100000000U,
    10000000U,
    1000000U,
    100000U,
    10000U,
    1000U,
    100U,
    10U,
    1U,
};

#define NUM_DECIMAL_DIGITS      (ARRAY_SIZE(divisors))
#define DECIMAL_BUFF_LEN        (NUM_DECIMAL_DIGITS + 2U)

#define NUM_OCTAL_DIGITS        11U
#define OCTAL_BUFF_LEN          (NUM_OCTAL_DIGITS + 2U)

static const char       upper_hex[]     SECTION_DEBUG_RODATA = "0123456789ABCDEF";
static const char       lower_hex[]     SECTION_DEBUG_RODATA = "0123456789abcdef";

#define NUM_HEX_NIBBLES         (sizeof(uint32_t) * 2U)
#define HEX_BUFF_LEN            (NUM_HEX_NIBBLES + 3U)

SECTION_DEBUG_TEXT static size_t
dbg_format_copy_string(char *strp,
                       char *buffp,
                       size_t limit,
                       size_t field_width,
                       bool left_adjust)
{
    size_t      len;
    uint32_t    offset          = 0;
    size_t      width           = field_width;

    ASSERT(strp != NULL);
    ASSERT(buffp != NULL);

    if (width > limit) {
        width = limit;
    }

    len = strlen(strp);
    if (len > limit) {
        len = limit;
    }

    if (width > len) {
        memset(buffp, ' ', width);
    } else {
        width = len;
    }

    if (left_adjust) {
        offset = width - len;
    }

    memcpy(&buffp[offset], strp, len);

    return width;
}

SECTION_DEBUG_TEXT static void
int32_to_decimal_str(int32_t value,
                     size_t n_digits,
                     size_t field_width,
                     char *buffp,
                     format_flags_t flags)
{
    int32_t             val;
    uint32_t            i;
    uint32_t            digit;
    size_t              width           = field_width;
    bool                emitted         = false;
    bool                negative;
    bool                zero_pad;
    char                *bp             = buffp;
    const uint32_t      *divs;

    ASSERT(buffp != NULL);

    /* Handle n_digits == 0 */
    if (n_digits == 0UL) {
        *bp = ' ';
        return;
    }

    negative = value < 0;
    val = negative ? -value : value;

    if (width > n_digits) {
        width = n_digits;
    }

    zero_pad = (flags & DBG_FMT_FL_ZERO_PAD) != 0U;

    if ((flags & DBG_FMT_FL_BLANK_POSITIVE) != 0U) {
        *bp++ = negative ? '-' : ' ';
    }

    if ((flags & DBG_FMT_FL_ALWAYS_SIGN) != 0U) {
        *bp++ = negative ? '-' : '+';
    }

    if (((flags & (DBG_FMT_FL_BLANK_POSITIVE | DBG_FMT_FL_ALWAYS_SIGN)) == 0U)
        && negative) {
        *bp++ = '-';
    }

    divs = &divisors[NUM_DECIMAL_DIGITS - n_digits];

    for (i = 0U; i < n_digits; i += 1U) {
        digit = val / divs[i];
        val %= divs[i];

        if (zero_pad
            && (i >= (n_digits - width))) {
            emitted = true;
        }

        if ((digit != 0U)
            || emitted) {
            emitted = true;
            *bp++ = '0' + digit;
        }
    }

    /*
     * Handle the case where the value is 0 and there's
     * no zero pad.
     */
    if (!emitted) {
        *bp = '0';
    }
}

SECTION_DEBUG_TEXT static void
uint32_to_decimal_str(uint32_t value,
                      size_t n_digits,
                      size_t field_width,
                      char *buffp,
                      bool zero_pad)
{
    uint32_t            i;
    uint32_t            digit;
    size_t              width           = field_width;
    uint32_t            val             = value;
    bool                emitted         = false;
    char                *bp             = buffp;
    const uint32_t      *divs;

    ASSERT(buffp != NULL);

    /* Handle n_digits == 0 */
    if (n_digits == 0UL) {
        *bp = ' ';
        return;
    }

    if (width > n_digits) {
        width = n_digits;
    }

    divs = &divisors[NUM_DECIMAL_DIGITS - n_digits];

    for (i = 0U; i < n_digits; i += 1U) {
        digit = val / divs[i];
        val %= divs[i];

        if (zero_pad
            && (i >= (n_digits - width))) {
            emitted = true;
        }

        if ((digit != 0U)
            || emitted) {
            emitted = true;
            *bp++ = '0' + digit;
        }
    }

    /*
     * Handle the case where the value is 0 and there's
     * no zero pad.
     */
    if (!emitted) {
        *bp = '0';
    }
}

SECTION_DEBUG_TEXT static void
uint32_to_octal_str(uint32_t value,
                    size_t n_digits,
                    size_t field_width,
                    char *buffp,
                    bool zero_pad)
{
    uint32_t    i;
    uint32_t    digit;
    size_t      width           = field_width;
    char        *bp             = buffp;
    bool        emitted         = false;

    ASSERT(buffp != NULL);

    /* Handle n_digits == 0 */
    if (n_digits == 0UL) {
        *bp = ' ';
        return;
    }

    if (width > n_digits) {
        width = n_digits;
    }

    for (i = 0U; i < n_digits; i += 1U) {
        digit = (value >> (3U * (n_digits - 1U - i))) & 0x07U;

        if (zero_pad
            && (i >= (n_digits - width))) {
            emitted = true;
        }

        if ((digit != 0U)
            || emitted) {
            emitted = true;
            *bp++ = '0' + digit;
        }
    }

    /*
     * Handle the case where the value is 0 and there's
     * no zero pad.
     */
    if (!emitted) {
        *bp = '0';
    }
}

SECTION_DEBUG_TEXT static void
uint32_to_hex_str(uint32_t value,
                  size_t n_digits,
                  size_t field_width,
                  char *buffp,
                  const char *hex_chars,
                  bool zero_pad)
{
    uint32_t    i;
    uint32_t    nibble;
    size_t      width           = field_width;
    char        *bp             = buffp;
    bool        emitted         = false;

    ASSERT(buffp != NULL);
    ASSERT(hex_chars != NULL);

    /* Handle n_digits == 0 */
    if (n_digits == 0UL) {
        *bp = ' ';
        return;
    }

    if (width > n_digits) {
        width = n_digits;
    }

    for (i = 0U; i < n_digits; i += 1U) {
        nibble = (value >> (4U * (n_digits - 1U - i))) & 0x0FU;

        if (zero_pad
            && (i >= (n_digits - width))) {
            emitted = true;
        }

        if ((nibble != 0U)
            || emitted) {
            emitted = true;
            *bp++ = hex_chars[nibble];
        }
    }

    /*
     * Handle the case where the value is 0 and there's
     * no zero pad.
     */
    if (!emitted) {
        *bp = '0';
    }
}

SECTION_DEBUG_TEXT static void
dbg_format_signed_decimal(format_value_t value,
                          format_flags_t flags,
                          size_t field_width,
                          char **bufp,
                          size_t *countp)
{
    size_t      count;
    size_t      width                   = field_width;
    char        buff[DECIMAL_BUFF_LEN];
    char        *bp;
    int32_t     val                     = 0;
    uint32_t    n_digits                = 0;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;
    count = *countp;

    memset(buff, 0, DECIMAL_BUFF_LEN);

    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0) {
        val = value.uint8;
        n_digits = 3;
    }

    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0) {
        val = value.uint16;
        n_digits = 5;
    }

    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
        val = value.uint32;
        n_digits = 10;
    }

    int32_to_decimal_str(val, n_digits, width, buff, flags);

    width = dbg_format_copy_string(buff, bp, count, width,
                                   ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0));

    *bufp += width;
    *countp -= width;

}

SECTION_DEBUG_TEXT static void
dbg_format_unsigned_decimal(format_value_t value,
                            format_flags_t flags,
                            size_t field_width,
                            char **bufp,
                            size_t *countp)
{
    size_t      count;
    size_t      width                   = field_width;
    char        buff[DECIMAL_BUFF_LEN];
    char        *bp;
    uint32_t    val                     = 0;
    uint32_t    n_digits                = 0;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;
    count = *countp;

    memset(buff, 0, DECIMAL_BUFF_LEN);

    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0) {
        val = value.uint8;
        n_digits = 3;
    }

    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0) {
        val = value.uint16;
        n_digits = 5;
    }

    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
        val = value.uint32;
        n_digits = 10;
    }

    uint32_to_decimal_str(val, n_digits, width, buff,
                          (flags & DBG_FMT_FL_ZERO_PAD) != 0);

    width = dbg_format_copy_string(buff, bp, count, width,
                                   ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0));

    *bufp += width;
    *countp -= width;
}

SECTION_DEBUG_TEXT static void
dbg_format_octal(format_value_t value,
                 format_flags_t flags,
                 size_t field_width,
                 char **bufp,
                 size_t *countp)
{
    size_t      count;
    size_t      width                   = field_width;
    char        buff[OCTAL_BUFF_LEN];
    char        *bp;
    char        *sp;
    uint32_t    val                     = 0;
    uint32_t    n_digits                = 0;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;
    count = *countp;

    memset(buff, 0, OCTAL_BUFF_LEN);

    if ((flags & DBG_FMT_FL_ALTERNATE) != 0) {
        buff[0] = '0';
        sp = &buff[1];
    } else {
        sp = &buff[0];
    }

    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0) {
        val = value.uint8;
        n_digits = 3;
    }

    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0) {
        val = value.uint16;
        n_digits = 6;
    }

    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
        val = value.uint32;
        n_digits = 11;
    }

    uint32_to_octal_str(val, n_digits, width, sp,
                        (flags & DBG_FMT_FL_ZERO_PAD) != 0);

    width = dbg_format_copy_string(buff, bp, count, width,
                                   ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0));

    *bufp += width;
    *countp -= width;
}

SECTION_DEBUG_TEXT static void
dbg_format_lower_hex(format_value_t value,
                     format_flags_t flags,
                     size_t field_width,
                     char **bufp,
                     size_t *countp)
{
    char        *bp;
    size_t      count;
    size_t      width                   = field_width;
    char        buff[HEX_BUFF_LEN];
    char        *sp;
    uint32_t    val                     = 0;
    uint32_t    n_digits                = 0;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;
    count = *countp;

    memset(buff, 0, HEX_BUFF_LEN);

    if ((flags & DBG_FMT_FL_ALTERNATE) != 0) {
        buff[0] = '0';
        buff[1] = 'x';
        sp = &buff[2];
    } else {
        sp = &buff[0];
    }

    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0) {
        val = value.uint8;
        n_digits = sizeof(uint8_t) * 2U;
    }

    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0) {
        val = value.uint16;
        n_digits = sizeof(uint16_t) * 2U;
    }

    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
        val = value.uint32;
        n_digits = sizeof(uint32_t) * 2U;
    }

    uint32_to_hex_str(val, n_digits, width, sp, lower_hex,
                      (flags & DBG_FMT_FL_ZERO_PAD) != 0);

    width = dbg_format_copy_string(buff, bp, count, width,
                                   ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0));

    *bufp += width;
    *countp -= width;
}

SECTION_DEBUG_TEXT static void
dbg_format_upper_hex(format_value_t value,
                      format_flags_t flags,
                      size_t field_width,
                      char **bufp,
                      size_t *countp)
{
    char        *bp;
    size_t      count;
    size_t      width                   = field_width;
    char        buff[HEX_BUFF_LEN];
    char        *sp;
    uint32_t    val                     = 0;
    uint32_t    n_digits                = 0;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;
    count = *countp;

    memset(buff, 0, HEX_BUFF_LEN);

    if ((flags & DBG_FMT_FL_ALTERNATE) != 0) {
        buff[0] = '0';
        buff[1] = 'X';
        sp = &buff[2];
    } else {
        sp = &buff[0];
    }

    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0) {
        val = value.uint8;
        n_digits = sizeof(uint8_t) * 2U;
    }

    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0) {
        val = value.uint16;
        n_digits = sizeof(uint16_t) * 2U;
    }

    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
        val = value.uint32;
        n_digits = sizeof(uint32_t) * 2U;
    }

    uint32_to_hex_str(val, n_digits, width, sp, upper_hex,
                      (flags & DBG_FMT_FL_ZERO_PAD) != 0);

    width = dbg_format_copy_string(buff, bp, count, width,
                                   ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0));

    *bufp += width;
    *countp -= width;
}

SECTION_DEBUG_TEXT static void
dbg_format_char(char value,
                format_flags_t flags,
                size_t field_width,
                char **bufp,
                size_t *countp)
{
    char        *bp;
    size_t      width           = field_width;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;

    if (width > 0) {
        memset(bp, ' ', width);
    } else {
        width = 1;
    }

    if ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0) {
        bp[0] = value;
    } else {
        bp[width - 1] = value;
    }

    *bufp += width;
    *countp -= width;
}

static void
dbg_format_string(char *strp,
                  format_flags_t flags,
                  size_t field_width,
                  char **bufp,
                  size_t *countp)
{
    char        *bp;
    char        *sp;
    size_t      count;
    size_t      width           = field_width;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);
    ASSERT(countp != NULL);
    ASSERT(field_width <= *countp);

    bp = *bufp;
    count = *countp;

    if (strp == NULL) {
        sp = (char *)"(NULL)";
    } else {
        sp = strp;
    }

    width = dbg_format_copy_string(sp, bp, count, width,
                                   ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0));

    *bufp += width;
    *countp -= width;
}



SECTION_DEBUG_TEXT static size_t
dbg_get_num(char **bufp)
{
    size_t      value           = 0;
    char        *bp;

    ASSERT(bufp != NULL);
    ASSERT(*bufp != NULL);

    bp = *bufp;

    INLINE_RFD(MISRA, DEVIATE, Directive_4_6, "Approval: Bug 200543266, DR: SWE-FSP-003-SWSADR.docx");
    while ((*bp != '\0') && (isdigit((int)*bp) != 0)) {
        value *= 10;
        value += *bp - '0';
        bp += 1;
    }

    *bufp = bp;

    return value;
}

SECTION_DEBUG_TEXT static format_flags_t
dbg_process_modifiers(char **fmtp,
                      size_t count,
                      size_t *field_widthp)
{
    char                *fp;
    format_flags_t      flags           = 0;
    size_t              field_width     = 0;
    bool                find_flags      = true;

    ASSERT(fmtp != NULL);
    ASSERT(*fmtp != NULL);
    ASSERT(field_widthp != NULL);

    fp = *fmtp;

    if (count == 0) {
        goto out;
    }

    /*
     * Get the flag characters
     */
    while (find_flags) {
        switch (*fp) {
        case '#':
            flags |= DBG_FMT_FL_ALTERNATE;
            fp += 1;
            break;

        case '0':
            flags |= DBG_FMT_FL_ZERO_PAD;
            fp += 1;
            break;

        case '-':
            flags |= DBG_FMT_FL_LEFT_ADJUST;
            fp += 1;
            break;

        case ' ':
            flags |= DBG_FMT_FL_BLANK_POSITIVE;
            fp += 1;
            break;

        case '+':
            flags |= DBG_FMT_FL_ALWAYS_SIGN;
            fp += 1;
            break;

        default:
            find_flags = false;
            break;
        }
    }

    if (*fp == '\0') {
        goto out;
    }

    /*
     * Get the field width if there is one
     */
    field_width = dbg_get_num(&fp);
    if (field_width > count) {
        field_width = count;
    }

    if (*fp == '\0') {
        goto out;
    }

    /*
     * Get a length modifier if there is one
     */
    switch (*fp) {
    case 'h':
        fp += 1;
        if (*fp == 'h') {
            flags |= DBG_FMT_FL_LENGTH_CHAR;
            fp += 1;
        } else {
            flags |= DBG_FMT_FL_LENGTH_SHORT;
        }
        break;

    case 'l':
        fp += 1;
        if (*fp == 'l') {
            flags |= DBG_FMT_FL_LENGTH_LONG_LONG;
            fp += 1;
        } else {
            flags |= DBG_FMT_FL_LENGTH_LONG;
        }
        break;

    default:
        break;
    }

    /*
     * Clean up the flags
     */
    if ((flags & DBG_FMT_FL_LEFT_ADJUST) != 0) {
        flags &= ~DBG_FMT_FL_ZERO_PAD;
    }

    if ((flags & DBG_FMT_FL_ALWAYS_SIGN) != 0) {
        flags &= ~DBG_FMT_FL_BLANK_POSITIVE;
    }

    if ((flags & DBG_FMT_FL_LENGTH_MASK) == 0U) {
        flags |= DBG_FMT_FL_LENGTH_LONG;
    }

  out:
    *fmtp = fp;
    *field_widthp = field_width;

    return flags;
}

SECTION_DEBUG_TEXT int32_t
dbg_vsnprintf(char *buff,
              size_t size,
              const char *format,
              va_list ap)
{
    size_t              count           = size - 1;
    char                *bp             = buff;
    char                *fp             = (char *)format;
    format_flags_t      flags           = 0;
    size_t              field_width;
    format_value_t      val;

    ASSERT(buff != NULL);
    ASSERT(format != NULL);

    memset(bp, 0, size);

    while ((*fp != '\0')
           && (count > 0)) {
        switch (*fp) {
        case '%':
            fp += 1;
            if (*fp == '%') {
                *bp++ = *fp++;
                count -= 1;
            } else {
                flags = dbg_process_modifiers(&fp, count, &field_width);

                /*
                 * Get conversion
                 */
                switch (*fp) {
                case 'd':
                case 'i':
                    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
                        val.sint32 = va_arg(ap, int32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0U) {
                        val.sint8 = va_arg(ap, int32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0U) {
                        val.sint16 = va_arg(ap, int32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_LONG_LONG) != 0U) {
                        val.sint64 = va_arg(ap, int64_t);
                    }

                    dbg_format_signed_decimal(val, flags, field_width,
                                              &bp, &count);
                    fp += 1;
                    break;

                case 'o':
                    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
                        val.uint32 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0U) {
                        val.uint8 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0U) {
                        val.uint16 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_LONG_LONG) != 0U) {
                        val.uint64 = va_arg(ap, uint64_t);
                    }

                    dbg_format_octal(val, flags, field_width, &bp, &count);
                    fp += 1;
                    break;

                case 'u':
                    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
                        val.uint32 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0U) {
                        val.uint8 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0U) {
                        val.uint16 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_LONG_LONG) != 0U) {
                        val.uint64 = va_arg(ap, uint64_t);
                    }

                    dbg_format_unsigned_decimal(val, flags, field_width,
                                                &bp, &count);
                    fp += 1;
                    break;

                case 'x':
                    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
                        val.uint32 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0U) {
                        val.uint8 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0U) {
                        val.uint16 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_LONG_LONG) != 0U) {
                        val.uint64 = va_arg(ap, uint64_t);
                    }

                    dbg_format_lower_hex(val, flags, field_width, &bp, &count);
                    fp += 1;
                    break;

                case 'X':
                    if ((flags & DBG_FMT_FL_LENGTH_LONG) != 0) {
                        val.uint32 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_CHAR) != 0U) {
                        val.uint8 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_SHORT) != 0U) {
                        val.uint16 = va_arg(ap, uint32_t);
                    }

                    if ((flags & DBG_FMT_FL_LENGTH_LONG_LONG) != 0U) {
                        val.uint64 = va_arg(ap, uint64_t);
                    }

                    dbg_format_upper_hex(val, flags, field_width, &bp, &count);
                    fp += 1;
                    break;

                case 'p':
                    val.uint32 = va_arg(ap, uint32_t);
                    dbg_format_lower_hex(val, flags, field_width, &bp, &count);
                    fp += 1;
                    break;

                case 'c':
                    val.ch = va_arg(ap, int32_t);
                    dbg_format_char(val.ch, flags, field_width, &bp, &count);
                    fp += 1;
                    break;

                case 's':
                    val.string = va_arg(ap, char *);
                    dbg_format_string(val.string, flags, field_width,
                                      &bp, &count);
                    fp += 1;
                    break;

                default:
                    break;
                }
            }
            break;

        case '\n':
            *bp++ = '\r';
            if (count > 1) {
                *bp++ = '\n';
                count -= 1;
            }

            fp += 1;
            count -= 1;
            break;

        default:
            *bp++ = *fp++;
        }
    }

    return bp - buff;
}

SECTION_DEBUG_TEXT int32_t
dbg_snprintf(char *buff,
             size_t size,
             const char *format,
             ...)
{
    int32_t     count;
    va_list     ap;

    va_start(ap, format);

    count = dbg_vsnprintf(buff, size,
                          format, ap);

    va_end(ap);

    return count;
}

static char             print_buffer[DBG_PRINT_BUFF_LEN]        SECTION_DEBUG_DATA;

SECTION_DEBUG_TEXT int32_t
dbg_vprintf(const char *format,
            va_list ap)
{
    int32_t     count;

    enter_critical();

    count = dbg_vsnprintf(print_buffer, DBG_PRINT_BUFF_LEN,
                          format, ap);

    dbg_puts(print_buffer);

    exit_critical();

    return count;
}


SECTION_DEBUG_TEXT int32_t
dbg_printf(const char *format,
           ...)
{
    int32_t     count;
    va_list     ap;

    enter_critical();

    va_start(ap, format);
    count = dbg_vsnprintf(print_buffer, DBG_PRINT_BUFF_LEN,
                          format, ap);

    va_end(ap);

    dbg_puts(print_buffer);

    exit_critical();

    return count;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
