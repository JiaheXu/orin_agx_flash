/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef MISC__BITOPS_H
#define MISC__BITOPS_H
#define FSP__MISC__BITOPS_H                             1

#ifndef __ASSEMBLER__
/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */

/* Hardware headers */
#include <misc/ct-assert.h>
#include <misc/macros.h>         // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                CERTC, DEVIATE, DCL37_C, "Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file bitops.h
 * @brief functions and macros that provide bit manipulation support.
 */

/**
 * @brief define a bit mask for the 16-bit value
 *
 * @jama_func_req_id 8218676
 *
 * This macro will set a bit represented by the bit-index in a half word.
 *
 * @param[in] _b_ 16-bit immediate value
 *
 * @result 16-bit value with the input bit-index set to 1.
 */
#define BIT16(_b_)  ((uint16_t)(((uint16_t)1U) << (_b_)))

/**
 * @brief define a bit mask for the 16-bit value
 *
 * @jama_func_req_id 8218676
 *
 * This macro will set a bit represented by the bit-index in a half word.
 *
 * @param[in] _b_ 16-bit variable
 *
 * @result 16-bit value with the input bit-index set to 1.
 */
static inline uint16_t BIT16_FN(uint16_t varbit)
{
    uint16_t retval = 0U;
    if(varbit < 16U) {
        INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-616, DR: SWE-FSP-050-SWSADR.docx");
        retval = (uint16_t)((uint16_t)1U << varbit);
    }
    return retval;
}

/**
 * @brief create bit mask of a specific field in a 16-bit value
 *
 * @jama_func_req_id 8245844
 *
 * This macro will create the bit mask of a field specified by msb and lsb
 * indices in a 16-bit value.
 *
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 *
 * @result bit mask of the field specified by msb and lsb indices.
 */
#define MASK16(_msb_, _lsb_)                                                    \
    (((uint16_t)(BIT16(_msb_) - (uint16_t)1U) | BIT16(_msb_)) &                 \
     ((uint16_t)~(BIT16(_lsb_) - (uint16_t)1U)))

/**
 * @brief Extract a field specified by start and end bit in a 16-bit value
 *
 * @jama_func_req_id 8220128
 *
 * This macro will extract the bit-field specified by msb and lsb
 * indices in a 16-bit value.
 *
 * @param[in] _x_ 16-bit value
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 * @param[in] _type_ return data type
 *
 * @result field specified by [msb, lsb] casted to return data type
 */
#define EXTRACT16(_x_, _msb_, _lsb_, _type_)                                    \
    ((_type_)((_type_)((_x_) & MASK16(_msb_, _lsb_)) >> (_lsb_)))

/**
 * @brief Write to a specific field in a 16-bit value
 *
 * @jama_func_req_id 8245826
 *
 * This macro will write to a bit-field specified by msb and lsb
 * indices in a 16-bit value.
 *
 * @param[in] _x_ 16-bit value
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 * @param[in] _value_ value to be written to the specific field
 *
 * @result 16-bit value with the modified field specified by msb and lsb indices
 */
#define INSERT16(_x_, _msb_, _lsb_, _value_)                                    \
    ((((uint16_t)_x_) & ((uint16_t)~MASK16(_msb_, _lsb_)))                      \
     | ((((uint16_t)_value_) << _lsb_) & MASK16(_msb_, _lsb_)))

/**
 * @brief define a bit mask for the 32-bit value
 *
 * @jama_func_req_id 8218670
 *
 * This macro will set a bit represented by the bit-index in a word.
 *
 * @param[in] _b_ 32-bit immediate value
 *
 * @result 32-bit value with the input bit-index set to 1.
 */
#define BIT32(_b_)  ((uint32_t)(((uint32_t)1U) << (_b_)))

/**
 * @brief define a bit mask for the 32-bit value
 *
 * @jama_func_req_id 8218670
 *
 * This macro will set a bit represented by the bit-index in a word.
 *
 * @param[in] _b_ 32-bit variable
 *
 * @result 32-bit value with the input bit-index set to 1.
 */
static inline uint32_t BIT32_FN(uint32_t varbit)
{
    uint32_t retval = 0U;
    if(varbit < 32U) {
        retval = (uint32_t)((uint32_t)1U << varbit);
    }
    return retval;
}

/**
 * @brief create bit mask of a specific field in a 32-bit value
 *
 * @jama_func_req_id 8245850
 *
 * This macro will create the bit mask of a field specified by msb and lsb
 * indices in a 32-bit value.
 *
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 *
 * @result bit mask of the field specified by msb and lsb indices.
 */
#define MASK32(_msb_, _lsb_)                                                    \
    (((uint32_t)(BIT32(_msb_) - (uint32_t)1U) | BIT32(_msb_)) & ((uint32_t)~(BIT32(_lsb_) - (uint32_t)1U)))

/**
 * @brief Extract a field specified by start and end bit in a 32-bit value
 *
 * @jama_func_req_id 8220131
 *
 * This macro will extract the bit-field specified by msb and lsb
 * indices in a 32-bit value.
 *
 * @param[in] _x_ 32-bit value
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 * @param[in] _type_ return data type
 *
 * @result field specified by [msb, lsb] casted to return data type
 */
#define EXTRACT32(_x_, _msb_, _lsb_, _type_)                                    \
    ((_type_)((_type_)((_x_) & MASK32(_msb_, _lsb_)) >> (_lsb_)))

/**
 * @brief Write to a specific field in a 32-bit value
 *
 * @jama_func_req_id 8245829
 *
 * This macro will write to a bit-field specified by msb and lsb
 * indices in a 32-bit value.
 *
 * @param[in] _x_ 32-bit value
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 * @param[in] _value_ value to be written to the specific field
 *
 * @result 32-bit value with the modified field specified by msb and lsb indices
 */
#define INSERT32(_x_, _msb_, _lsb_, _value_)                                    \
    ((((uint32_t)_x_) & ((uint32_t)~MASK32(_msb_, _lsb_)))                      \
     | ((((uint32_t)_value_) << _lsb_) & MASK32(_msb_, _lsb_)))

/*
 * Aliases for 32-bit operations
 */
#define BIT(_x_)                            BIT32(_x_)
#define MASK(_msb_, _lsb_)                  MASK32(_msb_, _lsb_)
#define EXTRACT(_x_, _msb_, _lsb_, _type_)  EXTRACT32(_x_, _msb_, _lsb_, _type_)
#define INSERT(_x_, _msb_, _lsb_, _value_)  INSERT32(_x_, _msb_, _lsb_, _value_)

/**
 * @brief define a bit mask for the 64-bit value
 *
 * @jama_func_req_id 8218694
 *
 * This macro will set a bit represented by the bit-index in a 64-bit value.
 *
 * @param[in] _b_ 64-bit immediate value
 *
 * @result 64-bit value with the input bit-index set to 1.
 */
#define BIT64(_b_)  ((uint64_t)(((uint64_t)1U) << (_b_)))

/**
 * @brief define a bit mask for the 64-bit value
 *
 * @jama_func_req_id 8218694
 *
 * This macro will set a bit represented by the bit-index in a 64-bit value.
 *
 * @param[in] _b_ 64-bit variable
 *
 * @result 64-bit value with the input bit-index set to 1.
 */
static inline uint64_t BIT64_FN(uint64_t varbit)
{
    uint64_t retval = 0U;
    if(varbit < 64U) {
        retval = (uint64_t)((uint64_t)1U << varbit);
    }
    return retval;
}

/**
 * @brief create bit mask of a specific field in a 64-bit value
 *
 * @jama_func_req_id 8245856
 *
 * This macro will create the bit mask of a field specified by msb and lsb
 * indices in a 64-bit value.
 *
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 *
 * @result bit mask of the field specified by msb and lsb indices.
 */
#define MASK64(_msb_, _lsb_)                                                    \
    (((uint64_t)(BIT64(_msb_) - (uint64_t)1U) | BIT64(_msb_)) & ((uint64_t)~(BIT64(_lsb_) - (uint64_t)1U)))

/**
 * @brief Extract a field specified by start and end bit in a 64-bit value
 *
 * @jama_func_req_id 8221487
 *
 * This macro will extract the bit-field specified by msb and lsb
 * indices in a 64-bit value.
 *
 * @param[in] _x_ 64-bit value
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 * @param[in] _type_ return data type
 *
 * @result field specified by [msb, lsb] casted to return data type
 */
#define EXTRACT64(_x_, _msb_, _lsb_, _type_)                                    \
    ((_type_)((_type_)((_x_) & MASK64(_msb_, _lsb_)) >> (_lsb_)))

/**
 * @brief Write to a specific field in a 64-bit value
 *
 * @jama_func_req_id 8245835
 *
 * This macro will write to a bit-field specified by msb and lsb
 * indices in a 64-bit value.
 *
 * @param[in] _x_ 64-bit value
 * @param[in] _msb_ msb index
 * @param[in] _lsb_ lsb index
 * @param[in] _value_ value to be written to the specific field
 *
 * @result 64-bit value with the modified field specified by msb and lsb indices
 */
#define INSERT64(_x_, _msb_, _lsb_, _value_)                                    \
    ((((uint64_t)_x_) & ((uint64_t)~MASK64(_msb_, _lsb_)))                      \
     | ((((uint64_t)_value_) << _lsb_) & MASK64(_msb_, _lsb_)))

/**
 * @brief Pack a 64-bit value from upper & lower 32-bit values
 *
 * @jama_func_req_id 8218523
 *
 * This macro creates the 64-bit value resulting from the MSW(upper 32-bit)
 * and LSW(lower 32-bit) values.
 *
 * @param[in] _h_ upper 32-bit value
 * @param[in] _l_ lower 32-bit value
 *
 * @result - 64-bit value resulting from the MSW and LSW
 */
#define PACK64(_l_, _h_)        ((((uint64_t)(_h_)) << 32) | (_l_))

/**
 * @brief Fetch the Most Significant Word from 64-bit value
 *
 * @jama_func_req_id 8218616
 *
 * This macro will fetch the Most significant word (lower 32-bits)
 * from a 64-bit value.
 *
 * @param[in] _x_ 64-bit value
 *
 * @result Most significant word (upper 32-bits)
 */
#define HI32(_x_)               ((uint32_t)(((_x_) >> 32U) & 0xFFFFFFFFU))

/**
 * @brief Fetch the Least Significant Word from 64-bit value
 *
 * @jama_func_req_id 8218625
 *
 * This macro will fetch the Most significant word (upper 32-bits)
 * from a 64-bit value.
 *
 * @param[in] _x_ 64-bit value
 *
 * @result Least significant word (lower 32-bits)
 */
#define LOW32(_x_)              ((uint32_t)((_x_) & 0xFFFFFFFFU))

/*!
 * @brief Utility functions to set/clear/test a bit in a bitmap.
 * The bitmap is an array, allowing it to support more than 64 bits.
 */

/*!
 * @brief The type used for each element of a bitmap.
 * Note: Ideally we would use uintptr_t here to match the word size of this architecture.
 * However, This requires us to do
 *    #define BITMAP_ELEMENT_SIZE (sizeof(bitmap_element_t) * 8U)
 * And this causes a CERT-C ARR39-C. Because of this, we have to fix it at 64 bits.
 */
typedef uint64_t bitmap_element_t;

/*!
 * @brief Number of bits per array element of _mpuEnableBitmap.
 */
#define BITMAP_ELEMENT_SIZE 64U

/*!
 * @brief Utility functions to set/clear/test a bit in a bitmap
 *
 * @param bm the bitmap in which to set/clear/test a bit
 * @param index the index of the bit to set/clear/test
 *
 * @note It is up to the caller to ensure that index is within the bounds of bm.
 *      bm must be at least the size of index / BITMAP_ELEMENT_SIZE.
 */

/**
 * @brief set a bit in a bitmap
 *
 * @jama_func_req_id TODO
 *
 * This function will set a bit at position index in a bitmap spanning an array.
 *
 * @param bm the bitmap in which to set a bit
 * @param index the index of the bit to set
 */
static inline void bitmap_set_bit(bitmap_element_t bm[], uint32_t bm_size, uint32_t index)
{
    if ((index / BITMAP_ELEMENT_SIZE) < bm_size)
    {
        bm[index / BITMAP_ELEMENT_SIZE] |=
                (bitmap_element_t)((bitmap_element_t)1U << (index % BITMAP_ELEMENT_SIZE));
    }
}

/**
 * @brief clear a bit in a bitmap
 *
 * @jama_func_req_id TODO
 *
 * This function will clear a bit at position index in a bitmap spanning an array.
 *
 * @param bm the bitmap in which to clear a bit
 * @param index the index of the bit to clear
 */
static inline void bitmap_clear_bit(bitmap_element_t bm[], uint32_t bm_size,  uint32_t index)
{
    if ((index / BITMAP_ELEMENT_SIZE) < bm_size)
    {
        bm[index / BITMAP_ELEMENT_SIZE] &=
                (bitmap_element_t)(~((bitmap_element_t)1U << (index % BITMAP_ELEMENT_SIZE)));
    }
}

/**
 * @brief test a bit in a bitmap
 *
 * @jama_func_req_id TODO
 *
 * This function will test a bit at position index in a bitmap spanning an array.
 *
 * @param bm the bitmap in which to test a bit
 * @param index the index of the bit to test
 *
 * @result True if the bit is set, False otherwise
 */
static inline bool bitmap_test_bit(const bitmap_element_t bm[], uint32_t bm_size, uint32_t index)
{
    bool ret = false;
    if ((index / BITMAP_ELEMENT_SIZE) < bm_size)
    {
        ret = (((bm[index / BITMAP_ELEMENT_SIZE] >> (index % BITMAP_ELEMENT_SIZE)) & 1U) == 1U);
    }

    return ret;
}

#if __GNUC__ > 0
#define HAS_BUILTIN_CTZ         1
#else
    #if __has_builtin(__builtin_ctz)
    #define HAS_BUILTIN_CTZ     1
    #else
    #define HAS_BUILTIN_CTZ     0
    #endif
#endif

/**
 * @brief Compute the set bit index from the least significant bit
 *
 * @jama_func_req_id 8218484
 *
 * This function will fetch the bit index of the set bit from the LSB. In
 * other words, counts the number of trailing zeroes.
 *
 * @param[in] value 32-bit value
 *
 * @return number of trailing zeroes or index of the bit set from LSB
 */
static inline uint32_t bit_number(uint32_t value)
{
#if HAS_BUILTIN_CTZ == 1
        int32_t ret;
        uint32_t n = 0;
	INLINE_RFD(MISRA, DEVIATE, Directive_4_6, "Approval: Bug 200543266, DR: SWE-FSP-003-SWSADR.docx")
        ret = (int32_t)__builtin_ctz((unsigned int)value);
        if (ret > 0) {
                n = (uint32_t)ret;
        }

        return n;
#else

        uint32_t lsb_only = value & ~(value - 1U);
        uint32_t n = 0;

        if (lsb_only == 0U)
                n += 32U;
        if ((lsb_only & 0xffff0000U) != 0U)
                n += 16U;
        if ((lsb_only & 0xff00ff00U) != 0U)
                n += 8U;
        if ((lsb_only & 0xf0f0f0f0U) != 0U)
                n += 4U;
        if ((lsb_only & 0xccccccccU) != 0U)
                n += 2U;
        if ((lsb_only & 0xaaaaaaaaU) != 0U)
                n += 1U;

        return n;
#endif
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")
static inline uint32_t next_bit(
        uint32_t firstbit,
        uint32_t nbits,
        const uint32_t words[],
        uint32_t invert)
{
        uint32_t word;
        uint32_t lsb_only;
        uint32_t fbit = firstbit;
        uint32_t ret;

        for (;;) {
                if (fbit >= nbits) {
                        ret = nbits;
                        goto out;
                }

                word = words[fbit / 32U] ^ invert;

                /* Mask before firstbit */
                word &= (uint32_t)(~((1UL << (fbit % 32UL)) - 1UL));

                if (word != 0U) {
                        break;
                }

                fbit = (fbit + 32U) & ~31U;
        }

        lsb_only = word & ~(word - 1U);

        fbit = (fbit & ~31U) + bit_number(lsb_only);

        if (fbit >= nbits) {
                ret = nbits;
                INLINE_RFD(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx");
                goto out;
        }

        ret = fbit;

out:
        return ret;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")

static inline uint32_t next_one(
        uint32_t firstbit,
        uint32_t nbits,
        const uint32_t words[])
{
        return next_bit(firstbit, nbits, words, 0U);
}

static inline uint32_t next_zero(
        uint32_t firstbit,
        uint32_t nbits,
        const uint32_t words[])
{
        return next_bit(firstbit, nbits, words, ~0U);
}

/**
 * @brief create a 64-bit value from MSW & LSW
 *
 * @jama_func_req_id 8218523
 *
 * This function creates the 64-bit value resulting from the MSW(upper 32-bit)
 * and LSW(lower 32-bit) values.
 *
 * @param[in] hi32 upper 32-bit value
 * @param[in] lo32 lower 32-bit value
 *
 * @result - 64-bit value resulting from the MSW and LSW
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx")
static inline uint64_t hilo_to_64(
        uint32_t hi32,
        uint32_t lo32)
{
#if defined(__BYTE_ORDER__)
        union {
                struct {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
                        uint32_t lo32, hi32;
#else
                        uint32_t hi32, lo32;
#endif
                } value32;
                uint64_t value64;
        } value = { .value32 = { .hi32 = hi32, .lo32 = lo32 }};

        return value.value64;
#else
        return ((uint64_t)hi32 << 32) | lo32;
#endif
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx")

#endif

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              CERTC, DEVIATE, DCL37_C, "Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx")
#endif /* MISC__BITOPS_H */
