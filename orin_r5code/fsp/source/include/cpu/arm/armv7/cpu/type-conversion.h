/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__TYPE_CONVERSION_H
#define CPU__TYPE_CONVERSION_H
#define FSP__CPU__TYPE_CONVERSION_H                     1

/* Compiler headers */
#include <stdint.h>
#include <string.h>             // for NULL, size_t

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>
#include <misc/macros.h>         // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */

/*
 * Table of abbreviations:
 *   These abbreviations are used in type conversions.  The following letters, or combinations of
 *   letters, preceeded by an underscore, shall mean the following:
 * _c       : const
 * _ch      : char
 * _fn      : function
 * _i32     : int32_t
 * _li      : long int
 * _ptr     : pointer
 * _st      : size_t
 * _u8      : uint8_t
 * _u32     : uint32_t
 * _upt     : uintptr_t
 * _v       : void
 * _vl      : volatile
 *
 */

/*
 * Table of Contents (abbreviations decoded as per table above):
 *   fsp_c_v_ptr_to_c_ch_ptr
 *   fsp_c_ch_ptr_to_v_ptr
 *   fsp_c_v_ptr_to_v_ptr
 *   fsp_c_ch_ptr_to_ch_ptr
 *   fsp_i32_to_u32
 *   fsp_li_to_st
 *   fsp_i32_to_st
 *   fsp_ch_to_u8
 *   fsp_v_ptr_to_u8_ptr
 *   fsp_c_v_ptr_to_c_u8_ptr
 *   fsp_c_v_ptr_to_uptr
 *   fsp_c_v_ptr_to_u32
 *   fsp_c_u32_ptr_to_u32
 *   fsp_c_u32_ptr_to_u64
 *   fsp_c_u8_ptr_to_u32
 *   fsp_u32_to_vl_u32_ptr
 *   fsp_v_fn_ptr_to_uptr
 *
 */

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
                MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")

/**
 * Converts constant void pointer to constant char pointer.
 *
 * @param[in] ptr constant void pointer which will be coverted to constant char pointer.
 * @param[out] ptr constant char pointer of constant void pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline const char *fsp_c_v_ptr_to_c_ch_ptr(const void *ptr)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    return (const char *)ptr;
}

/**
 * Converts constant char pointer to void pointer. Discards const correctness.
 *
 * @param[in] input constant char pointer which will be coverted to void pointer.
 * @param[out] output void pointer of constant char pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline void *fsp_c_ch_ptr_to_v_ptr(const char *input)
{
CT_ASSERT(FSP_ALIGNOF(const char *) >= FSP_ALIGNOF(void *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const char *) == sizeof(void *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_ch_ptr_to_v_ptr {
        const char *input;
        void * output;
    };
    union cast_fsp_c_ch_ptr_to_v_ptr t;

    t.input= input;
    return t.output;
}

/**
 * Converts constant void pointer non-constant.  Preserves 'const' for R-O APIs,
 * and allows non-const for R-W APIs.
 *
 * @param[in] input constant void pointer which will be coverted to void pointer
 * @param[out] output void pointer of constant void pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline void *fsp_c_v_ptr_to_v_ptr(const void *input)
{
CT_ASSERT(FSP_ALIGNOF(const void *) >= FSP_ALIGNOF(void *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const void *) == sizeof(void *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_v_ptr_to_v_ptr {
        const void *input;
        void * output;
    };
    union cast_fsp_c_v_ptr_to_v_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts constant char pointer to char pointer
 *
 * @param[in] input constant char pointer which will be coverted to char pointer
 * @param[out] output char pointer of constant char pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline char* fsp_c_ch_ptr_to_ch_ptr(const char *input)
{
CT_ASSERT(FSP_ALIGNOF(const char *) >= FSP_ALIGNOF(char *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const char *) == sizeof(char *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_ch_ptr_to_ch_ptr {
        const char *input;
        char* output;
    };
    union cast_fsp_c_ch_ptr_to_ch_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts signed int to unsigned int
 *
 * @param[in] val int32_t which will be coverted to unsigned integer.
 * @param[out] u_val unsigned integer value of int32_t.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint32_t fsp_i32_to_u32(int32_t input)
{
    union cast_fsp_i32_to_u32 {
        int32_t input;
        uint32_t output;
    };
    union cast_fsp_i32_to_u32 t;

    t.input = input;
    return t.output;
}

START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_6, "Approval: Bug 200543266, DR: SWE-FSP-003-SWSADR.docx")
/**
 * Converts signed long int to size_t
 *
 * @param[in] val long int which will be coverted to size_t.
 * @param[out] u_val size_t value of int.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline size_t fsp_li_to_st(long int input)
{
    union cast_fsp_li_to_st {
        long int input;
        size_t output;
    };
    union cast_fsp_li_to_st t;

    t.input = input;
    return t.output;
}
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_6, "Approval: Bug 200543266, DR: SWE-FSP-003-SWSADR.docx")

/**
 * Converts signed int to size_t
 *
 * @param[in] val int which will be coverted to size_t.
 * @param[out] u_val size_t value of int.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline size_t fsp_i32_to_st(int32_t input)
{
    union cast_fsp_i32_to_st {
        int32_t input;
        size_t output;
    };
    union cast_fsp_i32_to_st t;

    t.input = input;
    return t.output;
}

/**
 * Converts char to uint8_t
 *
 * @param[in] ch character which will be converted to unsigned character.
 * @param[out] uchar unsigned char of given character.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint8_t fsp_ch_to_u8(char input)
{
CT_ASSERT(FSP_ALIGNOF(char) >= FSP_ALIGNOF(uint8_t), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(char) == sizeof(uint8_t), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_ch_to_u8 {
        char input;
        uint8_t output;
    };
    union cast_fsp_ch_to_u8 t;

    t.input = input;
    return t.output;
}

/**
 * Converts void pointer to struct uint8_t pointer.
 *
 * @param[in] input void pointer which will be coverted to uint8_t pointer.
 * @param[out] output uint8_t pointer of void pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint8_t *fsp_v_ptr_to_u8_ptr(void *input)
{
CT_ASSERT(FSP_ALIGNOF(void *) >= FSP_ALIGNOF(uint8_t*), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(void *) == sizeof(uint8_t *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_v_ptr_to_u8_ptr {
        void * input;
        uint8_t *output;
    };
    union cast_fsp_v_ptr_to_u8_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts constant void pointer to struct constant uint8_t pointer.
 *
 * @param[in] input constant void pointer which will be coverted to constant uint8_t pointer.
 * @param[out] output constant uint8_t pointer of constant void pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline const uint8_t *fsp_c_v_ptr_to_c_u8_ptr(const void *input)
{
CT_ASSERT(FSP_ALIGNOF(const void *) >= FSP_ALIGNOF(const uint8_t *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const void *) == sizeof(const uint8_t *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_v_ptr_to_c_u8_ptr {
        const void * input;
        const uint8_t *output;
    };
    union cast_fsp_c_v_ptr_to_c_u8_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts const void pointer to uintptr_t
 *
 * @param[in] addr void pointer which will be coverted to uintptr_t
 * @param[out] value uintptr_t value of void pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uintptr_t fsp_c_v_ptr_to_uptr(const void *input)
{
    CT_ASSERT(FSP_ALIGNOF(const void *) >= FSP_ALIGNOF(uintptr_t *), "ASSERT due to data misalignment.");
    CT_ASSERT(sizeof(const void *) == sizeof(uintptr_t *), "ASSERT due to incompatible pointer sizes.");
    union cast_fsp_c_v_ptr_to_uptr {
        const void *input;
        uintptr_t output;
    };
    union cast_fsp_c_v_ptr_to_uptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts const void pointer to uint32_t
 *
 * @param[in] addr void pointer which will be coverted to uint32_t
 * @param[out] value uint32_t value of void pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint32_t fsp_c_v_ptr_to_u32(const void *input)
{
    CT_ASSERT(FSP_ALIGNOF(const void *) >= FSP_ALIGNOF(uint32_t), "ASSERT due to data misalignment.");
    CT_ASSERT(sizeof(const void *) == sizeof(uint32_t), "ASSERT due to incompatible pointer sizes.");
    union cast_fsp_c_v_ptr_to_u32 {
        const void *input;
        uint32_t output;
    };
    union cast_fsp_c_v_ptr_to_u32 t;

    t.input = input;
    return t.output;
}

/**
 * Converts const uint32_t pointer to uint32_t
 *
 * @param[in] addr uint32_t pointer which will be coverted to uint32_t
 * @param[out] value uint32_t value of uint32_t pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint32_t fsp_c_u32_ptr_to_u32(const uint32_t *input)
{
    CT_ASSERT(sizeof(const uint32_t *) == sizeof(uint32_t), "ASSERT due to incompatible pointer sizes.");
    union cast_fsp_c_u32_ptr_to_u32 {
        const uint32_t *input;
        uint32_t output;
    };
    union cast_fsp_c_u32_ptr_to_u32 t;

    t.input = input;
    return t.output;
}

/**
 * Converts const uint8_t pointer to uint32_t
 *
 * @param[in] addr uint8_t pointer which will be coverted to uint32_t
 * @param[out] value uint32_t value of uint8_t pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint32_t fsp_c_u8_ptr_to_u32(const uint8_t *input)
{
    CT_ASSERT(sizeof(const uint32_t *) == sizeof(uint32_t), "ASSERT due to incompatible pointer sizes.");
    union cast_fsp_c_u8_ptr_to_u32 {
        const uint8_t *input;
        uint32_t output;
    };
    union cast_fsp_c_u8_ptr_to_u32 t;

    t.input = input;
    return t.output;
}

/**
 * Converts const uint32_t pointer to uint64_t
 *
 * @param[in] addr uint32_t pointer which will be coverted to uint64_t
 * @param[out] value uint64_t value of uint32_t pointer.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uint64_t fsp_c_u32_ptr_to_u64(const uint32_t *input)
{
    CT_ASSERT(sizeof(const uint32_t *) == sizeof(uint32_t), "ASSERT due to incompatible pointer sizes.");
    union cast_fsp_c_u32_ptr_to_u64 {
        const uint32_t *input;
        uint64_t output;
    };
    union cast_fsp_c_u32_ptr_to_u64 t;

    t.input = input;
    return t.output;
}

/**
 * Converts uint32_t to volatile uint32_t pointer
 *
 * @param[in] addr unsigned integer which will be converted to volatile unsigned integer pointer.
 * @param[out] value volatile unsigned integer pointer represented by unsigned integer addr
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline volatile uint32_t* fsp_u32_to_vl_u32_ptr(uint32_t input)
{
    CT_ASSERT(FSP_ALIGNOF(uint32_t) >= FSP_ALIGNOF(volatile uint32_t *), "ASSERT due to data misalignment.");
    CT_ASSERT(sizeof(uint32_t) == sizeof(volatile uint32_t *), "ASSERT due to incompatible data sizes.");
    union cast_fsp_u32_to_vl_u32_ptr {
        uint32_t input;
        volatile uint32_t *output;
    };
    union cast_fsp_u32_to_vl_u32_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts void (function pointer) to uintptr_t
 *
 * @param[in] fun_ptr pointer to function.
 * @param[out] value uint32_t value of function pointer
 */
INLINE_RFD(MISRA, DEVIATE, Rule_5_9, "Approval:  Bug 3255774, DR: SWE-FSP-056-SWSADR.docx")
static inline uintptr_t fsp_v_fn_ptr_to_uptr(void (*input)(void))
{
    CT_ASSERT(FSP_ALIGNOF(void (*)(void)) >= FSP_ALIGNOF(uintptr_t), "ASSERT due to data misalignment.");
    CT_ASSERT(sizeof(void (*)(void)) == sizeof(uintptr_t), "ASSERT due to incompatible data sizes.");
    union cast_fsp_v_fn_ptr_to_uptr {
        void (*input)(void);
        uintptr_t output;
    };
    union cast_fsp_v_fn_ptr_to_uptr t;

    t.input = input;
    return t.output;
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
