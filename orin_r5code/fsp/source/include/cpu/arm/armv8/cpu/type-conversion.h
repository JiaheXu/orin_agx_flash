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
 *   fsp_c_v_ptr_to_v_ptr
 *   fsp_c_v_ptr_to_uptr
 *   fsp_c_v_ptr_to_u32
 *   fsp_upt_to_vl_u32_ptr
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
 * Converts uintptr_t to volatile uint32_t pointer
 *
 * @param[in] addr unsigned integer which will be converted to volatile unsigned integer pointer.
 * @param[out] value volatile unsigned integer pointer represented by unsigned integer addr
 */
static inline volatile uint32_t* fsp_upt_to_vl_u32_ptr(uintptr_t input)
{
    CT_ASSERT(FSP_ALIGNOF(uintptr_t) >= FSP_ALIGNOF(volatile uint32_t *), "ASSERT due to data misalignment.");
    CT_ASSERT(sizeof(uintptr_t) == sizeof(volatile uint32_t *), "ASSERT due to incompatible data sizes.");
    union cast_fsp_upt_to_vl_u32_ptr {
        uintptr_t input;
        volatile uint32_t *output;
    };
    union cast_fsp_upt_to_vl_u32_ptr t;

    t.input = input;
    return t.output;
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
