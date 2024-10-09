/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__ARMV7_EXCEPTION_H
#define CPU__ARMV7_EXCEPTION_H
#define FSP__CPU__ARMV7_EXCEPTION_H              1

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_20_10 \"Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx\") \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif
#define DATA_ABORT 0
#define PREFETCH_ABORT 1
#define UNDEFINED_INSTRUCTION 2
#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_20_10 MISRA_C_2012_Rule_2_5")
#endif

#ifndef __ASSEMBLER__

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

struct armv7_exception_frame {
        uint32_t sp;
        uint32_t lr;
        uint32_t spsr;
        uint32_t r0;
        uint32_t r1;
        uint32_t r2;
        uint32_t r3;
        uint32_t r4;
        uint32_t r5;
        uint32_t r6;
        uint32_t r7;
        uint32_t r8;
        uint32_t r9;
        uint32_t r10;
        uint32_t r11;
        uint32_t r12;
        uint32_t pc;
};

void armv7_exception(uint32_t type, struct armv7_exception_frame *frame);

typedef struct {
    uint32_t                            type;
    struct armv7_exception_frame        *frame;
    uint32_t                            dfar;
    uint32_t                            dfsr;
    uint32_t                            adfsr;
    uint32_t                            ifar;
    uint32_t                            ifsr;
    uint32_t                            aifsr;
} armv7_exception_t;


void exception_hook(const armv7_exception_t * const exception);

#endif /* !defined __ASSEMBLER__ */

#endif /* !defined CPU__ARMV7_EXCEPTION_H */
