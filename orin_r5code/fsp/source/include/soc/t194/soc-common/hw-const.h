/*
 * Copyright (c) 2019-2020 NVIDIA CORPORATION.  All rights reserved.
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
#ifndef SOC_COMMON__HW_CONST_H
#define SOC_COMMON__HW_CONST_H
#define FSP__SOC_COMMON__HW_CONST_H                     1

#ifndef __ASSEMBLER__

#include <misc/macros.h>

START_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#ifndef MK_U32_CONST
#define MK_U32_CONST(_constant_)        (_constant_##UL)
#endif

#ifndef MK_ADDR_CONST
#define MK_ADDR_CONST(_constant_)       (_constant_##UL)
#endif

#ifndef MK_ENUM_CONST
#define MK_ENUM_CONST(_constant_)       (_constant_##UL)
#endif

#ifndef MK_MASK_CONST
#define MK_MASK_CONST(_constant_)       (_constant_##UL)
#endif

#ifndef MK_SHIFT_CONST
#define MK_SHIFT_CONST(_constant_)      (_constant_##UL)
#endif

#ifndef MK_FIELD_CONST
#define MK_FIELD_CONST(_mask_, _shift_) (MK_MASK_CONST(_mask_) << (_shift_))
#endif

#ifndef MK_U64_ADDR_CONST
#define MK_U64_ADDR_CONST(_constant_)   (_constant_##ULL)
#endif
END_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#else

#define MK_U32_CONST(_constant_)        (_constant_)
#define MK_ADDR_CONST(_constant_)       (_constant_)
#define MK_ENUM_CONST(_constant_)       (_constant_)
#define MK_MASK_CONST(_constant_)       (_constant_)
#define MK_SHIFT_CONST(_constant_)      (_constant_)
#define MK_U64_ADDR_CONST(_constant)    (_constant_)

#define MK_FIELD_CONST(_mask_, _shift_) (MK_MASK_CONST(_mask_) << (_shift_))

#endif /* __ASSEMBLER__ */

#endif
