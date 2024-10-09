/**
 * @file gcc_builtins.h
 *
 * @brief GCC builtin macros
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS`` AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MISC__GCC_BUILTINS_H
#define MISC__GCC_BUILTINS_H
#define FSP__MISC__GCC_BUILTINS_H                         1

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif

/*
 * GCC Builtins
 * Defines macros for GCC builtin functions
 */

#define NVRV_BUILTIN_OFFSETOF(type, member) __builtin_offsetof(type, member)
#define NVRV_BUILTIN_UNREACHABLE            __builtin_unreachable

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_20_10 MISRA_C_2012_Rule_2_5")
#endif

#endif /* MISC__GCC_BUILTINS_H */
/** end of file **/
