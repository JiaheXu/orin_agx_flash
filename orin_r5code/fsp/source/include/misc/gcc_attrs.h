/**
 * @file gcc_attrs.h
 *
 * @brief GCC attribute macros
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

#ifndef MISC__GCC_ATTRS_H
#define MISC__GCC_ATTRS_H
#define FSP__MISC__GCC_ATTRS_H                         1

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif

/*
 * GCC Attributes
 * Defines macros for GCC attributes
 */

/*
 * Common attributes
 */
#define GCC_ATTR_ALIGNED(X)                        __attribute__((aligned((X))))
#define GCC_ATTR_SECTION(X)                        __attribute__((section((X))))
#define GCC_ATTR_UNUSED                            __attribute__((unused))
#define GCC_ATTR_USED                              __attribute__((used))
#define GCC_ATTR_VISIBILITY_HIDDEN                 __attribute__((visibility("hidden")))
#define GCC_ATTR_VISIBILITY_INTERNAL               __attribute__((visibility("internal")))
#define GCC_ATTR_VISIBILITY_PROTECTED              __attribute__((visibility("protected")))
#define GCC_ATTR_WEAK                              __attribute__((weak))

/*
 * Function only attributes
 */
#define GCC_ATTR_ALWAYSINLINE                      __attribute__((always_inline))
#define GCC_ATTR_FORMAT_ARG(Idx)                   __attribute__((format_arg((Idx))))
#define GCC_ATTR_FORMAT_PRINTF(Idx, firstToCheck)  __attribute__((format(printf, (Idx), (firstToCheck))))
#define GCC_ATTR_NAKED                             __attribute__((naked))
#define GCC_ATTR_NOINLINE                          __attribute__((noinline))
#define GCC_ATTR_NONNULL(...)                      __attribute__((nonnull(__VA_ARGS__)))
#define GCC_ATTR_NONNULL_ALL                       __attribute__((nonnull))
#define GCC_ATTR_NORETURN                          __attribute__((noreturn))
#define GCC_ATTR_PURE                              __attribute__((pure))
#define GCC_ATTR_WARN_UNUSED                       __attribute__((warn_unused_result))
#define GCC_ATTR_NO_SSP                            __attribute__((__optimize__("no-stack-protector")))

/*
 * Variable only attributes
 */
#define GCC_ATTR_CLEANUP(cleanupFcn)               __attribute__((cleanup((cleanupFcn))))
#define GCC_ATTR_PACKED                            __attribute__((packed))

/*
 * Misc attributes
 */
#define GCC_ATTR_FALLTHROUGH                       __attribute__((fallthrough))

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_20_10 MISRA_C_2012_Rule_2_5")
#endif

#endif /* MISC__GCC_ATTRS_H */
/** end of file **/
