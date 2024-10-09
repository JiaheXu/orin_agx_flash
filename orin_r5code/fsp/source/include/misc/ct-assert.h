/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef MISC__CT_ASSERT_H
#define MISC__CT_ASSERT_H

/*
 * Compile time assertion check
 *
 * If the asserted condition fails, it will result in a compilation error
 * due to the fact that a boolean condition that is false will be 0 which
 * results in a division by 0 (as seen by the compiler).
 */
#ifndef __ASSEMBLER__

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_20_10 \"Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx\") \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif
#define DO_CT_CONCAT(_x_, _y_)    _x_##_y_
#define CT_CONCAT(_a_, _b_)     DO_CT_CONCAT(_a_, _b_)
#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) MISRA_C_2012_Rule_20_10")
#endif

#ifdef USE_C11
#define CT_ASSERT(_cond_, _msg_)    _Static_assert(_cond_, _msg_);
#else
#define CT_ASSERT(_cond_, _msg_)                                                    \
    enum { CT_CONCAT(ASSERT_LINE_, __COUNTER__) = 1/(int)(!!(_cond_)) };
#endif // USE_C11

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) MISRA_C_2012_Rule_2_5")
#endif
#else

#define CT_ASSERT(_cond_, _msg_)

#endif // __ASSEMBLER__

#define HEADER_CHECK(x) CT_ASSERT(x, "Header file missing or invalid.")

#endif // MISC__CT_ASSERT_H
