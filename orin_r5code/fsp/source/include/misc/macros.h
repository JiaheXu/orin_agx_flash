/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef MISC__MACROS_H
#define MISC__MACROS_H
#define FSP__MISC__MACROS_H                             1

#ifndef __ASSEMBLER__
/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

#endif

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
        (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\") \
        (deviate MISRA_C_2012_Rule_20_10 \"Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx\") \
        (deviate CERT_DCL37_C  \"Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx\")")
#endif
/*
 * Create wrappers for Coverity MISRA/CERTC _Pragma
 *   inline block deviation records.  Feature is enabled in makefile with
 *   FSP_COVERITY=1, or by passing FSP_COVERITY=1 on the command line to
 *   the makefile.
 */
#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
  #define MISRA_Pragma(_p_) _Pragma(_p_)
  #define TO_MISRA_Pragma(_a_) MISRA_Pragma(STR(_a_))
  #define MISRA_START_DEVIATE(_r_, _d_) (CONCAT(deviate MISRA_C_2012_,_r_ _d_))
  #define CERTC_START_DEVIATE(_r_, _d_) (CONCAT(deviate CERT_,_r_ _d_))
  #define MISRA_START_FP(_r_, _d_) (CONCAT(false_positive MISRA_C_2012_,_r_ _d_))
  #define CERTC_START_FP(_r_, _d_) (CONCAT(false_positive CERT_,_r_ _d_))
  #define MISRA_END_DEVIATE(_r_, _d_) CONCAT(MISRA_C_2012_,_r_ _d_)
  #define CERTC_END_DEVIATE(_r_, _d_) CONCAT(CERT_,_r_ _d_)

  #define INLINE_RFD_SELECTOR(_c1_, _t1_, _r1_, _d1_, \
                              _c2_, _t2_, _r2_, _d2_, \
                              _c3_, _t3_, _r3_, _d3_, \
                              _c4_, _t4_, _r4_, _d4_, \
                              _c5_, _t5_, _r5_, _d5_, \
                              _c6_, _t6_, _r6_, _d6_, \
                              _c7_, _t7_, _r7_, _d7_, \
                              _c8_, _t8_, _r8_, _d8_, \
                              FUNC, ...) FUNC
  #define INLINE_RFD_CHOOSER(...) \
            INLINE_RFD_SELECTOR(__VA_ARGS__, INLINE_RFDx8,,,, \
                                             INLINE_RFDx7,,,, \
                                             INLINE_RFDx6,,,, \
                                             INLINE_RFDx5,,,, \
                                             INLINE_RFDx4,,,, \
                                             INLINE_RFDx3,,,, \
                                             INLINE_RFDx2,,,, \
                                             INLINE_RFDx1, )
  #define START_RFD_BLOCK_CHOOSER(...) \
            INLINE_RFD_SELECTOR(__VA_ARGS__, START_RFD_BLOCKx8,,,, \
                                             START_RFD_BLOCKx7,,,, \
                                             START_RFD_BLOCKx6,,,, \
                                             START_RFD_BLOCKx5,,,, \
                                             START_RFD_BLOCKx4,,,, \
                                             START_RFD_BLOCKx3,,,, \
                                             START_RFD_BLOCKx2,,,, \
                                             START_RFD_BLOCKx1, )
  #define END_RFD_BLOCK_CHOOSER(...) \
            INLINE_RFD_SELECTOR(__VA_ARGS__, END_RFD_BLOCKx8,,,, \
                                             END_RFD_BLOCKx7,,,, \
                                             END_RFD_BLOCKx6,,,, \
                                             END_RFD_BLOCKx5,,,, \
                                             END_RFD_BLOCKx4,,,, \
                                             END_RFD_BLOCKx3,,,, \
                                             END_RFD_BLOCKx2,,,, \
                                             END_RFD_BLOCKx1, )
  #define INLINE_RFD(...) INLINE_RFD_CHOOSER(__VA_ARGS__)(__VA_ARGS__)
  #define START_RFD_BLOCK(...) START_RFD_BLOCK_CHOOSER(__VA_ARGS__)(__VA_ARGS__)
  #define END_RFD_BLOCK(...) END_RFD_BLOCK_CHOOSER(__VA_ARGS__)(__VA_ARGS__)


// Input:
//   _c_:  Class:  MISRA, CERTC
//   _t_:  Types:  DEVIATE, FP
//   _r_:  Rule:   (e.g.) Rule_1_2, Directive_4_9, etc.
//   _d_:  Description:  Text description including BUG number and RFD document name
  #define INLINE_RFDx1(_c1_, _t1_, _r1_, _d1_) \
                       TO_MISRA_Pragma(coverity compliance \
                                                           _c1_##_START_##_t1_(_r1_,_d1_))
  #define INLINE_RFDx2(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_))
  #define INLINE_RFDx3(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                       _c3_, _t3_, _r3_, _d3_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_))
  #define INLINE_RFDx4(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                       _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_))
  #define INLINE_RFDx5(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                       _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                       _c5_, _t5_, _r5_, _d5_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_))
  #define INLINE_RFDx6(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                       _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                       _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_) \
                                                           _c6_##_START_##_t6_(_r6_,_d6_))
  #define INLINE_RFDx7(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                       _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                       _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_, \
                       _c7_, _t7_, _r7_, _d7_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_) \
                                                           _c6_##_START_##_t6_(_r6_,_d6_) \
                                                           _c7_##_START_##_t7_(_r7_,_d7_))
  #define INLINE_RFDx8(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                       _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                       _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_, \
                       _c7_, _t7_, _r7_, _d7_, _c8_, _t8_, _r8_, _d8_) \
                       TO_MISRA_Pragma(coverity compliance _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_) \
                                                           _c6_##_START_##_t6_(_r6_,_d6_) \
                                                           _c7_##_START_##_t7_(_r7_,_d7_) \
                                                           _c8_##_START_##_t8_(_r8_,_d8_))
  #define START_RFD_BLOCKx1(_c1_, _t1_, _r1_, _d1_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_))
  #define START_RFD_BLOCKx2(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_))
  #define START_RFD_BLOCKx3(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                            _c3_, _t3_, _r3_, _d3_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_))
  #define START_RFD_BLOCKx4(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                            _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_))
  #define START_RFD_BLOCKx5(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                            _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                            _c5_, _t5_, _r5_, _d5_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_))
  #define START_RFD_BLOCKx6(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                            _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                            _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_) \
                                                           _c6_##_START_##_t6_(_r6_,_d6_))
  #define START_RFD_BLOCKx7(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                            _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                            _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_, \
                            _c7_, _t7_, _r7_, _d7_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_) \
                                                           _c6_##_START_##_t6_(_r6_,_d6_) \
                                                           _c7_##_START_##_t7_(_r7_,_d7_))
  #define START_RFD_BLOCKx8(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                            _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                            _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_, \
                            _c7_, _t7_, _r7_, _d7_, _c8_, _t8_, _r8_, _d8_) \
                       TO_MISRA_Pragma(coverity compliance block(include) \
                                                           _c1_##_START_##_t1_(_r1_,_d1_) \
                                                           _c2_##_START_##_t2_(_r2_,_d2_) \
                                                           _c3_##_START_##_t3_(_r3_,_d3_) \
                                                           _c4_##_START_##_t4_(_r4_,_d4_) \
                                                           _c5_##_START_##_t5_(_r5_,_d5_) \
                                                           _c6_##_START_##_t6_(_r6_,_d6_) \
                                                           _c7_##_START_##_t7_(_r7_,_d7_) \
                                                           _c8_##_START_##_t8_(_r8_,_d8_))
  #define END_RFD_BLOCKx1(_c1_, _t1_, _r1_, _d1_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_))
  #define END_RFD_BLOCKx2(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_))
  #define END_RFD_BLOCKx3(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                          _c3_, _t3_, _r3_, _d3_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_) \
                                                           _c3_##_END_##_t3_(_r3_,_d3_))
  #define END_RFD_BLOCKx4(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                          _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_) \
                                                           _c3_##_END_##_t3_(_r3_,_d3_) \
                                                           _c4_##_END_##_t4_(_r4_,_d4_))
  #define END_RFD_BLOCKx5(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                          _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                          _c5_, _t5_, _r5_, _d5_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_) \
                                                           _c3_##_END_##_t3_(_r3_,_d3_) \
                                                           _c4_##_END_##_t4_(_r4_,_d4_) \
                                                           _c5_##_END_##_t5_(_r5_,_d5_))
  #define END_RFD_BLOCKx6(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                          _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                          _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_) \
                                                           _c3_##_END_##_t3_(_r3_,_d3_) \
                                                           _c4_##_END_##_t4_(_r4_,_d4_) \
                                                           _c5_##_END_##_t5_(_r5_,_d5_) \
                                                           _c6_##_END_##_t6_(_r6_,_d6_))
  #define END_RFD_BLOCKx7(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                          _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                          _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_, \
                          _c7_, _t7_, _r7_, _d7_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_) \
                                                           _c3_##_END_##_t3_(_r3_,_d3_) \
                                                           _c4_##_END_##_t4_(_r4_,_d4_) \
                                                           _c5_##_END_##_t5_(_r5_,_d5_) \
                                                           _c6_##_END_##_t6_(_r6_,_d6_) \
                                                           _c7_##_END_##_t7_(_r7_,_d7_))
  #define END_RFD_BLOCKx8(_c1_, _t1_, _r1_, _d1_, _c2_, _t2_, _r2_, _d2_, \
                          _c3_, _t3_, _r3_, _d3_, _c4_, _t4_, _r4_, _d4_, \
                          _c5_, _t5_, _r5_, _d5_, _c6_, _t6_, _r6_, _d6_, \
                          _c7_, _t7_, _r7_, _d7_, _c8_, _t8_, _r8_, _d8_) \
                       TO_MISRA_Pragma(coverity compliance end_block(include) \
                                                           _c1_##_END_##_t1_(_r1_,_d1_) \
                                                           _c2_##_END_##_t2_(_r2_,_d2_) \
                                                           _c3_##_END_##_t3_(_r3_,_d3_) \
                                                           _c4_##_END_##_t4_(_r4_,_d4_) \
                                                           _c5_##_END_##_t5_(_r5_,_d5_) \
                                                           _c6_##_END_##_t6_(_r6_,_d6_) \
                                                           _c7_##_END_##_t7_(_r7_,_d7_) \
                                                           _c8_##_END_##_t8_(_r8_,_d8_))
#else
  #define INLINE_RFD(...)
  #define START_RFD_BLOCK(...)
  #define END_RFD_BLOCK(...)
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAY_SSIZE(x) (int)(sizeof(x) / sizeof((x)[0]))

#define SIZE_K(KB) (1024U * (KB))
#define SIZE_M(MB) SIZE_K(1024U * (MB))

#define DIV_ROUND_UP(x, y) (((x) + (y) - 1U) / (y))

#define min(a, b) (((a) < (b)) ? (a) : (b))

#define max(a, b) (((a) > (b)) ? (a) : (b))

#define CONTAINER_OF(_ptr, _type, _field) \
        (_type *)((char *)_ptr - offsetof(_type, _field))

#ifndef U64_C
#define U64_C(x) UINT64_C(x)
#endif

#ifndef U32_C
#define U32_C(x) UINT32_C(x)
#endif

#ifndef U16_C
#define U16_C(x) UINT16_C(x)
#endif

#ifndef U8_C
#define U8_C(x) UINT8_C(x)
#endif

/* String concatenation */
#define TO_STR(_s_)               #_s_
#define STR(_string_)           TO_STR(_string_)

#define DO_CONCAT(_x_, _y_)       _x_##_y_
#define CONCAT(_a_, _b_)        DO_CONCAT(_a_, _b_)

/* compiler specific section name for modules */
#if defined(USE_GCC_SECTIONS) && (USE_GCC_SECTIONS == 1)
#define SECTION_BUILDER(_name_, _type_) ._name_._type_
#else
#define SECTION_BUILDER(_name_, _type_) ._type_._name_
#endif

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_2_5 MISRA_C_2012_Rule_20_10 CERT_DCL37_C")
#endif

#endif
