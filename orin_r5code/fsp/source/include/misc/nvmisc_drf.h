/*
 * Copyright 2020-21 NVIDIA Corporation.  All Rights Reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.  Any
 * use, reproduction, disclosure or distribution of this software and related
 * documentation without an express license agreement from NVIDIA Corporation
 * is strictly prohibited.
 */

/*
 * NOTE: Intended for use with NVGPU-style manuals. Not garunteed to work
 * with other manual styles.
 */

#ifndef MISC__NVRM_DRF_H
#define MISC__NVRM_DRF_H
#define FSP__MISC__NVMISC_DRF_H                         1

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_20_10 \"Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx\") \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
//
// Bit Macros
//
///////////////////////////////////////////////////////////////////////////////
#define LOWESTBIT(x)                 ((x) &  (((x) - 1U) ^ (x)))

// Index of the 'on' bit (assuming that there is only one).
// Even if multiple bits are 'on', result is in range of 0-31.
//
// Use an inline function here. Using a macro causes
// issues with code coverage since we cannot always hit the 5
// conditionals in the macro implmentation.
static inline uint8_t BIT_IDX_32(uint32_t n)
{
   return ((((n) & 0xFFFF0000U) != 0U) ? 0x10U: 0U) |
            ((((n) & 0xFF00FF00U) != 0U) ? 0x08U: 0U) |
            ((((n) & 0xF0F0F0F0U) != 0U) ? 0x04U: 0U) |
            ((((n) & 0xCCCCCCCCU) != 0U) ? 0x02U: 0U) |
            ((((n) & 0xAAAAAAAAU) != 0U) ? 0x01U: 0U);
}


// Index of the 'on' bit (assuming that there is only one).
// Even if multiple bits are 'on', result is in range of 0-63.
static inline uint8_t BIT_IDX_64(uint64_t n)
{
    return ((((n) & 0xFFFFFFFF00000000ULL) != 0U) ? 0x20U: 0U) |
            ((((n) & 0xFFFF0000FFFF0000ULL) != 0U) ? 0x10U: 0U) |
            ((((n) & 0xFF00FF00FF00FF00ULL) != 0U) ? 0x08U: 0U) |
            ((((n) & 0xF0F0F0F0F0F0F0F0ULL) != 0U) ? 0x04U: 0U) |
            ((((n) & 0xCCCCCCCCCCCCCCCCULL) != 0U) ? 0x02U: 0U) |
            ((((n) & 0xAAAAAAAAAAAAAAAAULL) != 0U) ? 0x01U: 0U);
}

///////////////////////////////////////////////////////////////////////////////
//
// Alignment Macros
//
///////////////////////////////////////////////////////////////////////////////
#ifndef NV_ALIGN_DOWN
#define NV_ALIGN_DOWN(v, gran)                 ((v) & ~((gran) - 1U))
#endif

#ifndef NV_ALIGN_UP
#define NV_ALIGN_UP(v, gran)                   (((v) + ((gran) - 1U)) & ~((gran) - 1U))
#endif

#ifndef NV_ALIGN_DOWN64
#define NV_ALIGN_DOWN64(v, gran)               ((v) & ~(((uint64_t)gran) - 1U))
#endif

#ifndef NV_ALIGN_UP64
#define NV_ALIGN_UP64(v, gran)                 (((v) + ((gran) - 1U)) & ~(((uint64_t)gran) - 1U))
#endif

#ifndef NV_IS_ALIGNED
#define NV_IS_ALIGNED(v, gran)                 (0U == ((v) & ((gran) - 1U)))
#endif

#ifndef NV_IS_ALIGNED64
#define NV_IS_ALIGNED64(v, gran)               (0U == ((v) & (((uint64_t)gran) - 1U)))
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Register Field Access
//
///////////////////////////////////////////////////////////////////////////////

#define DRF_HIBIT(x)                           (x ## _XMSB)
#define DRF_LOBIT(x)                           (x ## _XLSB)

#define DRF_EXTENT(drf)                        (drf ## _XMSB)
#define DRF_BASE(drf)                          (drf ## _XLSB)

#define DEVICE_EXTENT(drf)                     (drf ## _XMSB)
#define DEVICE_BASE(drf)                       (drf ## _XLSB)

#define DRF_IDX_EXTENT(drf,i)                  (drf ## _XMSB(i))
#define DRF_IDX_BASE(drf,i)                    (drf ## _XLSB(i))

#define DRF_SIZE(drf)                          ((drf ## _XMSB) - (drf ## _XLSB) + 1U)

///////////////////////////////////////////////////////////////////////////////
//
// Register Field Shifting and Masking
//
///////////////////////////////////////////////////////////////////////////////
#define DRF_SHIFT(drf)                         ((drf ## _XLSB) % 32U)
#define DRF_SHIFT64(drf)                       ((drf ## _XLSB) % 64U)
#define DRF_IDX_SHIFT(drf,i)                   ((drf ## _XLSB(i)) % 32U)
#define DRF_IDX_SHIFT64(drf,i)                 ((drf ## _XLSB(i)) % 64U)

#define DRF_MASK(drf)                          (0xFFFFFFFFU >> (31U - ((drf ## _XMSB) % 32U) + ((drf ## _XLSB) % 32U)))
#define DRF_MASK64(drf)                        (0xFFFFFFFFFFFFFFFFUL >> (63UL - ((drf ## _XMSB) % 64UL) + ((drf ## _XLSB) % 64UL)))
#define DRF_IDX_MASK(drf,i)                    (0xFFFFFFFFU >> (31U - ((drf ## _XMSB(i)) % 32U) + ((drf ## _XLSB(i)) % 32U)))
#define DRF_IDX_MASK64(drf,i)                  (0xFFFFFFFFFFFFFFFFUL >> (63UL - ((drf ## _XMSB(i)) % 64UL) + ((drf ## _XLSB(i)) % 64UL)))

#define DRF_SHIFTMASK(drf)                     ((uint32_t)(0xFFFFFFFFU >> (31U - ((drf ## _XMSB) % 32U) + ((drf ## _XLSB) % 32U))) << ((drf ## _XLSB) % 32U))
#define DRF_SHIFTMASK64(drf)                   ((uint64_t)(0xFFFFFFFFFFFFFFFFUL >> (63UL - ((drf ## _XMSB) % 64UL) + ((drf ## _XLSB) % 64UL))) << ((drf ## _XLSB) % 64U))

#define DRF_IDX_SHIFTMASK(drf,i)               ((uint32_t)(0xFFFFFFFFU >> (31U - ((drf ## _XMSB(i)) % 32U) + ((drf ## _XLSB(i)) % 32U))) << ((drf ## _XLSB(i)) % 32U))
#define DRF_IDX_SHIFTMASK64(drf,i)             ((uint64_t)(0xFFFFFFFFFFFFFFFFUL >> (63UL - ((drf ## _XMSB(i)) % 64UL) + ((drf ## _XLSB(i)) % 64UL))) << ((drf ## _XLSB(i)) % 64U))


///////////////////////////////////////////////////////////////////////////////
//
// Register Field Reading and Definition
//
///////////////////////////////////////////////////////////////////////////////
#define DRF_DEF(d,r,f,c)                       (((uint32_t)(NV ## d ## r ## f ## c)) << DRF_SHIFT(NV ## d ## r ## f))
#define DRF_DEF64(d,r,f,c)                     (((uint64_t)(NV ## d ## r ## f ## c)) << DRF_SHIFT64(NV ## d ## r ## f))
#define DRF_IDX_DEF(d,r,f,i,c)                 (((uint32_t)(NV ## d ## r ## f ## c) << DRF_IDX_SHIFT(NV ## d ## r ## f,i))
#define DRF_IDX_DEF64(d,r,f,i,c)               (((uint64_t)(NV ## d ## r ## f ## c)) << DRF_IDX_SHIFT64(NV ## d ## r ## f,i))

#define DRF_NUM(d,r,f,n)                       ((((uint32_t)(n)) & DRF_MASK(NV ## d ## r ## f)) << DRF_SHIFT(NV ## d ## r ## f))
#define DRF_NUM64(d,r,f,n)                     ((((uint64_t)(n)) & DRF_MASK64(NV ## d ## r ## f)) << DRF_SHIFT64(NV ## d ## r ## f))
#define DRF_IDX_NUM(d,r,f,i,n)                 ((((uint32_t)(n)) & DRF_IDX_MASK(NV ## d ## r ## f,i)) << DRF_IDX_SHIFT(NV ## d ## r ## f,i))
#define DRF_IDX_NUM64(d,r,f,i,n)               ((((uint64_t)(n)) & DRF_IDX_MASK64(NV ## d ## r ## f,i)) << DRF_IDX_SHIFT64(NV ## d ## r ## f,i))

#define DRF_VAL(d,r,f,v)                       (((v) >> DRF_SHIFT(NV ## d ## r ## f)) & DRF_MASK(NV ## d ## r ## f))
#define DRF_VAL64(d,r,f,v)                     ((((uint64_t)(v)) >> DRF_SHIFT64(NV ## d ## r ## f)) & DRF_MASK64(NV ## d ## r ## f))
#define DRF_IDX_VAL(d,r,f,i,v)                 (((v) >> DRF_IDX_SHIFT(NV ## d ## r ## f,i)) & DRF_IDX_MASK(NV ## d ## r ## f,i))
#define DRF_IDX_VAL64(d,r,f,i,v)               ((((uint64_t)(v)) >> DRF_IDX_SHIFT64(NV ## d ## r ## f,i)) & DRF_IDX_MASK64(NV ## d ## r ## f,i))

///////////////////////////////////////////////////////////////////////////////
//
// Register Field Modification
//
///////////////////////////////////////////////////////////////////////////////
#define FLD_SET_DRF_NUM(d,r,f,n,v)             (((v) & ~DRF_SHIFTMASK(NV ## d ## r ## f)) | DRF_NUM(d,r,f,n))
#define FLD_SET_DRF_NUM64(d,r,f,n,v)           ((((uint64_t)(v)) & ~DRF_SHIFTMASK64(NV ## d ## r ## f)) | DRF_NUM64(d,r,f,n))
#define FLD_IDX_SET_DRF_NUM(d,r,f,i,n,v)       (((v) & ~DRF_IDX_SHIFTMASK(NV ## d ## r ## f,i)) | DRF_IDX_NUM(d,r,f,i,n))
#define FLD_IDX_SET_DRF_NUM64(d,r,f,i,n,v)     ((((uint64_t)(v)) & ~DRF_IDX_SHIFTMASK64(NV ## d ## r ## f,i)) | DRF_IDX_NUM64(d,r,f,i,n))

#define FLD_SET_DRF(d,r,f,c,v)                 (((v) & ~DRF_SHIFTMASK(NV ## d ## r ## f)) | DRF_DEF(d,r,f,c))
#define FLD_SET_DRF64(d,r,f,c,v)               ((((uint64_t)(v)) & ~DRF_SHIFTMASK64(NV ## d ## r ## f)) | DRF_DEF64(d,r,f,c))
#define FLD_IDX_SET_DRF(d,r,f,i,c,v)           (((v) & ~DRF_IDX_SHIFTMASK(NV ## d ## r ## f,i)) | DRF_IDX_DEF(d,r,f,i,c))
#define FLD_IDX_SET_DRF64(d,r,f,i,c,v)         ((((uint64_t)(v)) & ~DRF_IDX_SHIFTMASK64(NV ## d ## r ## f,i)) | DRF_IDX_DEF64(d,r,f,i,c))

#define RESETVAL(d,r)                          (NV ## d ## r ## _RESET_VAL)

///////////////////////////////////////////////////////////////////////////////
//
// Register Field Test
//
///////////////////////////////////////////////////////////////////////////////
#define FLD_TEST_DRF(d,r,f,c,v)                (DRF_VAL(d,r,f,(v)) == (NV ## d ## r ## f ## c))
#define FLD_TEST_DRF64(d,r,f,c,v)              (DRF_VAL64(d,r,f,(v)) == (NV ## d ## r ## f ## c))
#define FLD_IDX_TEST_DRF(d,r,f,i,c,v)          (DRF_IDX_VAL(d,r,f,i,(v)) == ((uint32_t)(NV ## d ## r ## f ## c)))
#define FLD_IDX_TEST_DRF64(d,r,f,i,c,v)        (DRF_IDX_VAL64(d,r,f,i,(v)) == ((uint64_t)(NV ## d ## r ## f ## c)))

#define FLD_TEST_DRF_NUM(d,r,f,n,v)            ((DRF_VAL(d,r,f,(v)) == (n)))
#define FLD_TEST_DRF_NUM64(d,r,f,n,v)          (DRF_VAL64(d,r,f,(v)) == (n))

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_20_10 MISRA_C_2012_Rule_2_5")
#endif

#endif
