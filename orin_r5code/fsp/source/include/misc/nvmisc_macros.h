/*
 * Copyright (c) 1993-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.  Any
 * use, reproduction, disclosure or distribution of this software and related
 * documentation without an express license agreement from NVIDIA Corporation
 * is strictly prohibited.
 */

#ifndef MISC__NVMISC_MACROS_H
#define MISC__NVMISC_MACROS_H
#define FSP__MISC__NVMISC_MACROS_H                      1

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* These macros are taken from desktop driver branch file
 *  $P4PATH/sw/dev/gpu_drv/$BRANCH/sdk/nvidia/inc/nvmisc.h
 * This is to handle difference between DRF macros
 * between Mobile and Desktop
 */

// nvmisc_macros.h can't access nvmisc.h in the Mobile tree... but in some places both are used.
#if !defined(__NV_MISC_H) && !defined(NV_MISC_H)

#define DEVICE_BASE(d)          (0 ? d)  // what's up with this name? totally non-parallel to the macros below
#define DEVICE_EXTENT(d)        (1 ? d)  // what's up with this name? totally non-parallel to the macros below

#ifdef NV_USE_FIELD_RANGE_DEFS // These macros are added to remove MISRA C rule 14.3 violations
// Components which have defined HIGH_FIELD/LOW_FIELD can use below DRF macros
#define DRF_BASE(drf)           (drf##_LOW_FIELD)
#define DRF_EXTENT(drf)         (drf##_HIGH_FIELD)
#define DRF_SHIFT(drf)          ((drf##_LOW_FIELD) % 32U)
#define DRF_MASK(drf)           (0xFFFFFFFFU >> (31U - ((drf##_HIGH_FIELD) % 32U) + ((drf##_LOW_FIELD) % 32U)))

#else // For components which have not defined HIGH_FIELD/LOW_FIELD

// the first bit occupied by the field in a HW value (e.g., register)
#define DRF_BASE(drf)           (NV_FALSE ? drf)

// the last bit occupied by the field in a HW value
#define DRF_EXTENT(drf)         (NV_TRUE ? drf)

// the bit shift amount for accessing the field
#define DRF_SHIFT(drf)          (((NvU32)DRF_BASE(drf)) % 32U)

// bitmask for the value (unshifted)
#define DRF_MASK(drf)           (0xFFFFFFFFU >> (31U - (((NvU32)DRF_EXTENT(drf)) % 32U) + (((NvU32)DRF_BASE(drf)) % 32U)))
#endif

// bitmask for the value (shifted for HW value)
#define DRF_SHIFTMASK(drf)      (DRF_MASK(drf) << (DRF_SHIFT(drf)))

// size of the field (in bits)
#define DRF_SIZE(drf)           (DRF_EXTENT(drf) - DRF_BASE(drf) + 1U)

// constant value for HW, e.g., reg |= DRF_DEF(A06F, _GP_ENTRY1, _LEVEL, _SUBROUTINE);
#define DRF_DEF(d,r,f,c)        (((NvU32)(NV ## d ## r ## f ## c)) << (DRF_SHIFT(NV ## d ## r ## f)))

// numeric value for HW, e.g., reg |= DRF_NUM(A06F, _GP_ENTRY1, _LENGTH, numWords);
// Note: n should be unsigned 32-bit integer
#define DRF_NUM(d,r,f,n)        ((((NvU32)(n)) & (DRF_MASK(NV ## d ## r ## f))) << (DRF_SHIFT(NV ## d ## r ## f)))

// numeric value from HW value, e.g., unsigned int numWords = DRF_VAL(A06F, _GP_ENTRY1, _LENGTH, reg);
// Note: v should be unsigned 32-bit integer
#define DRF_VAL(d,r,f,v)        (((v) >> DRF_SHIFT(NV ## d ## r ## f)) & DRF_MASK(NV ## d ## r ## f))

#endif

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //FSP_MISC_NVMISC_MACROS_H
