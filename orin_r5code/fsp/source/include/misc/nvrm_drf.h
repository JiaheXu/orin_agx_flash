/*
 * Copyright 2007-2020 NVIDIA Corporation.  All Rights Reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.  Any
 * use, reproduction, disclosure or distribution of this software and related
 * documentation without an express license agreement from NVIDIA Corporation
 * is strictly prohibited.
 */

#ifndef MISC__NVRM_DRF_H
#define MISC__NVRM_DRF_H
#define FSP__MISC__NVRM_DRF_H                           1

/**
 *  @defgroup nvrm_drf RM DRF Macros
 *
 *  @ingroup nvddk_rm
 *
 * The following suite of macros are used for generating values to write into
 * hardware registers, or for extracting fields from read registers.  The
 * hardware headers have a RANGE define for each field in the register in the
 * form of x:y, 'x' being the high bit, 'y' the lower.  Through a clever use
 * of the C ternary operator, x:y may be passed into the macros below to
 * geneate masks, shift values, etc.
 *
 * There are two basic flavors of DRF macros, the first is used to define
 * a new register value from 0, the other is modifiying a field given a
 * register value.  An example of the first:
 *
 * reg = NV_DRF_DEF( HW, REGISTER0, FIELD0, VALUE0 )
 *     | NV_DRF_DEF( HW, REGISTER0, FIELD3, VALUE2 );
 *
 * To modify 'reg' from the previous example:
 *
 * reg = NV_FLD_SET_DRF_DEF( HW, REGISTER0, FIELD2, VALUE1, reg );
 *
 * To pass in numeric values instead of defined values from the header:
 *
 * reg = NV_DRF_NUM( HW, REGISTER3, FIELD2, 1024 );
 *
 * To read a value from a register:
 *
 * val = NV_DRF_VAL( HW, REGISTER3, FIELD2, reg );
 *
 * Some registers have non-zero reset values which may be extracted from the
 * hardware headers via NV_RESETVAL.
 */

/*
 * The NV_FIELD_* macros are helper macros for the public NV_DRF_* macros.
 */
#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_20_10 \"Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx\") \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif
#define NV_FIELD_LOWBIT(x)      (x##_LSB)
#define NV_FIELD_HIGHBIT(x)     (x##_MSB)
#define NV_FIELD_SIZE(x)        (NV_FIELD_HIGHBIT(x)-NV_FIELD_LOWBIT(x)+1)
#define NV_FIELD_SHIFT(x)       (x##_LSB %32U)
#define NV_FIELD_MASK(x)        (0xFFFFFFFFUL>>(31U-(x##_MSB %32U)+(x##_LSB %32U)))
#define NV_FIELD_BITS(val, x)   (((val) & NV_FIELD_MASK(x))<<NV_FIELD_SHIFT(x))
#define NV_FIELD_SHIFTMASK(x)   (NV_FIELD_MASK(x)<< (NV_FIELD_SHIFT(x)))

/*
 * Dynamic versions of some of the above macros
 */
#define NV_DYN_FIELD_SHIFT(lsb)                 (lsb %32U)
#define NV_DYN_FIELD_MASK(msb, lsb)             (0xFFFFFFFFUL>>(31U-(msb %32U)+(lsb %32U)))
#define NV_DYN_FIELD_BITS(val, msb, lsb)        (((val) & NV_DYN_FIELD_MASK(msb, lsb))<<NV_DYN_FIELD_SHIFT(lsb))


/*
 * The NV_FIELD64_* macros are helper macros for the public NV_DRF64_* macros.
 */
#define NV_FIELD64_SHIFT(x) (x##_LSB %64UL)
#define NV_FIELD64_MASK(x)  (0xFFFFFFFFFFFFFFFFULL>>(63UL-(x##_MSB %64UL)+(x##_LSB %64UL)))


/** NV_DRF_DEF - define a new register value.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param c defined value for the field
 */
#define NV_DRF_DEF(d,r,f,c) \
    ((d##_##r##_0_##f##_##c) << NV_FIELD_SHIFT(d##_##r##_0_##f))

/** NV_DRF_NUM - define a new register value.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param n numeric value for the field
 */
#define NV_DRF_NUM(d,r,f,n) \
    ((uint32_t)((n)& NV_FIELD_MASK(d##_##r##_0_##f)) << \
        NV_FIELD_SHIFT(d##_##r##_0_##f))

/** NV_DRF_VAL - read a field from a register.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param v register value
 */
#define NV_DRF_VAL(d,r,f,v) \
    (((v)>> NV_FIELD_SHIFT(d##_##r##_0_##f)) & \
        NV_FIELD_MASK(d##_##r##_0_##f))

/** NV_FLD_SET_DRF_NUM - modify a register field.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param n numeric field value
    @param v register value
 */
#define NV_FLD_SET_DRF_NUM(d,r,f,n,v) \
    (((v) & ~NV_FIELD_SHIFTMASK(d##_##r##_0_##f)) | NV_DRF_NUM(d,r,f,n))

/** NV_FLD_SET_DRF_DEF - modify a register field.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param c defined field value
    @param v register value
 */
#define NV_FLD_SET_DRF_DEF(d,r,f,c,v) \
    (((v) & ~NV_FIELD_SHIFTMASK(d##_##r##_0_##f)) | \
        NV_DRF_DEF(d,r,f,c))

/** NV_RESETVAL - get the reset value for a register.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
 */
#define NV_RESETVAL(d,r)    (d##_##r##_0_RESET_VAL)


/** NV_DRF64_NUM - define a new 64-bit register value.

    @ingroup nvrm_drf

    @param d register domain
    @param r register name
    @param f register field
    @param n numeric value for the field
 */
#define NV_DRF64_NUM(d,r,f,n) \
    (((n)& NV_FIELD64_MASK(d##_##r##_0_##f)) << \
        NV_FIELD64_SHIFT(d##_##r##_0_##f))

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_20_10 MISRA_C_2012_Rule_2_5")
#endif

#endif // FSP_MISC_NVRM_DRF_H
