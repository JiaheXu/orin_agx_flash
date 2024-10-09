/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__ARMV7_MPU_H
#define CPU__ARMV7_MPU_H
#define FSP__CPU__ARMV7_MPU_H                    1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file armv7-mpu.h
 * @brief Defines for the register layout of the MPU
 */

/**
 * @brief R5 Processor Modes
 *
 * @macro-title R5 Processor Modes
 */
#define R5_PSR_MODE_USER        0x10UL  // user mode
#define R5_PSR_MODE_FIQ         0x11UL  // processing an FIQ
#define R5_PSR_MODE_IRQ         0x12UL  // processing an IRQ
#define R5_PSR_MODE_SUPER       0x13UL  // supervisor mode
#define R5_PSR_MODE_ABORT       0x17UL  // processing an abort
#define R5_PSR_MODE_UNDEF       0x1BUL  // undefined exception
#define R5_PSR_MODE_SYSTEM      0x1FUL  // system mode

#define R5_PSR_MODE_MASK        0x1FUL  // mask for extracting processor mode

/**
 * @brief CPSR register bit-masks
 *
 * @macro-title CPSR bit fields
 */
#define R5_CPSR_IRQ_SHIFT 7U
#define R5_CPSR_IRQ_MASK  (1U << R5_CPSR_IRQ_SHIFT)

#define R5_CPSR_FIQ_SHIFT 6U
#define R5_CPSR_FIQ_MASK  (1U << R5_CPSR_FIQ_SHIFT)

/**
 * @brief MPU Region Memory Attributes
 *
 * MPU Region Memory Attributes
 *
 * See section 4.3.21 of the ARM Cortex R5 Technical Reference Manual for
 * additional descriptions of these bits and what they do.
 *
 * The following bits define how the R5 will access a region of memory that
 * has been described by an individual MPU region.
 *
 * These bits are used in the setting of an MPU region's access control
 * register.
 */
#define R5_DRACR_TEX_SHIFT      3
#define R5_DRACR_S_SHIFT        2
#define R5_DRACR_C_SHIFT        1
#define R5_DRACR_B_SHIFT        0

/**
 * @brief Strongly ordered memory
 */
#define R5_DRACR_STRONGLY_ORDERED               (0UL << R5_DRACR_TEX_SHIFT)

/**
 * @brief Shareable device memory
 */
#define R5_DRACR_SHAREABLE_DEV                  ((0UL << R5_DRACR_TEX_SHIFT) | \
                                                BIT(R5_DRACR_B_SHIFT))

/**
 * @brief Device memory
 */
#define R5_DRACR_DEV                            (2UL << R5_DRACR_TEX_SHIFT)

/**
 * @brief Uncached Shareable memory
 */
#define R5_DRACR_NORMAL_UNCACHED_SHAREABLE      (1UL << R5_DRACR_TEX_SHIFT) | \
                                                BIT(R5_DRACR_S_SHIFT)

/**
 * @brief Uncached memory
 */
#define R5_DRACR_NORMAL_UNCACHED                (1UL << R5_DRACR_TEX_SHIFT)

/**
 * @brief Cached shareable memory
 */
#define R5_DRACR_NORMAL_CACHED_SHAREABLE        ((1UL << R5_DRACR_TEX_SHIFT) | \
                                                BIT(R5_DRACR_S_SHIFT) | \
                                                BIT(R5_DRACR_C_SHIFT) | \
                                                BIT(R5_DRACR_B_SHIFT))

/**
 * @brief Cached memory
 */
#define R5_DRACR_NORMAL_CACHED                  ((1UL << R5_DRACR_TEX_SHIFT) | \
                                                BIT(R5_DRACR_C_SHIFT) | \
                                                BIT(R5_DRACR_B_SHIFT))

/**
 * @brief MPU Region Memory Access
 *
 * See section 4.3.21 of the ARM Cortex R5 Technical Reference Manual for
 * additional descriptions of these bits and what they do.
 *
 * The following define the access permissions for a region of memory that
 * has been described by an individual MPU region.
 *
 * These bits are used in the setting of an MPU region's access control
 * register.
 */
/**
 * @brief No exectute
 */
#define R5_DRACR_XN             (1UL << 12)

#define R5_DRACR_AP_SHIFT       8
#define R5_DRACR_AP_MASK        (7UL << R5_DRACR_AP_SHIFT)

/**
 * @brief No access allowed
 */
#define R5_DRACR_AP_NONE        (0UL << R5_DRACR_AP_SHIFT)

/**
 * @brief Privileged Read/Write, No User access
 */
#define R5_DRACR_AP_PRIV_RW     (1UL << R5_DRACR_AP_SHIFT)

/**
 * @brief User Read Only, Privileged Read/Write
 */
#define R5_DRACR_AP_USER_RO     (2UL << R5_DRACR_AP_SHIFT)

/**
 * @brief User Read/Write, Privileged Read/Write
 */
#define R5_DRACR_AP_USER_RW     (3UL << R5_DRACR_AP_SHIFT)

/**
 * @brief Privileged Read Only, No User access
 */
#define R5_DRACR_AP_PRIV_RO     (5UL << R5_DRACR_AP_SHIFT)

/**
 * @brief Privileged Read Only, User Read Only
 */
#define R5_DRACR_AP_ALL_RO      (6UL << R5_DRACR_AP_SHIFT)

/*
 * When debugging with DS5 and DStream, it is necessary to
 * have the code reagions be writable since the debugger
 * needs to insert breakpoint instructions into the code.
 */
#if defined(DS5_DEBUG_ENABLE) && (DS5_DEBUG_ENABLE == 1)
#define R5_DRACR_AP_USER_CODE   R5_DRACR_AP_USER_RW
#define R5_DRACR_AP_PRIV_CODE   R5_DRACR_AP_PRIV_RW
#else
/**
 * @brief User Code Access - synonym for User Read Only
 */
#define R5_DRACR_AP_USER_CODE   R5_DRACR_AP_USER_RO

/**
 * @brief Privileged Code Access - synonym for Privileged Read Only
 */
#define R5_DRACR_AP_PRIV_CODE   R5_DRACR_AP_PRIV_RO
#endif

/*
 * Some synonyms for memory access
 */
#define R5_DRACR_AP_RW          R5_DRACR_AP_USER_RW
#define R5_DRACR_AP_RO          R5_DRACR_AP_USER_RO

#define R5_MPUIR_0_DREGION_RANGE        15:8
#define R5_MPUIR_0_DREGION_MSB          MK_SHIFT_CONST(15)
#define R5_MPUIR_0_DREGION_LSB          MK_SHIFT_CONST(8)

#define R5_DRSR_0_RSIZE_RANGE           5:1
#define R5_DRSR_0_RSIZE_MSB             MK_SHIFT_CONST(5)
#define R5_DRSR_0_RSIZE_LSB             MK_SHIFT_CONST(1)

#define R5_DRSR_0_EN_RANGE              0:0
#define R5_DRSR_0_EN_MSB                MK_SHIFT_CONST(0)
#define R5_DRSR_0_EN_LSB                MK_SHIFT_CONST(0)

#define R5_SCTLR_0_BR_RANGE             17:17
#define R5_SCTLR_0_BR_MSB               MK_SHIFT_CONST(17)
#define R5_SCTLR_0_BR_LSB               MK_SHIFT_CONST(17)

#define R5_SCTLR_0_M_RANGE              0:0
#define R5_SCTLR_0_M_MSB                MK_SHIFT_CONST(0)
#define R5_SCTLR_0_M_LSB                MK_SHIFT_CONST(0)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
