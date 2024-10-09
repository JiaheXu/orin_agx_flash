/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef VIC__TEGRA_VIC_H
#define VIC__TEGRA_VIC_H
#define FSP__VIC__TEGRA_VIC_H                           1

/* Compiler headers */
#ifndef __ASSEMBLER__
#include <stdint.h>                                     // for uint32_t
#endif

/* Early FSP headers */
#include <misc/ct-assert.h>                             // for CT_ASSERT
#include <soc-common/hw-const.h>                        // Must appear before any hwinc files

/* Hardware headers */

/* Late FSP headers */
#include <misc/linker-symbols.h>                        // for IMAGE_SYM_BASE, IMAGE_SYM_LIMIT
#include <misc/macros.h>                                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <processor/irqs-hw.h>                          // for MAX_VIC_CONTROLLER
#ifndef __ASSEMBLER__
#include <error/common-errors.h>                        // for uint32_t
#endif

/* Module-specific FSP headers */
#include <cpu/arm-vic.h>                                // for ARM_VIC_IRQ_COUNT, FSP__ARM__COMMON...
#include <vic/vic-ld.h>                                 // for FSP__VIC__VIC_LD_H, Image$$VIC_ISR_...
#include <vic/sections-vic.h>                           // Immune from CT_ASSERT protection

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__LINKER_SYMBOLS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARM_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__VIC_LD_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief Tegra interrupt vector
 * @macro-title Tegra interrupt vector macro definitions
 * An interrupt vector is composed of:
 * Bits            Description
 * 31-7            Unused
 * 5               VIC number
 * 4-0             VIC IRQ
 *
 * NOTE: The following macros do not use the bit operations as they can be
 * used in assembler. HW constant macros of the type "MK_*X*_CONST" allow the
 * headers to be used by the assembler and the linker scripts.
 * MK_SHIFT_CONST - Define a constant indicating shift amount.
 * MK_U32_CONST   - Define a u32 constant
 * MK_MASK_CONT   - Define a constant indicating bit-mask
 *
 * @VIC_ID_SHIFT                Bitwise shift amount for VIC instance number in
 *                              the vector
 * @VIC_IRQ_MASK                Bit-mask for interrupt number in the VIC vector
 * @MAX_VIC_INTERRUPT_VECTOR    Maximum valid value of interrupt vector
 * @MAX_VIC_INSTANCE_ID         Maximum valid value of VIC instance ID
 * @TEGRA_INTERRUPT(_c_, _v_)   Returns tegra interrupt vector based on VIC
 *                              instance and IRQ numbers given as an input
 * @TEGRA_INTERRUPT_TO_VIC(_x_) Returns VIC instance number from the tegra
 *                              interrupt vector.
 * @TEGRA_INTERRUPT_TO_IRQ(_x_) Returns IRQ number for the VIC instance from
 *                              the tegra interrupt vector
 */
#define VIC_ID_SHIFT                    MK_SHIFT_CONST(5)
#define VIC_IRQ_MASK                    (ARM_VIC_IRQ_COUNT - MK_U32_CONST(1))
#define MAX_VIC_INTERRUPT_VECTOR        ((MK_MASK_CONST(0x1) << VIC_ID_SHIFT) \
                                         | VIC_IRQ_MASK)
#define MAX_VIC_INSTANCE_ID             ((uint32_t)(MAX_VIC_CONTROLLER) - 1U)
#define TEGRA_INTERRUPT(_c_, _v_)       (((((_c_)) & MK_MASK_CONST(0x1)) << VIC_ID_SHIFT) \
                                         | ((_v_) & VIC_IRQ_MASK))
#define TEGRA_INTERRUPT_TO_VIC(_x_)     (((_x_) >> VIC_ID_SHIFT) & MK_MASK_CONST(0x01))
#define TEGRA_INTERRUPT_TO_IRQ(_x_)     ((_x_) & VIC_IRQ_MASK)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")

#ifndef __ASSEMBLER__

/**
 * Converts (struct vic_isr_map_s *) to uint32_t
 *
 * @param[in] addr void pointer which will be coverted to uintptr_t
 * @param[out] value uintptr_t value of void pointer.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx")
static inline uint32_t vic_isr_ptr_to_uint32_t(struct vic_isr_map_s *addr)
{
    union type_cast_vic {
        struct vic_isr_map_s *addr;
        uint32_t addr_value;
    };
    union type_cast_vic t;

    t.addr = addr;

    return t.addr_value;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx")

/**
 * @brief Structure exposed to save interrupt context
 */
typedef struct arm_vic_context irq_context;

/**
 * @brief Returns number of ISR map entries created by the clients
 *
 * The approach below is not optimal or elegant, however it serves to avoid other
 * potential problems.  There were three viable approaches considered:
 * 1. File the RFD for MISRA 11.4 violation.
 * 2. Use 'ptrdiff_t' to subtract the pointers and convert them to integer.
 *    Unfortunately, this approach generates an MISRA 18.2 violation in addition
 *    to a CERTC INT31-C violation.  Neither of these two new violations would
 *    be covered under any existing RFD.
 * 3. Use the approach below.  The approach below fixes the MISRA 11.4 violation.
 *    However, it also generates a CERTC INT30-C violation.  The INT30-C violation
 *    already has an approved RFD and would be covered by the results of this
 *    approach.
 */
static inline uint32_t num_isr_map_entries(void)
{
    uint32_t num_isr;

    num_isr =  vic_isr_ptr_to_uint32_t(&IMAGE_SYM_LIMIT(VIC_ISR_MAP));
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    num_isr -= vic_isr_ptr_to_uint32_t(&IMAGE_SYM_BASE(VIC_ISR_MAP));

    return num_isr / sizeof(struct vic_isr_map_s);
}

void tegra_vic_init(void)                       SECTION_VIC_INIT_TEXT;

error_t tegra_safe_vic_init(void)               SECTION_VIC_INIT_TEXT;

void bad_irq(const irq_vect_t vector)           SECTION_VIC_ERROR_TEXT;

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
#endif
