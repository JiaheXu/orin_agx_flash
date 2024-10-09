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

#ifndef CPU__ARM_VIC_H
#define CPU__ARM_VIC_H
#define FSP__CPU__ARM_VIC_H                     1

#ifndef __ASSEMBLER__
/* Compiler headers */
#include <stdint.h>
#endif

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */

/* Hardware headers */

/* Late FSP headers */
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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define MAX_ARM_VICS        MK_U32_CONST(2)
#define ARM_VIC_IRQ_COUNT   MK_U32_CONST(32)
#define MAX_ARM_VIC_IRQS    (MAX_ARM_VICS * ARM_VIC_IRQ_COUNT)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#ifndef __ASSEMBLER__
struct arm_vic_context {
        uint32_t vect_addr[ARM_VIC_IRQ_COUNT];
        uint32_t intenable;
        uint32_t intselect;
};

INLINE_RFD(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx", MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
typedef void (*arm_vic_handler)(void);

void arm_vic_enable(uint32_t vic_base, uint32_t irq);
void arm_vic_disable(uint32_t vic_base, uint32_t irq);
void arm_vic_disable_all(uint32_t vic_base);
void arm_vic_gen_software_int(uint32_t vic_base, uint32_t irq);
void arm_vic_clear_software_int(uint32_t vic_base, uint32_t irq);
void arm_vic_set_isr_vect_addr(uint32_t vic_base, uint32_t irq, void (*isr_vect_addr)(void));
uint32_t arm_vic_read_irq_status(uint32_t vic_base);
uint32_t arm_vic_read_fiq_status(uint32_t vic_base);
uint32_t arm_vic_read_raw_int_status(uint32_t vic_base);
void arm_vic_write_intselect(uint32_t vic_base, uint32_t intselect);
uint32_t arm_vic_read_intselect(uint32_t vic_base);
void arm_vic_write_intenable(uint32_t vic_base, uint32_t intenable);
uint32_t arm_vic_read_intenable(uint32_t vic_base);
void arm_vic_save_state(uint32_t vic_base, struct arm_vic_context *ctx);
void arm_vic_restore_state(uint32_t vic_base, const struct arm_vic_context *ctx);
#endif

#endif
