/*
 * Copyright (c) 2015-2021 NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stdbool.h>               // for bool
#include <stdint.h>                // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>        // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <irq/irqs.h>              // irq_set_handler
#include <irq/safe-irqs.h>         // for irq_callback_fn, enter_critical
#include <misc/attributes.h>       // for UNUSED
#include <misc/macros.h>           // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <error/common-errors.h>   // for E_SUCCESS, error_t

/* Module-specific FSP headers */
#include <cpu/barriers.h>          // for barrier_compiler
#include <cpu/armv7-mpu.h>         // for FSP__CPU__ARMV7_MPU_H, R5_P...
#include <cpu/armv7-regs.h>        // for rd_cpsr, FSP__CPU__ARMV7_RE...

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__IRQ__IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__BARRIERS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_REGS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

static inline uint32_t arm_get_mode(void)
{
    uint32_t cpsr;

    cpsr = rd_cpsr();
    return cpsr & R5_PSR_MODE_MASK;
}

bool in_interrupt(void)
{
    uint32_t mode = arm_get_mode();
    return (mode == R5_PSR_MODE_FIQ) || (mode == R5_PSR_MODE_IRQ);
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
/* Weak dummy implementation for users with their own IRQ handling. */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
__attribute__((weak)) error_t irq_safe_set_handler(uint32_t irq, irq_callback_fn routine, void *data)
{
    UNUSED((irq));
    UNUSED((&routine));
    UNUSED((data));

    return E_SUCCESS;
}

/* Weak dummy implementation for users with their own IRQ handling. */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
__attribute__((weak)) void irq_set_handler(uint32_t irq, irq_callback_fn routine, void *data)
{
    UNUSED((irq));
    UNUSED((&routine));
    UNUSED((data));
}

/* Weak dummy implementation for users with their own IRQ handling. */
__attribute__((weak))
void irq_handler(uint32_t irq)
{
    (void) irq;
}

/* Weak dummy implementation for users with their own implementation. */
__attribute__((weak))
void exit_critical(void)
{
    barrier_compiler();
}

/* Weak dummy implementation for users with their own implementation. */
__attribute__((weak))
void enter_critical(void)
{
    barrier_compiler();
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
