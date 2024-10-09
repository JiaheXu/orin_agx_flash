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

/* Compiler headers */
#include <stdint.h>               // for uint32_t
#include <stddef.h>               // for NULL

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/arm-vic.h>          // for arm_vic_disable, arm_vic_enable
#include <debug/abort.h>          // for tegra_abort, FSP__DEBUG__ABORT_H
#include <irq/irqs.h>             // for irq_enable
#include <irq/safe-irqs.h>        // for irq_safe_disable, irq_safe_enable
#include <misc/bitops.h>          // for EXTRACT
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <misc/attributes.h>      // for WEAK
#include <error/common-errors.h>  // for error_t, E_SUCCESS, MODULE_ID_VIC...

/* Module-specific FSP headers */
#include <vic/tegra-vic.h>        // for TEGRA_INTERRUPT_TO_IRQ, TEGRA_INTER...
#include <vic/vic-abort-codes.h>  // for ABORT_BAD_IRQ, FSP__VIC__VIC_ABORT_...
#include <vic/sections-vic.h>     // Immune from CT_ASSERT protection
#include <vic/vic-ld.h>           // for vic_base_addr, irq_vect_t...
#include <vic/vic-errors.h>       // for E_VIC_INVALID_VECTOR_ID, ...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__CPU__ARM_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__VIC_ABORT_CODES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__VIC_LD_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__VIC_ERRORS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * irq_enable()   - enables the interrupt on a particular vector
 *
 * @vector:     interrupt vector number (VIC and IRQ are encoded in the
 *              vector number) to be enabled.
 *
 * This function will enable the vector to generate interrupts.  Until this
 * function is called, no interrupts from the indicated vector will be presented
 * to the CPU.
 *
 * Return Values:
 *      none
 */

SECTION_VIC_TEXT WEAK void
irq_enable(const uint32_t vector)
{
    uint32_t    vic;
    uint32_t    irq;

    if (vector > MAX_VIC_INTERRUPT_VECTOR) {
        goto out;
    }

    vic     = TEGRA_INTERRUPT_TO_VIC(vector);
    irq     = TEGRA_INTERRUPT_TO_IRQ(vector);

    arm_vic_enable(vic_base_addr[vic], irq);

out:
    return;
}

/**
 * irq_disable()  - disables the interrupt on a particular vector
 *
 * @vector:     interrupt vector number (VIC and IRQ are encoded in the
 *              vector number) to be disabled.
 *
 * This function will disable the vector to prevent it from generating
 * interrupts.  After this function has been called, no interrupts from the
 * specified vector will be presented to the CPU.
 *
 * Return Values:
 *      none
 */
SECTION_VIC_TEXT WEAK void
irq_disable(const uint32_t vector)
{
    uint32_t    vic;
    uint32_t    irq;

    if (vector > MAX_VIC_INTERRUPT_VECTOR) {
        goto out;
    }

    vic     = TEGRA_INTERRUPT_TO_VIC(vector);
    irq     = TEGRA_INTERRUPT_TO_IRQ(vector);

    arm_vic_disable(vic_base_addr[vic], irq);

out:
    return;
}

SECTION_VIC_TEXT void
save_irq_state(const uint32_t vic, irq_context *ctx)
{
    if ((vic > MAX_VIC_INSTANCE_ID) || (ctx == NULL)) {
        goto out;
    }

    arm_vic_save_state(vic_base_addr[vic], ctx);

out:
    return;
}

SECTION_VIC_TEXT void
restore_irq_state(const uint32_t vic, const irq_context *const ctx)
{
    if ((vic > MAX_VIC_INSTANCE_ID) || (ctx == NULL)) {
        goto out;
    }

    arm_vic_restore_state(vic_base_addr[vic], ctx);

out:
    return;
}

/**
 * irq_safe_enable()   - enables the interrupt on a particular vector
 *
 * @vector:     interrupt vector number (VIC and IRQ are encoded in the
 *              vector number) to be enabled.
 *
 * This function will enable the vector to generate interrupts.  Until this
 * function is called, no interrupts from the indicated vector will be presented
 * to the CPU.
 */
SECTION_VIC_TEXT WEAK error_t
irq_safe_enable(const uint32_t vector)
{
    uint32_t    vic;
    uint32_t    irq;
    error_t     err = E_SUCCESS;

    if (vector > MAX_VIC_INTERRUPT_VECTOR) {
        err = E_VIC_INVALID_VECTOR;
        goto out;
    }

    vic     = TEGRA_INTERRUPT_TO_VIC(vector);
    irq     = TEGRA_INTERRUPT_TO_IRQ(vector);

    arm_vic_enable(vic_base_addr[vic], irq);

out:
    return err;
}

/**
 * irq_safe_disable()  - disables the interrupt on a particular vector
 *
 * @vector:     interrupt vector number (VIC and IRQ are encoded in the
 *              vector number) to be disabled.
 *
 * This function will disable the vector to prevent it from generating
 * interrupts.  After this function has been called, no interrupts from the
 * specified vector will be presented to the CPU.
 */
SECTION_VIC_TEXT WEAK error_t
irq_safe_disable(const uint32_t vector)
{
    uint32_t    vic;
    uint32_t    irq;
    error_t     err = E_SUCCESS;

    if (vector > MAX_VIC_INTERRUPT_VECTOR) {
        err = E_VIC_INVALID_VECTOR;
        goto out;
    }

    vic     = TEGRA_INTERRUPT_TO_VIC(vector);
    irq     = TEGRA_INTERRUPT_TO_IRQ(vector);

    arm_vic_disable(vic_base_addr[vic], irq);

out:
    return err;
}

SECTION_VIC_TEXT error_t
irq_state_safe_save(const uint32_t vic, irq_context *ctx)
{
    error_t err = E_SUCCESS;

    if (vic > MAX_VIC_INSTANCE_ID) {
        err = E_VIC_INVALID_VIC_INSTANCE;
        goto out;
    }

    if (ctx == NULL) {
        err = E_VIC_NULL_INPUT_PARAMETER;
        goto out;
    }

    arm_vic_save_state(vic_base_addr[vic], ctx);

out:
    return err;
}

SECTION_VIC_TEXT error_t
irq_state_safe_restore(const uint32_t vic, const irq_context *const ctx)
{
    error_t err = E_SUCCESS;

    if (vic > MAX_VIC_INSTANCE_ID) {
        err = E_VIC_INVALID_VIC_INSTANCE;
        goto out;
    }

    if (ctx == NULL) {
        err = E_VIC_NULL_INPUT_PARAMETER;
        goto out;
    }

    arm_vic_restore_state(vic_base_addr[vic], ctx);

out:
    return err;
}

/**
 * bad_irq()            - ISR to be called for an unrecognized interrupt
 *
 * @vector:     interrupt vector number (VIC and IRQ are encoded in the
 *              vector number) that generated the unrecognized interrupt.
 *
 * This function is the interrupt handler that gets called for any interrupt
 * vector that does not otherwise have a registered interrupt handler.
 *
 * Return Values:
 *      none
 *
 * This function is not expected to return.
 */
SECTION_VIC_ERROR_TEXT void
bad_irq(const irq_vect_t vector)
{
    tegra_abort(ABORT_BAD_IRQ, EXTRACT(vector, 15U, 0U, uint32_t));
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
