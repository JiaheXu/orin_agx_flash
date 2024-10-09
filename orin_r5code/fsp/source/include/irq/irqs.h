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

#ifndef IRQ__IRQS_H
#define IRQ__IRQS_H
#define FSP__IRQ__IRQS_H                                1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>         // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <vic/tegra-vic.h>
#include <irq/safe-irqs.h>

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Enable an interrupt
 *
 * This function enables a device interrupt using IRQ vector number.
 * The IRQ vector number maps to the tegra interrupt number already known
 * to the client.
 *
 * Returns none. With static configuration of interrupts it is unlikely for
 * the client to send invalid value for "vector".
 * For an invalid value of "vector", the function returns back without
 * configuring any VIC registers.
 *
 * @pre
 * - Basic init of vectored interrupt controller has been done with ISRs
 * mapped to the respective valid interrupt sources.
 *
 * @param[in] vector IRQ number for the device interrupt
 *                   [0 - MAX_VIC_INTERRUPT_VECTOR]
 *
 *
 * @func_req_id 8668094
 */
void irq_enable(const uint32_t vector);

/**
 * @brief Disable an interrupt
 *
 * This function disables a device interrupt using IRQ vector number.
 * The IRQ vector number maps to tegra interrupt number already known
 * to the client
 *
 * Returns none. With static configuration of interrupts it is unlikely for
 * the client to send invalid value for "vector".
 * For an invalid value of "vector", the function returns back without
 * configuring any VIC registers
 *
 * @pre
 * - Basic init of vectored interrupt controller has been done with ISRs
 * mapped to the respective valid interrupt sources.
 *
 * @param[in] vector IRQ number for the device interrupt
 *                   [0 - MAX_VIC_INTERRUPT_VECTOR]
 *
 * @func_req_id 8668106
 */
void irq_disable(const uint32_t vector);

/**
 * @brief Set IRQ handler for a device interrupt
 *
 * This function registers an ISR for a device's IRQ number.
 *
 * Returns none
 *
 * @param[in] irq     SW IRQ number for the device's interrupt
 * @param[in] routine Function pointer to the ISR
 * @param[in] data    Pointer to the input arguments required by the ISR
 *
 * @pre
 * - Interrupts are set using dynamic configuration
 *
 * @func_req_id 8668238
 * @func_req_id 10242855
 */
void irq_set_handler(uint32_t irq, irq_callback_fn routine, void *data);

/**
 * @brief Save interrupt context
 *
 * This function saves the VIC interrupt context comprising of 32-bit
 * interrupt enable mask, 32-bit interrupt priority mask and interrupt handlers.
 * Returns none. With static configuration of interrupts it is unlikely for
 * the client to send invalid values for the input argument "vic".
 * For an invalid value of "vic" or a NULL irq_context pointer,
 * this function returns back without configuring/reading any VIC registers
 *
 * @param[in] vic VIC instance number [0 - MAX_VIC_INSTANCE_ID]
 * @param[in] ctx Pointer to the irq_context structure pre-declared by the user
 *                for saving the interrupt context of input VIC instance
 *
 * @func_req_id 10111856
 */
void save_irq_state(const uint32_t vic, irq_context *ctx);

/**
 * @brief Restore interrupt context
 *
 * This function restores the VIC interrupt context comprising of 32-bit
 * interrupt enable mask, 32-bit interrupt priority mask and interrupt handlers.
 * Returns none. With static configuration of interrupts it is unlikely for
 * the client to send invalid values for the input argument "vic".
 * For an invalid value of "vic" or a NULL irq_context pointer,
 * this function returns back without configuring/reading any VIC registers
 *
 * @param[in] vic VIC instance number [0 - MAX_VIC_INSTANCE_ID]
 * @param[in] ctx Pointer to the irq_context structure with interrupt context
 *                data for the input VIC instance saved by call to
 *                save_irq_state().
 *                Data stored in the input irq_context structure is written
 *                back to the VIC registers restoring the context.
 *
 * @func_req_id 10112363
 */
void restore_irq_state(const uint32_t vic, const irq_context *const ctx);

#endif
