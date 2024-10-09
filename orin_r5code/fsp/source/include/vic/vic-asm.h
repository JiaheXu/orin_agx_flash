/*
 * Copyright (c) 2018-2020 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef VIC__VIC_ASM_H
#define VIC__VIC_ASM_H
#define FSP__VIC__VIC_ASM_H                             1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <ospl/rtos-port-asm.h>

/* Module-specific FSP headers */
#include <vic/tegra-vic.h>

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
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSPL__RTOS_PORT_ASM_H, "Header file missing or invalid.")

#ifdef __ASSEMBLER__
/*
 * Macros used for generating code for interrupt handlers.
 */

/*
 * External references used by these macros
 */
.extern bad_irq
.type bad_irq, STT_FUNC

.extern vic0_isr_end
.type vic0_isr_end, STT_FUNC

.extern vic1_isr_end
.type vic1_isr_end, STT_FUNC

.extern vic_base_addr
.type vic_base_addr, STT_OBJECT

/**
 * @brief bad_irq_handler macro
 *
 * Defines the handler for an interrupt vector for which there is
 * no other handler.  Calls the function:
 *      void bad_irq(uint32_t vector)
 *
 * @param[in] label  The label to be applied to the code
 * @param[in] vector Tegra interrupt vector [0 - MAX_VIC_INTERRUPT_VECTOR]
 *
 */
.macro  bad_irq_handler label, vector
        .global         \label
        .balign         4
\label :
        rtosIRQ_ENTRY
        MOV     R0, #\vector
        BLX     bad_irq
        B       .               // not reached
.endm

/**
 * @brief Interrupt handler with address as input
 *
 * Interrupt handlers that take an address as a parameter to be passed
 * to a C function with the prototype:
 *     void func(void *addr)
 *
 * @param[in] label       Label to be applied to the code
 * @param[in] addr        Address to be passed to the function
 * @param[in] func        Function to be called
 * @param[in] vector      Tegra interrupt vector [0 - MAX_VIC_INTERRUPT_VECTOR]
 *
 * @func_req_id 9370049
 */
.macro  isr_addr         label, addr, func, vector
        .global         \label
        .balign         4
\label :
        rtosIRQ_ENTRY
        LDR     R0, =\addr
        BLX     \func
.if TEGRA_INTERRUPT_TO_VIC(\vector)
        B       vic1_isr_end
.else
        B       vic0_isr_end
.endif
.endm

/**
 * @brief Interrupt handler with value as an input to the function
 * Interrupt handlers that take a value as a parameter to be passed
 * to a C function with the prototype:
 *     void func(uint32_t value)
 *
 * @param[in] label  Label to be applied to the code
 * @param[in] value  16-bit constant value to be passed to the function
 * @param[in] func   Function to be called
 * @param[in] vector Tegra interrupt vector [0 - MAX_VIC_INTERRUPT_VECTOR]
 *
 * @func_req_id 9919361
 */
.macro  isr_value       label, value, func, vector
        .global         \label
        .balign         4
\label :
        rtosIRQ_ENTRY
        MOVW    R0, #\value
        MOVT    R0, #0
        BLX     \func
.if TEGRA_INTERRUPT_TO_VIC(\vector)
        B       vic1_isr_end
.else
        B       vic0_isr_end
.endif
.endm

/**
 * @brief ACK_VIC0_IRQ macro
 *
 * Acknowledges an interrupt originating from VIC0.
 *
 * Usage:
 *      ACK_VIC0_IRQ label
 *
 * Registers Modified:
 *      R0
 *
 * @param[in] label Label to be applied to the code
 */
.macro  ACK_VIC0_IRQ    label
        .global         \label
        .balign         4
\label :
        LDR     R0, =vic_base_addr
        LDR     R0, [R0]
        STR     R0, [R0, #APS_VIC_VICADDRESS_0]
.endm

/**
 * @brief ACK_VIC1_IRQ macro
 *
 * Acknowledges an interrupt originating from VIC1.
 *
 * Usage:
 *      ACK_VIC1_IRQ label
 *
 * Registers Modified:
 *      R0
 *      R1
 *
 * @param[in] label Label to be applied to the code
 */
.macro  ACK_VIC1_IRQ    label
        .global         \label
        .balign         4
\label :
        LDR     R0, =vic_base_addr
        LDR     R1, [R0, #4]
        STR     R0, [R1, #APS_VIC_VICADDRESS_0]
        LDR     R1, [R0]
        STR     R0, [R1, #APS_VIC_VICADDRESS_0]
.endm

/**
 * @brief isr_map_entry macro
 *
 * Creates an ISR map entry
 *
 * Usage:
 *      isr_map_entry vector, isr_fn
 *
 * @param[in] vector vector number that encodes both the VIC # and the IRQ #.
 *                    [0 - MAX_VIC_INTERRUPT_VECTOR]
 * @param[in] isr_fn assembler ISR function that will be dispatched when an
 *                   interrupt occurs
 *
 * @pre
 * - Interrupts set using static configuration
 *
 * @func_req_id 10897357
 */
.macro  isr_map_entry   vector, isr_fn
        .balign         8
        .long   \vector
        .long   \isr_fn
.endm

#endif // __ASSEMBLER__
#endif
