/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef GPIO__TEGRA_GPIO_PRIV_H
#define GPIO__TEGRA_GPIO_PRIV_H
#define FSP__GPIO__TEGRA_GPIO_PRIV_H  1

/* Compiler headers */
#include <stdbool.h>                       // for bool
#include <stdint.h>                        // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <soc-common/hw-const.h>           // for MK_U32_CONSTANT

/* Hardware headers */

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @macro-title Number of GPIO pins per port/bank
 */
#define GPIOS_PER_BANK    MK_U32_CONST(8)

/**
 * @brief IRQ handler structure
 *
 * @handler Pointer to the interrupt service routine
 * @data    Data to be used by the interrupt handler.
 */
struct gpio_irq_handler {
    void (*handler)(void *hdata);
    void *data;
};

/**
 * @brief Tegra GPIO structure
 *
 * @base_addr         Base address of GPIO controller
 * @bank_count        Number of GPIO banks/ports supported by the GPIO chip
 * @bank_bases        Pointer to an array storing base address of the GPIO banks.
 * @irqs              Pointer to any array storing irq number of all the GPIO
 *                    sub-controllers(HW controllers) supported by this instance.
 * @nirqs             Number of irqs supported by this GPIO controller
 * @irq_handlers      Pointer to array storing IRQ handlers mapped one-to-one
 *                    with the gpio pins
 * @irq_status_offset Offset of IRQ status register relative to the
 *                    ENABLE_CONFIG register
 * @bank_irq_status   Pointer to the array storing IRQ enable status per GPIO
 *                    bank
 * @isr_status        Boolean variable indicating GPIO interrupt handling
 *                    mechanism. Set it to false, if the client does not need
 *                    GPIO interrupts or chooses to use its own interrupt
 *                    handling mechanism instead of registering to the common
 *                    GPIO IRQ handler.
 */
struct tegra_gpio_id {
    uint32_t                base_addr;
    uint32_t                bank_count;
    const uint32_t          *bank_bases;
    uint32_t                *irqs;
    uint32_t                nirqs;
    struct gpio_irq_handler *irq_handlers;
    uint32_t                irq_status_offset;
    uint8_t                 *bank_irq_status;
    bool                    isr_status;
};

#endif