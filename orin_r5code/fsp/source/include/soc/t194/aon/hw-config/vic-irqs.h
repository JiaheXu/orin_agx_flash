/* Copyright (c) 2018-2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef HW_CONFIG__VIC_IRQS_H
#define HW_CONFIG__VIC_IRQS_H
#define FSP__HW_CONFIG__VIC_IRQS_H                      1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, ...

/* Module-specific FSP headers */
#include <processor/irqs-hw.h>
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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define NUM_VICS                        2

/*
 * Define the various vectors where the VIC and IRQ are encoded into one value
 */
#define INTERRUPT_WDTFIQ                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_WDTFIQ)
#define INTERRUPT_WDTIRQ                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_WDTIRQ)
#define INTERRUPT_TIMER0                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_TIMER0)
#define INTERRUPT_TIMER1                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_TIMER1)
#define INTERRUPT_TIMER2                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_TIMER2)
#define INTERRUPT_TIMER3                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_TIMER3)
#define INTERRUPT_MBOX                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_MBOX)
#define INTERRUPT_GTE                   TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_GTE)
#define INTERRUPT_PMU                   TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_PMU)
#define INTERRUPT_DMA0                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA0)
#define INTERRUPT_DMA1                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA1)
#define INTERRUPT_DMA2                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA2)
#define INTERRUPT_DMA3                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA3)
#define INTERRUPT_DMA4                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA4)
#define INTERRUPT_DMA5                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA5)
#define INTERRUPT_DMA6                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA6)
#define INTERRUPT_DMA7                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMA7)
#define INTERRUPT_I2C1                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_I2C1)
#define INTERRUPT_I2C2                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_I2C2)
#define INTERRUPT_I2C3                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_I2C3)
#define INTERRUPT_SPI                   TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_SPI)
#define INTERRUPT_DMIC                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_DMIC)
#define INTERRUPT_UART_1                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_UART_1)
#define INTERRUPT_UART_2                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_UART_2)
#define INTERRUPT_CAN1_0                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_CAN1_0)
#define INTERRUPT_CAN1_1                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_CAN1_1)
#define INTERRUPT_CAN2_0                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_CAN2_0)
#define INTERRUPT_CAN2_1                TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_CAN2_1)
#define INTERRUPT_LIC0                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_LIC0)
#define INTERRUPT_LIC1                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_LIC1)
#define INTERRUPT_LIC2                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_LIC2)
#define INTERRUPT_LIC3                  TEGRA_INTERRUPT(0, NV_AON_INTERRUPT_LIC3)

// VIC1 interrupts
#define INTERRUPT_NOC_ERR               TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_NOC_ERR)
#define INTERRUPT_GPIO                  TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_GPIO)
#define INTERRUPT_WAKE0                 TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_WAKE0)
#define INTERRUPT_PMC                   TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_PMC)
#define INTERRUPT_V1RSVD4               TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD4)
#define INTERRUPT_PM                    TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_PM)
#define INTERRUPT_FPUINT                TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_FPUINT)
#define INTERRUPT_AOVC                  TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_AOVC)
#define INTERRUPT_ACTMON                TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_ACTMON)
#define INTERRUPT_AOWDT                 TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_AOWDT)
#define INTERRUPT_TOP0_HSP_DB           TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_TOP0_HSP_DB)
#define INTERRUPT_CTIIRQ                TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_CTIIRQ)
#define INTERRUPT_NOC_SEC               TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_NOC_SEC)
#define INTERRUPT_CAR                   TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_CAR)
#define INTERRUPT_UART6                 TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_UART6)
#define INTERRUPT_UART8                 TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_UART8)
#define INTERRUPT_GPIO_3                TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_GPIO_3)
#define INTERRUPT_V1RSVD17              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD17)
#define INTERRUPT_V1RSVD18              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD18)
#define INTERRUPT_V1RSVD19              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD19)
#define INTERRUPT_V1RSVD20              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD20)
#define INTERRUPT_V1RSVD21              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD21)
#define INTERRUPT_V1RSVD22              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD22)
#define INTERRUPT_V1RSVD23              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD23)
#define INTERRUPT_V1RSVD24              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD24)
#define INTERRUPT_V1RSVD25              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD25)
#define INTERRUPT_V1RSVD26              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD26)
#define INTERRUPT_V1RSVD27              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD27)
#define INTERRUPT_V1RSVD28              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD28)
#define INTERRUPT_V1RSVD29              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD29)
#define INTERRUPT_V1RSVD30              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD30)
#define INTERRUPT_V1RSVD31              TEGRA_INTERRUPT(1, NV_AON_INTERRUPT_V1RSVD31)

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
