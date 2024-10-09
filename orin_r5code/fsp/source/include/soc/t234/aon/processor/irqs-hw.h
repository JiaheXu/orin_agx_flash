/*
* Copyright (c) 2020 NVIDIA CORPORATION.  All rights reserved.
*
* NVIDIA CORPORATION and its licensors retain all intellectual property
* and proprietary rights in and to this software, related documentation
* and any modifications thereto.  Any use, reproduction, disclosure or
* distribution of this software and related documentation without an express
* license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#ifndef PROCESSOR__IRQS_HW_H
#define PROCESSOR__IRQS_HW_H
#define FSP__PROCESSOR__IRQS_HW_H                       1

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#define MAX_VIC_CONTROLLER       2

#define NV_AON_INTERRUPT_VIC0_BASE	0
#define NV_AON_INTERRUPT_VIC1_BASE	32

// VIC0 interrupts
#define NV_AON_INTERRUPT_WDTFIQ   0
#define NV_AON_INTERRUPT_WDTIRQ   1
#define NV_AON_INTERRUPT_TIMER0   2
#define NV_AON_INTERRUPT_TIMER1   3
#define NV_AON_INTERRUPT_TIMER2   4
#define NV_AON_INTERRUPT_TIMER3   5
#define NV_AON_INTERRUPT_MBOX     6
#define NV_AON_INTERRUPT_GTE      7
#define NV_AON_INTERRUPT_PMU      8
#define NV_AON_INTERRUPT_DMA0     9
#define NV_AON_INTERRUPT_DMA1     10
#define NV_AON_INTERRUPT_DMA2     11
#define NV_AON_INTERRUPT_DMA3     12
#define NV_AON_INTERRUPT_DMA4     13
#define NV_AON_INTERRUPT_DMA5     14
#define NV_AON_INTERRUPT_DMA6     15
#define NV_AON_INTERRUPT_DMA7     16
#define NV_AON_INTERRUPT_V0RSVD17 17
#define NV_AON_INTERRUPT_I2C2     18
#define NV_AON_INTERRUPT_I2C3     19
#define NV_AON_INTERRUPT_SPI      20
#define NV_AON_INTERRUPT_DMIC     21
#define NV_AON_INTERRUPT_UART_1   22
#define NV_AON_INTERRUPT_UART_J   23
#define NV_AON_INTERRUPT_CAN1_0   24
#define NV_AON_INTERRUPT_CAN1_1   25
#define NV_AON_INTERRUPT_CAN2_0   26
#define NV_AON_INTERRUPT_CAN2_1   27
#define NV_AON_INTERRUPT_LIC0     28
#define NV_AON_INTERRUPT_LIC1     29
#define NV_AON_INTERRUPT_LIC2     30
#define NV_AON_INTERRUPT_LIC3     31

// VIC1 interrupts
#define NV_AON_INTERRUPT_NOC_ERR   0
#define NV_AON_INTERRUPT_GPIO      1
#define NV_AON_INTERRUPT_WAKE0     2
#define NV_AON_INTERRUPT_PMC       3
#define NV_AON_INTERRUPT_V1RSVD4   4
#define NV_AON_INTERRUPT_PM        5
#define NV_AON_INTERRUPT_FPUINT    6
#define NV_AON_INTERRUPT_V1RSVD7   7
#define NV_AON_INTERRUPT_ACTMON    8
#define NV_AON_INTERRUPT_AOWDT     9
#define NV_AON_INTERRUPT_TOP0_HSP_DB 10
#define NV_AON_INTERRUPT_CTIIRQ   11
#define NV_AON_INTERRUPT_NOC_SEC  12
#define NV_AON_INTERRUPT_CAR      13
#define NV_AON_INTERRUPT_UART6    14
#define NV_AON_INTERRUPT_UART8    15
#define NV_AON_INTERRUPT_GPIO_3   16
#define NV_AON_INTERRUPT_CEC 17
#define NV_AON_INTERRUPT_V1RSVD18 18
#define NV_AON_INTERRUPT_V1RSVD19 19
#define NV_AON_INTERRUPT_V1RSVD20 20
#define NV_AON_INTERRUPT_V1RSVD21 21
#define NV_AON_INTERRUPT_V1RSVD22 22
#define NV_AON_INTERRUPT_V1RSVD23 23
#define NV_AON_INTERRUPT_V1RSVD24 24
#define NV_AON_INTERRUPT_V1RSVD25 25
#define NV_AON_INTERRUPT_V1RSVD26 26
#define NV_AON_INTERRUPT_V1RSVD27 27
#define NV_AON_INTERRUPT_V1RSVD28 28
#define NV_AON_INTERRUPT_V1RSVD29 29
#define NV_AON_INTERRUPT_V1RSVD30 30
#define NV_AON_INTERRUPT_V1RSVD31 31
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* PROCESSOR__IRQS_HW_H */
