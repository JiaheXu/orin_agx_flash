/*
 * Copyright (c) 2016-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef PROCESSOR__TEGRA_UART_HW_PARAMS_H
#define PROCESSOR__TEGRA_UART_HW_PARAMS_H
#define FSP__PROCESSOR__TEGRA_UART_HW_PARAMS_H          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <processor/irqs-hw.h>

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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

#ifndef TEGRA_UARTC_ENABLED
#define TEGRA_UARTC_ENABLED
#endif
#define TEGRA_UARTC_IRQ NV_AON_INTERRUPT_UART_1

#define TEGRA_UARTF_ENABLED
#define TEGRA_UARTF_IRQ NV_AON_INTERRUPT_VIC1_BASE + NV_AON_INTERRUPT_UART6

#define TEGRA_UARTG_ENABLED
#define TEGRA_UARTG_IRQ NV_AON_INTERRUPT_UART_2

#define TEGRA_UARTH_ENABLED
#define TEGRA_UARTH_IRQ NV_AON_INTERRUPT_VIC1_BASE + NV_AON_INTERRUPT_UART8


#endif