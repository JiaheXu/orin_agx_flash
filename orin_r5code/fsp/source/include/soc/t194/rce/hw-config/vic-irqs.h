/*
 * t194/rce/hw-config/vic-irqs.h - Interrupts for safertos-common handlers
 *
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef HW_CONFIG__VIC_IRQS_H
#define HW_CONFIG__VIC_IRQS_H
#define FSP__HW_CONFIG__VIC_IRQS_H                      1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <processor/irqs-hw.h>
#include <vic/tegra-vic.h>

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
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")

#define INTERRUPT_WDTFIQ		NV_RCE_IRQ_WDTFIQ
#define INTERRUPT_WDTIRQ		NV_RCE_IRQ_WDTIRQ
#define INTERRUPT_TIMER0		NV_RCE_IRQ_TIMER0
#define INTERRUPT_TIMER1		NV_RCE_IRQ_TIMER1
#define INTERRUPT_TIMER2		NV_RCE_IRQ_TIMER2
#define INTERRUPT_TIMER3		NV_RCE_IRQ_TIMER3

#endif  /* HW_CONFIG__VIC_IRQS_H */
