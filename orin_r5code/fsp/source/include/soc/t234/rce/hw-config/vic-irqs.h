/*
 * t234/rce/hw-config/vic-irqs.h - Interrupts for safertos-common handlers
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

#include <misc/ct-assert.h>
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
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")

/*
 * Define the various vectors where the VIC and IRQ are encoded into one value
 */
#define INTERRUPT_WDTFIQ            TEGRA_INTERRUPT(0, NV_RCE_IRQ_WDTFIQ)
#define INTERRUPT_WDTIRQ            TEGRA_INTERRUPT(0, NV_RCE_IRQ_WDTIRQ)
#define INTERRUPT_TIMER0            TEGRA_INTERRUPT(0, NV_RCE_IRQ_TIMER0)
#define INTERRUPT_TIMER1            TEGRA_INTERRUPT(0, NV_RCE_IRQ_TIMER1)
#define INTERRUPT_TIMER2            TEGRA_INTERRUPT(0, NV_RCE_IRQ_TIMER2)
#define INTERRUPT_TIMER3            TEGRA_INTERRUPT(0, NV_RCE_IRQ_TIMER3)
#define INTERRUPT_MBOX              TEGRA_INTERRUPT(0, NV_RCE_IRQ_MBOX)
#define INTERRUPT_GTE               TEGRA_INTERRUPT(0, NV_RCE_IRQ_GTE)
#define INTERRUPT_PMU               TEGRA_INTERRUPT(0, NV_RCE_IRQ_PMU)
#define INTERRUPT_DMA0              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA0)
#define INTERRUPT_DMA1              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA1)
#define INTERRUPT_DMA2              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA2)
#define INTERRUPT_DMA3              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA3)
#define INTERRUPT_DMA4              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA4)
#define INTERRUPT_DMA5              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA5)
#define INTERRUPT_DMA6              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA6)
#define INTERRUPT_DMA7              TEGRA_INTERRUPT(0, NV_RCE_IRQ_DMA7)
#define INTERRUPT_LIC0              TEGRA_INTERRUPT(0, NV_RCE_IRQ_LIC0)
#define INTERRUPT_LIC1              TEGRA_INTERRUPT(0, NV_RCE_IRQ_LIC1)
#define INTERRUPT_LIC2              TEGRA_INTERRUPT(0, NV_RCE_IRQ_LIC2)
#define INTERRUPT_LIC3              TEGRA_INTERRUPT(0, NV_RCE_IRQ_LIC3)
#define INTERRUPT_V0RSVD21          TEGRA_INTERRUPT(0, NV_RCE_IRQ_V0RSVD21)
#define INTERRUPT_HSM_CRITICAL_ERR  TEGRA_INTERRUPT(0, NV_RCE_IRQ_HSM_CRITICAL_ERR)
#define INTERRUPT_VI_HP             TEGRA_INTERRUPT(0, NV_RCE_IRQ_VI_HP)
#define INTERRUPT_VI2_HP            TEGRA_INTERRUPT(0, NV_RCE_IRQ_VI2_HP)
#define INTERRUPT_HSM_HP            TEGRA_INTERRUPT(0, NV_RCE_IRQ_HSM_HP)
#define INTERRUPT_HSM_LP            TEGRA_INTERRUPT(0, NV_RCE_IRQ_HSM_LP)
#define INTERRUPT_VI_LP             TEGRA_INTERRUPT(0, NV_RCE_IRQ_VI_LP)
#define INTERRUPT_V0RSVD28          TEGRA_INTERRUPT(0, NV_RCE_IRQ_V0RSVD28)
#define INTERRUPT_V0RSVD29          TEGRA_INTERRUPT(0, NV_RCE_IRQ_V0RSVD29)
#define INTERRUPT_V0RSVD30          TEGRA_INTERRUPT(0, NV_RCE_IRQ_V0RSVD30)
#define INTERRUPT_V0RSVD31          TEGRA_INTERRUPT(0, NV_RCE_IRQ_V0RSVD31)

// VIC1 interrupts
#define INTERRUPT_V1RSVD0           TEGRA_INTERRUPT(1, NV_RCE_IRQ_V1RSVD0)
#define INTERRUPT_ACTMON            TEGRA_INTERRUPT(1, NV_RCE_IRQ_ACTMON)
#define INTERRUPT_FPUINT            TEGRA_INTERRUPT(1, NV_RCE_IRQ_FPUINT)
#define INTERRUPT_PM                TEGRA_INTERRUPT(1, NV_RCE_IRQ_PM)
#define INTERRUPT_MC_SBE            TEGRA_INTERRUPT(1, NV_RCE_IRQ_MC_SBE)
#define INTERRUPT_NOC_SECURE        TEGRA_INTERRUPT(1, NV_RCE_IRQ_NOC_SECURE)
#define INTERRUPT_HSP1_SI_0         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_0)
#define INTERRUPT_HSP1_SI_1         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_1)
#define INTERRUPT_HSP1_SI_2         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_2)
#define INTERRUPT_HSP1_SI_3         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_3)
#define INTERRUPT_HSP1_SI_4         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_4)
#define INTERRUPT_HSP1_SI_5         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_5)
#define INTERRUPT_HSP1_SI_6         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_6)
#define INTERRUPT_HSP1_SI_7         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP1_SI_7)
#define INTERRUPT_TOP0_HSP_DB       TEGRA_INTERRUPT(1, NV_RCE_IRQ_TOP0_HSP_DB)
#define INTERRUPT_CAR               TEGRA_INTERRUPT(1, NV_RCE_IRQ_CAR)
#define INTERRUPT_V1RSVD16          TEGRA_INTERRUPT(1, NV_RCE_IRQ_V1RSVD16)
#define INTERRUPT_CTIIRQ            TEGRA_INTERRUPT(1, NV_RCE_IRQ_CTIIRQ)
#define INTERRUPT_V1RSVD18          TEGRA_INTERRUPT(1, NV_RCE_IRQ_V1RSVD18)
#define INTERRUPT_I2C1              TEGRA_INTERRUPT(1, NV_RCE_IRQ_I2C1)
#define INTERRUPT_I2C3              TEGRA_INTERRUPT(1, NV_RCE_IRQ_I2C3)
#define INTERRUPT_I2C8              TEGRA_INTERRUPT(1, NV_RCE_IRQ_I2C8)
#define INTERRUPT_DISP              TEGRA_INTERRUPT(1, NV_RCE_IRQ_DISP)
#define INTERRUPT_V1RSVD23          TEGRA_INTERRUPT(1, NV_RCE_IRQ_V1RSVD23)
#define INTERRUPT_HSP2_SI_0         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_0)
#define INTERRUPT_HSP2_SI_1         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_1)
#define INTERRUPT_HSP2_SI_2         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_2)
#define INTERRUPT_HSP2_SI_3         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_3)
#define INTERRUPT_HSP2_SI_4         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_4)
#define INTERRUPT_HSP2_SI_5         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_5)
#define INTERRUPT_HSP2_SI_6         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_6)
#define INTERRUPT_HSP2_SI_7         TEGRA_INTERRUPT(1, NV_RCE_IRQ_HSP2_SI_7)

#endif  /* HW_CONFIG__VIC_IRQS_H */
