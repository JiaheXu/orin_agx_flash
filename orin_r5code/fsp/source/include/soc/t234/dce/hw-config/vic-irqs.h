/*
 * Copyright (c) 2018-2019 NVIDIA CORPORATION.  All rights reserved.
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
#include <soc-common/hw-const.h>

/* Hardware headers */
#include <arsce_interrupts.h>
#include <vic/tegra-vic.h>

/* Late FSP headers */

#define NUM_VICS                        2

/*
 * VIC0 Interrupts
 */
#define ISR_NUM_WDTFIQ                  NV_SCE_INTERRUPT_WDTFIQ
#define ISR_NUM_WDTIRQ                  NV_SCE_INTERRUPT_WDTIRQ
#define ISR_NUM_TIMER0                  NV_SCE_INTERRUPT_TIMER0
#define ISR_NUM_TIMER1                  NV_SCE_INTERRUPT_TIMER1
#define ISR_NUM_TIMER2                  NV_SCE_INTERRUPT_TIMER2
#define ISR_NUM_TIMER3                  NV_SCE_INTERRUPT_TIMER3
#define ISR_NUM_MBOX                    NV_SCE_INTERRUPT_MBOX
#define ISR_NUM_GTE                     NV_SCE_INTERRUPT_GTE
#define ISR_NUM_PMU                     NV_SCE_INTERRUPT_PMU
#define ISR_NUM_DMA0                    NV_SCE_INTERRUPT_DMA0
#define ISR_NUM_DMA1                    NV_SCE_INTERRUPT_DMA1
#define ISR_NUM_DMA2                    NV_SCE_INTERRUPT_DMA2
#define ISR_NUM_DMA3                    NV_SCE_INTERRUPT_DMA3
#define ISR_NUM_DMA4                    NV_SCE_INTERRUPT_DMA4
#define ISR_NUM_DMA5                    NV_SCE_INTERRUPT_DMA5
#define ISR_NUM_DMA6                    NV_SCE_INTERRUPT_DMA6
#define ISR_NUM_DMA7                    NV_SCE_INTERRUPT_DMA7
#define ISR_NUM_LIC0                    NV_SCE_INTERRUPT_LIC0
#define ISR_NUM_LIC1                    NV_SCE_INTERRUPT_LIC1
#define ISR_NUM_LIC2                    NV_SCE_INTERRUPT_LIC2
#define ISR_NUM_LIC3                    NV_SCE_INTERRUPT_LIC3
#define ISR_NUM_V0RSVD21                NV_SCE_INTERRUPT_V0RSVD21
#define ISR_NUM_HSM_CRITICAL_ERR        NV_SCE_INTERRUPT_HSM_CRITICAL_ERR
#define ISR_NUM_VI_HP                   NV_SCE_INTERRUPT_VI_HP
#define ISR_NUM_VI_LP                   NV_SCE_INTERRUPT_VI_LP
#define ISR_NUM_HSM_HP                  NV_SCE_INTERRUPT_HSM_HP
#define ISR_NUM_HSM_LP                  NV_SCE_INTERRUPT_HSM_LP
#define ISR_NUM_V0RSVD27                NV_SCE_INTERRUPT_V0RSVD27
#define ISR_NUM_V0RSVD28                NV_SCE_INTERRUPT_V0RSVD28
#define ISR_NUM_V0RSVD29                NV_SCE_INTERRUPT_V0RSVD29
#define ISR_NUM_V0RSVD30                NV_SCE_INTERRUPT_V0RSVD30
#define ISR_NUM_V0RSVD31                NV_SCE_INTERRUPT_V0RSVD31

/*
 * VIC1 Interrupts
 */
#define ISR_NUM_V1RSVD0                 NV_SCE_INTERRUPT_V1RSVD0
#define ISR_NUM_ACTMON                  NV_SCE_INTERRUPT_ACTMON
#define ISR_NUM_FPUINT                  NV_SCE_INTERRUPT_FPUINT
#define ISR_NUM_PM                      NV_SCE_INTERRUPT_PM
#define ISR_NUM_MC_SBE                  NV_SCE_INTERRUPT_MC_SBE
#define ISR_NUM_NOC_SECURE              NV_SCE_INTERRUPT_NOC_SECURE
#define ISR_NUM_CAR                     NV_SCE_INTERRUPT_CAR
#define ISR_NUM_V1RSVD7                 NV_SCE_INTERRUPT_V1RSVD7
#define ISR_NUM_V1RSVD8                 NV_SCE_INTERRUPT_V1RSVD8
#define ISR_NUM_V1RSVD9                 NV_SCE_INTERRUPT_V1RSVD9
#define ISR_NUM_V1RSVD10                NV_SCE_INTERRUPT_V1RSVD10
#define ISR_NUM_V1RSVD11                NV_SCE_INTERRUPT_V1RSVD11
#define ISR_NUM_V1RSVD12                NV_SCE_INTERRUPT_V1RSVD12
#define ISR_NUM_V1RSVD13                NV_SCE_INTERRUPT_V1RSVD13
#define ISR_NUM_TOP0_HSP_DB             NV_SCE_INTERRUPT_TOP0_HSP_DB
#define ISR_NUM_V1RSVD15                NV_SCE_INTERRUPT_V1RSVD15
#define ISR_NUM_V1RSVD16                NV_SCE_INTERRUPT_V1RSVD16
#define ISR_NUM_CTIIRQ                  NV_SCE_INTERRUPT_CTIIRQ
#define ISR_NUM_V1RSVD18                NV_SCE_INTERRUPT_V1RSVD18
#define ISR_NUM_I2C1                    NV_SCE_INTERRUPT_I2C1
#define ISR_NUM_I2C3                    NV_SCE_INTERRUPT_I2C3
#define ISR_NUM_I2C8                    NV_SCE_INTERRUPT_I2C8
#define ISR_NUM_DISP                    NV_SCE_INTERRUPT_DISP
#define ISR_NUM_V1RSVD23                NV_SCE_INTERRUPT_V1RSVD23
#define ISR_NUM_HSP2_SI_0               NV_SCE_INTERRUPT_HSP2_SI_0
#define ISR_NUM_HSP2_SI_1               NV_SCE_INTERRUPT_HSP2_SI_1
#define ISR_NUM_HSP2_SI_2               NV_SCE_INTERRUPT_HSP2_SI_2
#define ISR_NUM_HSP2_SI_3               NV_SCE_INTERRUPT_HSP2_SI_3
#define ISR_NUM_HSP2_SI_4               NV_SCE_INTERRUPT_HSP2_SI_4
#define ISR_NUM_HSP2_SI_5               NV_SCE_INTERRUPT_HSP2_SI_5
#define ISR_NUM_HSP2_SI_6               NV_SCE_INTERRUPT_HSP2_SI_6
#define ISR_NUM_HSP2_SI_7               NV_SCE_INTERRUPT_HSP2_SI_7

/*
 * Define the various vectors where the VIC and IRQ are encoded into one value
 */
#define INTERRUPT_WDTFIQ                TEGRA_INTERRUPT(0, ISR_NUM_WDTFIQ)
#define INTERRUPT_WDTIRQ                TEGRA_INTERRUPT(0, ISR_NUM_WDTIRQ)
#define INTERRUPT_TIMER0                TEGRA_INTERRUPT(0, ISR_NUM_TIMER0)
#define INTERRUPT_TIMER1                TEGRA_INTERRUPT(0, ISR_NUM_TIMER1)
#define INTERRUPT_TIMER2                TEGRA_INTERRUPT(0, ISR_NUM_TIMER2)
#define INTERRUPT_TIMER3                TEGRA_INTERRUPT(0, ISR_NUM_TIMER3)
#define INTERRUPT_MBOX                  TEGRA_INTERRUPT(0, ISR_NUM_MBOX)
#define INTERRUPT_GTE                   TEGRA_INTERRUPT(0, ISR_NUM_GTE)
#define INTERRUPT_PMU                   TEGRA_INTERRUPT(0, ISR_NUM_PMU)
#define INTERRUPT_DMA0                  TEGRA_INTERRUPT(0, ISR_NUM_DMA0)
#define INTERRUPT_DMA1                  TEGRA_INTERRUPT(0, ISR_NUM_DMA1)
#define INTERRUPT_DMA2                  TEGRA_INTERRUPT(0, ISR_NUM_DMA2)
#define INTERRUPT_DMA3                  TEGRA_INTERRUPT(0, ISR_NUM_DMA3)
#define INTERRUPT_DMA4                  TEGRA_INTERRUPT(0, ISR_NUM_DMA4)
#define INTERRUPT_DMA5                  TEGRA_INTERRUPT(0, ISR_NUM_DMA5)
#define INTERRUPT_DMA6                  TEGRA_INTERRUPT(0, ISR_NUM_DMA6)
#define INTERRUPT_DMA7                  TEGRA_INTERRUPT(0, ISR_NUM_DMA7)
#define INTERRUPT_LIC0                  TEGRA_INTERRUPT(0, ISR_NUM_LIC0)
#define INTERRUPT_LIC1                  TEGRA_INTERRUPT(0, ISR_NUM_LIC1)
#define INTERRUPT_LIC2                  TEGRA_INTERRUPT(0, ISR_NUM_LIC2)
#define INTERRUPT_LIC3                  TEGRA_INTERRUPT(0, ISR_NUM_LIC3)
#define INTERRUPT_V0RSVD21              TEGRA_INTERRUPT(0, ISR_NUM_V0RSVD21)
#define INTERRUPT_HSM_CRIT_ERR          TEGRA_INTERRUPT(0, ISR_NUM_HSM_CRITICAL_ERR)
#define INTERRUPT_VI_HP                 TEGRA_INTERRUPT(0, ISR_NUM_VI_HP)
#define INTERRUPT_VI_LP                 TEGRA_INTERRUPT(0, ISR_NUM_VI_LP)
#define INTERRUPT_HSM_HP                TEGRA_INTERRUPT(0, ISR_NUM_HSM_HP)
#define INTERRUPT_HSM_LP                TEGRA_INTERRUPT(0, ISR_NUM_HSM_LP)
#define INTERRUPT_V0RSVD27              TEGRA_INTERRUPT(0, ISR_NUM_V0RSVD27)
#define INTERRUPT_V0RSVD28              TEGRA_INTERRUPT(0, ISR_NUM_V0RSVD28)
#define INTERRUPT_V0RSVD29              TEGRA_INTERRUPT(0, ISR_NUM_V0RSVD29)
#define INTERRUPT_V0RSVD30              TEGRA_INTERRUPT(0, ISR_NUM_V0RSVD30)
#define INTERRUPT_V0RSVD31              TEGRA_INTERRUPT(0, ISR_NUM_V0RSVD31)
#define INTERRUPT_V1RSVD0               TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD0)
#define INTERRUPT_ACTMON                TEGRA_INTERRUPT(1, ISR_NUM_ACTMON)
#define INTERRUPT_FPUINT                TEGRA_INTERRUPT(1, ISR_NUM_FPUINT)
#define INTERRUPT_PM                    TEGRA_INTERRUPT(1, ISR_NUM_PM)
#define INTERRUPT_MC_SBE                TEGRA_INTERRUPT(1, ISR_NUM_MC_SBE)
#define INTERRUPT_NOC_SECURE            TEGRA_INTERRUPT(1, ISR_NUM_NOC_SECURE)
#define INTERRUPT_CAR                   TEGRA_INTERRUPT(1, ISR_NUM_CAR)
#define INTERRUPT_V1RSVD7               TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD7)
#define INTERRUPT_V1RSVD8               TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD8)
#define INTERRUPT_V1RSVD9               TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD9)
#define INTERRUPT_V1RSVD10              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD10)
#define INTERRUPT_V1RSVD11              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD11)
#define INTERRUPT_V1RSVD12              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD12)
#define INTERRUPT_V1RSVD13              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD13)
#define INTERRUPT_TOP0_HSP_DB           TEGRA_INTERRUPT(1, ISR_NUM_TOP0_HSP_DB)
#define INTERRUPT_V1RSVD15              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD15)
#define INTERRUPT_V1RSVD16              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD16)
#define INTERRUPT_CTIIRQ                TEGRA_INTERRUPT(1, ISR_NUM_CTIIRQ)
#define INTERRUPT_V1RSVD18              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD18)
#define INTERRUPT_I2C1                  TEGRA_INTERRUPT(1, ISR_NUM_I2C1)
#define INTERRUPT_I2C3                  TEGRA_INTERRUPT(1, ISR_NUM_I2C3)
#define INTERRUPT_I2C8                  TEGRA_INTERRUPT(1, ISR_NUM_I2C8)
#define INTERRUPT_DISP                  TEGRA_INTERRUPT(1, ISR_NUM_DISP)
#define INTERRUPT_V1RSVD23              TEGRA_INTERRUPT(1, ISR_NUM_V1RSVD23)
#define INTERRUPT_HSP2_SI_0             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_0)
#define INTERRUPT_HSP2_SI_1             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_1)
#define INTERRUPT_HSP2_SI_2             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_2)
#define INTERRUPT_HSP2_SI_3             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_3)
#define INTERRUPT_HSP2_SI_4             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_4)
#define INTERRUPT_HSP2_SI_5             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_5)
#define INTERRUPT_HSP2_SI_6             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_6)
#define INTERRUPT_HSP2_SI_7             TEGRA_INTERRUPT(1, ISR_NUM_HSP2_SI_7)

#endif
