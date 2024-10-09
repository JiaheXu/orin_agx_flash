/*
* Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

#define MAX_VIC_CONTROLLER       2
#define NV_FSI_SPI_INTR_BASE    32
#define NV_FSI_PPI_INTR_BASE    16

/* SPI interupts source numbers starts */
/* SPI irq must be added by 32 (NV_FSI_SPI_INTR_BASE) while using */
#define NV_FSI_GIC_TIMER_INTR0 4
#define NV_FSI_GIC_TIMER_INTR1 34
#define NV_FSI_GIC_TIMER_INTR2 66
#define NV_FSI_GIC_TIMER_INTR3 96
#define NV_FSI_GIC_TOP_HSP2_SI0 21
#define NV_FSI_GIC_TOP_HSP2_SI1 22

#define NV_FSI_GIC_TOP_HSP2_SI2 44
#define NV_FSI_GIC_TOP_HSP2_SI4 46
#define NV_FSI_GIC_HSP_SI0 3
#define NV_FSI_GIC_HSP_SI2 64
#define NV_FSI_GIC_HSP_SI4 28
#define NV_FSI_GIC_SCE_HSP1_SI2 15
#define NV_FSI_GIC_SCE_HSP1_SI3 16
#define NV_FSI_GIC_SCE_HSP1_SI4 36
#define NV_FSI_GIC_SCE_HSP2_SI2 42
#define NV_FSI_GIC_SCE_HSP2_SI4 17

/* FSI GPCDMA interrupt numbers */
#define NV_FSI_GIC_DMA_INTR0 26
#define NV_FSI_GIC_DMA_INTR1 27
#define NV_FSI_GIC_DMA_INTR2 52
#define NV_FSI_GIC_DMA_INTR3 53
#define NV_FSI_GIC_DMA_INTR4 67
#define NV_FSI_GIC_DMA_INTR5 68
#define NV_FSI_GIC_DMA_INTR6 97
#define NV_FSI_GIC_DMA_INTR7 98
#define NV_FSI_GIC_DMA_INTR8 99

/* PPI interupt source numbers */
/* PPI irq numbers must be added by 16 (PPI_BASE) while using*/
#define NV_FSI_GIC_WDT_nIRQ0    0
#define NV_FSI_GIC_WDT_nFIQ0    1
#define NV_FSI_GIC_WDT_nIRQ1    0
#define NV_FSI_GIC_WDT_nFIQ1    1
#define NV_FSI_GIC_WDT_nIRQ2    0
#define NV_FSI_GIC_WDT_nFIQ2    1
#define NV_FSI_GIC_WDT_nIRQ3    0
#define NV_FSI_GIC_WDT_nFIQ3    1

#endif /* PROCESSOR__IRQS_HW_H */
