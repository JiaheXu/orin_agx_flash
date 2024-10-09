/*
* Copyright (c) 2016-2019 NVIDIA CORPORATION.  All rights reserved.
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

#define NV_AON_INTERRUPT_VIC0_BASE	0
#define NV_AON_INTERRUPT_VIC1_BASE	32

// VIC0 interrupts
#define	NV_SCE_IRQ_WDTFIQ	 	 0
#define	NV_SCE_IRQ_WDTIRQ	 	 1
#define	NV_SCE_IRQ_TIMER0	 	 2
#define	NV_SCE_IRQ_TIMER1	 	 3
#define	NV_SCE_IRQ_TIMER2	 	 4
#define	NV_SCE_IRQ_TIMER3	 	 5
#define	NV_SCE_IRQ_MBOX		 	 6
#define	NV_SCE_IRQ_GTE		 	 7
#define	NV_SCE_IRQ_PMU		 	 8
#define	NV_SCE_IRQ_DMA0		 	 9
#define	NV_SCE_IRQ_DMA1			10
#define	NV_SCE_IRQ_DMA2			11
#define	NV_SCE_IRQ_DMA3			12
#define	NV_SCE_IRQ_DMA4			13
#define	NV_SCE_IRQ_DMA5			14
#define	NV_SCE_IRQ_DMA6			15
#define	NV_SCE_IRQ_DMA7			16
#define	NV_SCE_IRQ_LIC0			17
#define	NV_SCE_IRQ_LIC1			18
#define	NV_SCE_IRQ_LIC2			19
#define	NV_SCE_IRQ_LIC3			20
#define	NV_SCE_IRQ_V0RSVD21		21
#define	NV_SCE_IRQ_HSM_CRITICAL_ERR	22
#define	NV_SCE_IRQ_VI_HP		23
#define	NV_SCE_IRQ_VI_LP		24
#define	NV_SCE_IRQ_HSM_HP		25
#define	NV_SCE_IRQ_HSM_LP		26
#define	NV_SCE_IRQ_V0RSVD27		27
#define	NV_SCE_IRQ_V0RSVD28		28
#define	NV_SCE_IRQ_V0RSVD29		29
#define	NV_SCE_IRQ_V0RSVD30		30
#define	NV_SCE_IRQ_V0RSVD31		31

/* VIC1 interrupts */
#define NV_SCE_IRQ_NOC_NON_SECURE	32
#define	NV_SCE_IRQ_ACTMON		33
#define	NV_SCE_IRQ_FPUINT		34
#define	NV_SCE_IRQ_PM			35
#define	NV_SCE_IRQ_MC_SBE		36
#define	NV_SCE_IRQ_NOC_SECURE		37
#define	NV_SCE_IRQ_CAR			38
#define	NV_SCE_IRQ_V1RSVD7		39
#define	NV_SCE_IRQ_V1RSVD8		40
#define	NV_SCE_IRQ_V1RSVD9		41
#define	NV_SCE_IRQ_V1RSVD10		42
#define	NV_SCE_IRQ_V1RSVD11		43
#define	NV_SCE_IRQ_V1RSVD12		44
#define	NV_SCE_IRQ_V1RSVD13		45
#define	NV_SCE_IRQ_TOP0_HSP_DB		46
#define	NV_SCE_IRQ_V1RSVD15		47
#define	NV_SCE_IRQ_V1RSVD16		48
#define	NV_SCE_IRQ_CTIIRQ		49
#define	NV_SCE_IRQ_V1RSVD18		50
#define	NV_SCE_IRQ_I2C1			51
#define	NV_SCE_IRQ_I2C3			52
#define	NV_SCE_IRQ_I2C8			53
#define	NV_SCE_IRQ_V1RSVD22		54
#define	NV_SCE_IRQ_V1RSVD23		55
#define	NV_SCE_IRQ_V1RSVD24		56
#define	NV_SCE_IRQ_V1RSVD25		57
#define	NV_SCE_IRQ_V1RSVD26		58
#define	NV_SCE_IRQ_V1RSVD27		59
#define	NV_SCE_IRQ_V1RSVD28		60
#define	NV_SCE_IRQ_V1RSVD29		61
#define	NV_SCE_IRQ_V1RSVD30		62
#define	NV_SCE_IRQ_V1RSVD31		63

#endif /* PROCESSOR__IRQS_HW_H */
