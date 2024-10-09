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

#ifndef NV_RCE_IRQ
#define NV_RCE_IRQ(x) (x ## U)
#endif

/* VIC0 interrupts */
#define NV_RCE_IRQ_WDTFIQ	NV_RCE_IRQ(0)
#define NV_RCE_IRQ_WDTIRQ	NV_RCE_IRQ(1)
#define NV_RCE_IRQ_TIMER0	NV_RCE_IRQ(2)
#define NV_RCE_IRQ_TIMER1	NV_RCE_IRQ(3)
#define NV_RCE_IRQ_TIMER2	NV_RCE_IRQ(4)
#define NV_RCE_IRQ_TIMER3	NV_RCE_IRQ(5)
#define NV_RCE_IRQ_MBOX	        NV_RCE_IRQ(6)
#define NV_RCE_IRQ_GTE	        NV_RCE_IRQ(7)
#define NV_RCE_IRQ_PMU	        NV_RCE_IRQ(8)
#define NV_RCE_IRQ_DMA0	        NV_RCE_IRQ(9)
#define NV_RCE_IRQ_DMA1	        NV_RCE_IRQ(10)
#define NV_RCE_IRQ_DMA2	        NV_RCE_IRQ(11)
#define NV_RCE_IRQ_DMA3	        NV_RCE_IRQ(12)
#define NV_RCE_IRQ_DMA4	        NV_RCE_IRQ(13)
#define NV_RCE_IRQ_DMA5	        NV_RCE_IRQ(14)
#define NV_RCE_IRQ_DMA6	        NV_RCE_IRQ(15)
#define NV_RCE_IRQ_DMA7	        NV_RCE_IRQ(16)
#define NV_RCE_IRQ_LIC0	        NV_RCE_IRQ(17)
#define NV_RCE_IRQ_LIC1	        NV_RCE_IRQ(18)
#define NV_RCE_IRQ_LIC2	        NV_RCE_IRQ(19)
#define NV_RCE_IRQ_LIC3	        NV_RCE_IRQ(20)
#define NV_RCE_IRQ_V0RSVD21	NV_RCE_IRQ(21)
#define NV_RCE_IRQ_V0RSVD22	NV_RCE_IRQ(22)
#define NV_RCE_IRQ_VI_HP	NV_RCE_IRQ(23)
#define NV_RCE_IRQ_VI_LP	NV_RCE_IRQ(24)
#define NV_RCE_IRQ_HSM_HP	NV_RCE_IRQ(25)
#define NV_RCE_IRQ_HSM_LP	NV_RCE_IRQ(26)
#define NV_RCE_IRQ_CPU_ERR	NV_RCE_IRQ(27)
#define NV_RCE_IRQ_SC_SYSINTR	NV_RCE_IRQ(28)
#define NV_RCE_IRQ_SC_SYSINTL	NV_RCE_IRQ(29)
#define NV_RCE_IRQ_SC_WARN	NV_RCE_IRQ(30)
#define NV_RCE_IRQ_SC_INFO	NV_RCE_IRQ(31)

/* VIC1 interrupts */
#define NV_RCE_IRQ_APBERR	NV_RCE_IRQ(32)
#define NV_RCE_IRQ_ACTMON	NV_RCE_IRQ(33)
#define NV_RCE_IRQ_FPUINT	NV_RCE_IRQ(34)
#define NV_RCE_IRQ_PM		NV_RCE_IRQ(35)
#define NV_RCE_IRQ_MC_SBE	NV_RCE_IRQ(36)
#define NV_RCE_IRQ_APBSEC	NV_RCE_IRQ(37)
#define NV_RCE_IRQ_CAR		NV_RCE_IRQ(38)
#define NV_RCE_IRQ_V1RSVD7	NV_RCE_IRQ(39)
#define NV_RCE_IRQ_V1RSVD8	NV_RCE_IRQ(40)
#define NV_RCE_IRQ_V1RSVD9	NV_RCE_IRQ(41)
#define NV_RCE_IRQ_V1RSVD10	NV_RCE_IRQ(42)
#define NV_RCE_IRQ_V1RSVD11	NV_RCE_IRQ(43)
#define NV_RCE_IRQ_V1RSVD12	NV_RCE_IRQ(44)
#define NV_RCE_IRQ_V1RSVD13	NV_RCE_IRQ(45)
#define NV_RCE_IRQ_TOP0_HSP_DB	NV_RCE_IRQ(46)
#define NV_RCE_IRQ_V1RSVD15	NV_RCE_IRQ(47)
#define NV_RCE_IRQ_V1RSVD16	NV_RCE_IRQ(48)
#define NV_RCE_IRQ_CTIIRQ	NV_RCE_IRQ(49)
#define NV_RCE_IRQ_V1RSVD18	NV_RCE_IRQ(50)
#define NV_RCE_IRQ_I2C1	        NV_RCE_IRQ(51)
#define NV_RCE_IRQ_I2C3	        NV_RCE_IRQ(52)
#define NV_RCE_IRQ_I2C8	        NV_RCE_IRQ(53)
#define NV_RCE_IRQ_V1RSVD22	NV_RCE_IRQ(54)
#define NV_RCE_IRQ_V1RSVD23	NV_RCE_IRQ(55)
#define NV_RCE_IRQ_V1RSVD24	NV_RCE_IRQ(56)
#define NV_RCE_IRQ_V1RSVD25	NV_RCE_IRQ(57)
#define NV_RCE_IRQ_V1RSVD26	NV_RCE_IRQ(58)
#define NV_RCE_IRQ_V1RSVD27	NV_RCE_IRQ(59)
#define NV_RCE_IRQ_V1RSVD28	NV_RCE_IRQ(60)
#define NV_RCE_IRQ_V1RSVD29	NV_RCE_IRQ(61)
#define NV_RCE_IRQ_V1RSVD30	NV_RCE_IRQ(62)
#define NV_RCE_IRQ_V1RSVD31	NV_RCE_IRQ(63)

#endif  /* T194_RCE_PROCESSOR_IRQS_HW_H */
