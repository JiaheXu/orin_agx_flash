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

#define NV_BPMP_IRQ_VIC0_BASE	0
#define NV_BPMP_IRQ_VIC1_BASE	32

// VIC0 interrupts
#define NV_BPMP_IRQ_WDTIRQ 0
#define NV_BPMP_IRQ_TIMER0 1
#define NV_BPMP_IRQ_TIMER1 2
#define NV_BPMP_IRQ_TIMER2 3
#define NV_BPMP_IRQ_TIMER3 4
#define NV_BPMP_IRQ_WDTFIQ 5
#define NV_BPMP_IRQ_HSP_DB 6
#define NV_BPMP_IRQ_RSVD7  7
#define NV_BPMP_IRQ_PM     8
#define NV_BPMP_IRQ_CVC    9
#define NV_BPMP_IRQ_H1XSNC 10
#define NV_BPMP_IRQ_RSVD11 11
#define NV_BPMP_IRQ_RSVD12 12
#define NV_BPMP_IRQ_WAKE0  13
#define NV_BPMP_IRQ_WAKE1  14
#define NV_BPMP_IRQ_WAKE2  15
#define NV_BPMP_IRQ_LIC0   16
#define NV_BPMP_IRQ_LIC1   17
#define NV_BPMP_IRQ_RSVD18 18
#define NV_BPMP_IRQ_RSVD19 19
#define NV_BPMP_IRQ_DMA0   20
#define NV_BPMP_IRQ_DMA1   21
#define NV_BPMP_IRQ_DMA2   22
#define NV_BPMP_IRQ_DMA3   23
#define NV_BPMP_IRQ_HSP_SI_0 24
#define NV_BPMP_IRQ_FPUINT 25
#define NV_BPMP_IRQ_GPMU   26
#define NV_BPMP_IRQ_EDP    27
#define NV_BPMP_IRQ_SCTHRM 28
#define NV_BPMP_IRQ_RSVD29 29
#define NV_BPMP_IRQ_AONPWRI2C 30
#define NV_BPMP_IRQ_SOCPWRI2C 31

// VIC1 interrupts
#define NV_BPMP_IRQ_CPM0   0
#define NV_BPMP_IRQ_CPM1   1
#define NV_BPMP_IRQ_CPM3   2
#define NV_BPMP_IRQ_RSVD3  3
#define NV_BPMP_IRQ_RSVD4  4
#define NV_BPMP_IRQ_RSVD5  5
#define NV_BPMP_IRQ_UART   6
#define NV_BPMP_IRQ_QSPI   7
#define NV_BPMP_IRQ_PMUIRQ 8
#define NV_BPMP_IRQ_CNTRL_ACTMON   9
#define NV_BPMP_IRQ_BPMP_ACTMON    10
#define NV_BPMP_IRQ_SIMON0 11
#define NV_BPMP_IRQ_SIMON1 12
#define NV_BPMP_IRQ_SIMON2 13
#define NV_BPMP_IRQ_SIMON3 14
#define NV_BPMP_IRQ_APBERR 15
#define NV_BPMP_IRQ_GTE    16
#define NV_BPMP_IRQ_APBSEC 17
#define NV_BPMP_IRQ_CTIIRQ 18
#define NV_BPMP_IRQ_HSP_SI_1 19
#define NV_BPMP_IRQ_HSP_SI_2 20
#define NV_BPMP_IRQ_HSP_SI_3 21
#define NV_BPMP_IRQ_HSP_SI_4 22
#define NV_BPMP_IRQ_CAR      23
#define NV_BPMP_IRQ_V1RSVD24 24
#define NV_BPMP_IRQ_V1RSVD25 25
#define NV_BPMP_IRQ_V1RSVD26 26
#define NV_BPMP_IRQ_V1RSVD27 27
#define NV_BPMP_IRQ_V1RSVD28 28
#define NV_BPMP_IRQ_V1RSVD29 29
#define NV_BPMP_IRQ_V1RSVD30 30
#define NV_BPMP_IRQ_V1RSVD31 31

#endif /* PROCESSOR__IRQS_HW_H */
