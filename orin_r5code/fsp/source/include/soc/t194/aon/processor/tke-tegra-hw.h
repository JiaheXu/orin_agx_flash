/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef PROCESSOR__TKE_TEGRA_HW_H
#define PROCESSOR__TKE_TEGRA_HW_H
#define FSP__PROCESSOR__TKE_TEGRA_HW_H                  1

#include <misc/macros.h>

extern struct tegra_tke_id tegra_tke_id_timer0;
extern struct tegra_tke_id tegra_tke_id_timer1;
extern struct tegra_tke_id tegra_tke_id_timer2;
extern struct tegra_tke_id tegra_tke_id_timer3;

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#define TEGRA_TKE_TIMER_CLK_SRC_USECCNT		0
#define TEGRA_TKE_TIMER_CLK_SRC_OSCCNT		1
#define TEGRA_TKE_TIMER_CLK_SRC_TSC_BIT0	2
#define TEGRA_TKE_TIMER_CLK_SRC_TSC_BIT12	3
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
