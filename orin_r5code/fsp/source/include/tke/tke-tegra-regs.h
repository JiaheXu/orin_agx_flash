/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TKE__TKE_TEGRA_REGS_H
#define TKE__TKE_TEGRA_REGS_H
#define FSP__TKE__TKE_TEGRA_REGS_H                      1

/* Late FSP headers */
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#define TKE_TIMER_TMRCR_0               0x0
#define TKE_TIMER_TMRCR_0_PTV           0x1fffffffU
#define TKE_TIMER_TMRCR_0_PER           BIT(30)
#define TKE_TIMER_TMRCR_0_EN            BIT(31)

#define TKE_TIMER_TMRSR_0               0x4
#define TKE_TIMER_TMRSR_0_INTR_CLR      BIT(30)
#define TKE_TIMER_TMRSR_0_PCV           0x1fffffffU

#define TKE_TIMER_TMRCSSR_0             0x8

#define TSC_MTSCANNR_0                  0x8
#define TSC_MTSCANDR_0                  0xc

#define TSC_MTSCANNR_0_M0_SHIFT         16
#define TSC_MTSCANNR_0_R0_SHIFT         0
#define TSC_MTSCANDR_0_D0_SHIFT         0
#define TSC_MTSCANNR_0_M0_MASK          (0xfff << TSC_MTSCANNR_0_M0_SHIFT)
#define TSC_MTSCANNR_0_R0_MASK          (0xfff << TSC_MTSCANNR_0_R0_SHIFT)
#define TSC_MTSCANDR_0_D0_MASK          (0xfff << TSC_MTSCANDR_0_D0_SHIFT)

#define TSC_BASE_RATE                   32768UL
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif  /* FSP_TKE_TKE_TEGRA_REGS_H */
