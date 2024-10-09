/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
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

#ifndef SOC_COMMON__HSP_TEGRA_TOP_HW_H
#define SOC_COMMON__HSP_TEGRA_TOP_HW_H
#define FSP__SOC_COMMON__HSP_TEGRA_TOP_HW_H             1

/* FIXME: There should be a header to define this: */
/* define doorbell master ID */
#define TEGRA_HSP_DB_MASTER_TZ                  0x0
#define TEGRA_HSP_DB_MASTER_CCPLEX              0x1
#define TEGRA_HSP_DB_MASTER_CCPMU               0x2
#define TEGRA_HSP_DB_MASTER_BPMP_FW             0x3
#define TEGRA_HSP_DB_MASTER_SPE                 0x4
#define TEGRA_HSP_DB_MASTER_AON                 0x4
#define TEGRA_HSP_DB_MASTER_SCE                 0x5
#define TEGRA_HSP_DB_MASTER_DMA                 0x6
#define TEGRA_HSP_DB_MASTER_TSECA_NONSECURE     0x7
#define TEGRA_HSP_DB_MASTER_TSECB_NONSECURE     0x8
#define TEGRA_HSP_DB_MASTER_JTAGM_DFT           0x9
#define TEGRA_HSP_DB_MASTER_CSITE               0xA
#define TEGRA_HSP_DB_MASTER_APE                 0xB
#define TEGRA_HSP_DB_MASTER_PEATRANS            0xC
#define TEGRA_HSP_DB_MASTER_NVDEC_NONSECURE     0xD
#define TEGRA_HSP_DB_MASTER_RCE                 0xE
#define TEGRA_HSP_DB_MASTER_NVDEC1_NONSECURE    0xF
#define TEGRA_HSP_DB_MASTER_PSC_FW_USER         0x10
#define TEGRA_HSP_DB_MASTER_PSC_FW_SUPERVISOR   0x11
#define TEGRA_HSP_DB_MASTER_PSC_FW_MACHINE      0x12
#define TEGRA_HSP_DB_MASTER_PSC_BOOT            0x13
#define TEGRA_HSP_DB_MASTER_BPMP_BOOT           0x14
#define TEGRA_HSP_DB_MASTER_TSECA_LIGHTSECURE   0x15
#define TEGRA_HSP_DB_MASTER_TSECB_LIGHTSECURE   0x16
#define TEGRA_HSP_DB_MASTER_NVDEC_LIGHTSECURE   0x17
#define TEGRA_HSP_DB_MASTER_NVDEC1_LIGHTSECURE  0x18
#define TEGRA_HSP_DB_MASTER_TSECA_HEAVYSECURE   0x19
#define TEGRA_HSP_DB_MASTER_TSECB_HEAVYSECURE   0x1A
#define TEGRA_HSP_DB_MASTER_NVDEC_HEAVYSECURE   0x1B
#define TEGRA_HSP_DB_MASTER_NVDEC1_HEAVYSECURE  0x1C
#define TEGRA_HSP_DB_MASTER_DCE                 0x1D
#define TEGRA_HSP_DB_MASTER_CBB_INTERNAL        0x1E
#define TEGRA_HSP_DB_MASTER_RSVD                0x3F
#define TEGRA_HSP_DB_MASTER_MAXNUM              0x3F
#define TEGRA_HSP_DB_MASTER_NON_SECURE          TEGRA_HSP_DB_MASTER_NVDEC1_NONSECURE

/* FIXME: There should be a header to define this: */
/* define doorbell number and usage*/
#define TEGRA_HSP_DB_DPMU       0
#define TEGRA_HSP_DB_CCPLEX     1
#define TEGRA_HSP_DB_CCPLEX_TZ  2
#define TEGRA_HSP_DB_BPMP       3
#define TEGRA_HSP_DB_SPE        4
#define TEGRA_HSP_DB_SCE        5
#define TEGRA_HSP_DB_APE        6
#define TEGRA_HSP_DB_RCE        7
#define TEGRA_HSP_DB_DCE        8
#define TEGRA_HSP_DB_PSC        9
#define TEGRA_HSP_DB_MAXNUM     10

#endif /* SOC_COMMON__HSP_TEGRA_TOP_HW_H */
