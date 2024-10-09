/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2018-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef NVRISCV_PEREGRINE_H
#define NVRISCV_PEREGRINE_H

/*!
 * @file    engine.h
 * @brief   This file hides header differences between different Peregrines.
 */

// Register headers
#include NVRISCV64_MANUAL_LOCAL_IO

#if defined(NVRISCV_IS_ENGINE_gsp)
    #if (NVRISCV_HAS_PRI || NVRISCV_HAS_CSB_OVER_PRI)
        #include <dev_falcon_v4.h>
        #include <dev_riscv_pri.h>
        #define FALCON_BASE   NV_FALCON_GSP_BASE
    #endif
    #if NVRISCV_HAS_PRI
        #define RISCV_BASE    NV_FALCON2_GSP_BASE
    #endif
#elif defined(NVRISCV_IS_ENGINE_pmu)
    #if (NVRISCV_HAS_PRI || NVRISCV_HAS_CSB_OVER_PRI)
        #include <dev_falcon_v4.h>
        #include <dev_riscv_pri.h>
        #define FALCON_BASE   NV_FALCON_PWR_BASE
    #endif
    #if NVRISCV_HAS_PRI
        #define RISCV_BASE    NV_FALCON2_PWR_BASE
    #endif
#elif defined(NVRISCV_IS_ENGINE_minion)
    #if (NVRISCV_HAS_PRI || NVRISCV_HAS_CSB_OVER_PRI)
        #include <dev_falcon_v4.h>
        #include <dev_riscv_pri.h>
        #define FALCON_BASE   NV_FALCON_MINION_BASE
    #endif
    #if NVRISCV_HAS_PRI
        #define NV_FALCON2_MINION_BASE 0xA06400
        #define RISCV_BASE    NV_FALCON2_MINION_BASE
    #endif
#elif defined(NVRISCV_IS_ENGINE_sec)
    #if (NVRISCV_HAS_PRI || NVRISCV_HAS_CSB_OVER_PRI)
        #include <dev_falcon_v4.h>
        #include <dev_riscv_pri.h>
        #define FALCON_BASE   0x840000
    #endif
    #if NVRISCV_HAS_PRI
        #define RISCV_BASE    NV_FALCON2_SEC_BASE
    #endif
#elif defined(NVRISCV_IS_ENGINE_fsp)
    #if (NVRISCV_HAS_PRI || NVRISCV_HAS_CSB_OVER_PRI)
        #include <dev_falcon_v4.h>
        #include <dev_riscv_pri.h>
        #define FALCON_BASE   0x8F0000
    #endif
    #if NVRISCV_HAS_PRI
        #define RISCV_BASE    NV_FALCON2_FSP_BASE
    #endif
#elif defined(NVRISCV_IS_ENGINE_nvdec)
    #if (NVRISCV_HAS_PRI || NVRISCV_HAS_CSB_OVER_PRI)
        #include <dev_falcon_v4.h>
        #include <dev_riscv_pri.h>
        // Use NVDEC0 base
        #define FALCON_BASE   NV_FALCON_NVDEC0_BASE
    #endif
    #if NVRISCV_HAS_PRI
        // Use NVDEC0 base
        #define RISCV_BASE    NV_FALCON2_NVDEC0_BASE
    #endif
#endif

#endif // NVRISCV_PEREGRINE_H
