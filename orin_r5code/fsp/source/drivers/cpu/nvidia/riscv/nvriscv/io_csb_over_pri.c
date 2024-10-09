/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2019-2020 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#include NVRISCV64_MANUAL_ADDRESS_MAP
#include <misc/nvmisc_drf.h>
#if NVRISCV_IS_ENGINE_pmu
#include <dev_pwr_csb.h>
#endif
#include <cpu/peregrine.h>
#include <cpu/io.h>

uint32_t csbRead(uint32_t addr)
{
#if NVRISCV_IS_ENGINE_pmu
    if (((uintptr_t)addr <= DEVICE_EXTENT(NV_CPWR_THERM)) &&
            ((uintptr_t)addr >= DEVICE_BASE(NV_CPWR_THERM)))
    {
        return priRead(addr);
    }
    else
#endif
    {
        return priRead((FALCON_BASE +
                        (addr >> NVRISCV_HAS_CSB_OVER_PRI_SHIFT)));
    }
}

void csbWrite(uint32_t addr, uint32_t val)
{
#ifdef NVRISCV_IS_ENGINE_pmu
    if (((uintptr_t)addr <= DEVICE_EXTENT(NV_CPWR_THERM)) &&
        ((uintptr_t)addr >= DEVICE_BASE(NV_CPWR_THERM)))
    {
        priWrite(addr, val);
    }
    else
#endif
    {
        priWrite((FALCON_BASE + (addr >> NVRISCV_HAS_CSB_OVER_PRI_SHIFT)), val);
    }
}
