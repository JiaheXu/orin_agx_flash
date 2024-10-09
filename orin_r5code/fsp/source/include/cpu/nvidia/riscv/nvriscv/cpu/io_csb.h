/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2019-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef CPU__IO_CSB_H
#define CPU__IO_CSB_H
#define FSP__CPU__IO_CSB_H                       1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Late FSP headers */
#include <cpu/mmio-access.h>      // for ioread32, iowrite32, ...

/* NVRISCV conifgured headers */
#include NVRISCV64_MANUAL_ADDRESS_MAP

/**
 * @file io_csb.h
 * @brief Provides functions for accessing CSB aperture memory.
 */

/**
 * @brief Read from a CSB register
 *
 * @param[in] addr: the address to read from within the aperture
 *
 * @return contents of the register
 */
static inline uint32_t csbRead(uint32_t addr)
{
    return ioread32_offset(NV_RISCV_AMAP_INTIO_START, addr);
}

/**
 * @brief Write to a CSB register
 *
 * @param[in] addr: the address to read from within the aperture
 * @param[in] val: the value to write at the address
 *
 * @return none
 */
static inline void csbWrite(uint32_t addr, uint32_t val)
{
    iowrite32_offset(NV_RISCV_AMAP_INTIO_START, addr, val);
}

#endif // CPU__IO_CSB_H
