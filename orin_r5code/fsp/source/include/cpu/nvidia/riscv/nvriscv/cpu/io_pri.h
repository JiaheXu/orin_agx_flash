/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef CPU__IO_PRI_H
#define CPU__IO_PRI_H
#define FSP__CPU__IO_PRI_H                       1

/* Compiler headers */
#include <stdint.h>
#if NVRISCV_FEATURE_PRI_CHECKED_IO
#include <stdbool.h>
#endif

/* Early FSP headers */

/* Late FSP headers */
#include <cpu/peregrine.h>
#include <cpu/mmio-access.h>      // for ioread32, iowrite32, ...

/* NVRISCV conifgured headers */
#include NVRISCV64_MANUAL_ADDRESS_MAP

/**
 * @file io_pri.h
 * @brief Provides functions for accessing PRI aperture memory.
 */

/**
 * @brief Read an arbitrary PRI register
 *
 * @param[in] addr: the address to read from within the aperture
 *
 * @return contents of the register
 */
static inline uint32_t priRead(uint32_t addr)
{
    return ioread32_offset(NV_RISCV_AMAP_PRIV_START, addr);
}

/**
 * @brief Write to an arbitrary PRI register
 *
 * @param[in] addr: the address to read from within the aperture
 * @param[in] val: the value to write at the address
 *
 * @return none
 */
static inline void priWrite(uint32_t addr, uint32_t val)
{
    iowrite32_offset(NV_RISCV_AMAP_PRIV_START, addr, val);
}

/**
 * @brief Read from the Peregrine's Falcon block
 *
 * @param[in] addr: the address to read from within the aperture
 *
 * @return contents of the register
 */
static inline uint32_t falconRead(uint32_t offset)
{
    return ioread32_offset(FALCON_BASE, offset);
}

/**
 * @brief Write to the Peregrine's Falcon block
 *
 * @param[in] addr: the address to read from within the aperture
 * @param[in] val: the value to write at the address
 *
 * @return none
 */
static inline void falconWrite(uint32_t offset, uint32_t val)
{
    iowrite32_offset(FALCON_BASE, offset, val);
}

/**
 * @brief Read from the Peregrine's RISC-V block
 *
 * @param[in] addr: the address to read from within the aperture
 *
 * @return contents of the register
 */
static inline uint32_t riscvRead(uint32_t offset)
{
    return ioread32_offset(RISCV_BASE, offset);
}

/**
 * @brief Write to the Peregrine's RISC-V block
 *
 * @param[in] addr: the address to read from within the aperture
 * @param[in] val: the value to write at the address
 *
 * @return none
 */
static inline void riscvWrite(uint32_t offset, uint32_t val)
{
    iowrite32_offset(RISCV_BASE, offset, val);
}

#endif // CPU__IO_PRI_H
