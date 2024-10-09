/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef CPU__IO_LOCAL_H
#define CPU__IO_LOCAL_H
#define FSP__CPU__IO_LOCAL_H                       1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Late FSP headers */
#include <cpu/mmio-access.h>      // for ioread32, iowrite32, ...

/* NVRISCV conifgured headers */
#include NVRISCV64_MANUAL_ADDRESS_MAP
#include NVRISCV64_MANUAL_LOCAL_IO

/**
 * @file io_local.h
 * @brief Provides functions for accessing Local aperture memory.
 */

/**
 * @brief Read from a Local register using local IO offset
 *
 * @param[in] addr: the address to read from within the aperture
 *
 * @return contents of the register
 */
static inline uint32_t localRead(uint32_t addr)
{
    return ioread32(addr);
}

/**
 * @brief Write to a Local register using local IO offset
 *
 * @param[in] addr: the address to read from within the aperture
 * @param[in] val: the value to write at the address
 *
 * @return none
 */
static inline void localWrite(uint32_t addr, uint32_t val)
{
    iowrite32(addr, val);
}

#endif // CPU__IO_LOCAL_H
