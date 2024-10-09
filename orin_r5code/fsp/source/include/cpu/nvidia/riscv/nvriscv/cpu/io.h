/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2019-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef CPU__IO_H
#define CPU__IO_H
#define FSP__CPU__IO_H                       1

/**
 * @file io.h
 * @brief Provides access to Peregrine MMIO registers.
 */

#include <stdint.h>

#if NVRISCV_HAS_PRI
#include <cpu/io_pri.h>
#endif

#if NVRISCV_HAS_CSB_MMIO
#include <cpu/io_csb.h>
#elif NVRISCV_HAS_PRI && NVRISCV_HAS_CSB_OVER_PRI
uint32_t csbRead(uint32_t addr);
void csbWrite(uint32_t addr, uint32_t val);
#endif

#include <cpu/io_local.h>

#if NVRISCV_HAS_DIO_SE || NVRISCV_HAS_DIO_SNIC || NVRISCV_HAS_DIO_FBHUB
#include <cpu/io_dio.h>
#endif

#endif // CPU__IO_H
