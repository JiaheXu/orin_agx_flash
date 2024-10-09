/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2018-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

/*
 * nvgpu-diag - Diagnostics module for debugging FSP on NVRISCV.
 */

/*
 * @file nvgpu-diag.c
 * @brief Diagnostics module for debugging FSP on NVRISCV.
 */
#include <stdbool.h>

#include <stdint.h>
#include <stddef.h>
#include <misc/ct-assert.h>
#include <misc/macros.h>

#include <misc/nvmisc_drf.h>
#include <cpu/cache.h>
#include <cpu/csr.h>
#include <cpu/io.h>
#include <cpu/mmio-access.h>
#include <cpu/riscv-mpu.h>
#include <cpu/fence.h>
#include <cpu/barriers.h>
#include <cpu/sbi.h>
#include <cpu/shutdown.h>

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__MMIO_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__IO_H, "Header file missing or invalid.")
#if NVRISCV_HAS_CSB_MMIO
CT_ASSERT(FSP__CPU__IO_CSB_H, "Header file missing or invalid.")
#endif
CT_ASSERT(FSP__CPU__IO_LOCAL_H, "Header file missing or invalid.")
#if NVRISCV_HAS_PRI
CT_ASSERT(FSP__CPU__IO_PRI_H, "Header file missing or invalid.")
#endif
#if NVRISCV_HAS_DIO_SE || NVRISCV_HAS_DIO_SNIC || NVRISCV_HAS_DIO_FBHUB
CT_ASSERT(FSP__CPU__IO_DIO_H, "Header file missing or invalid.")
#endif
CT_ASSERT(FSP__MISC__NVMISC_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__RISCV_MPU_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__FENCE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__CACHE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__BARRIER_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__SHUTDOWN_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__SBI_H, "Header file missing or invalid.")

#if 0
#pragma message "content of INT_MAX: " STR(INT_MAX)
#pragma message "content of INT32_MAX: " STR(INT32_MAX)
#pragma message "content of UINT_MAX: " STR(UINT_MAX)
#pragma message "content of UINT32_MAX: " STR(UINT32_MAX)
#pragma message "content of LONG_MAX: " STR(LONG_MAX)
#pragma message "content of size_t: " STR(__SIZE_TYPE__)
#pragma message "content of sizeof(uintptr_t): " STR(sizeof(uintptr_t))
#pragma message "content of sizeof(uint32_t): " STR(sizeof(uint32_t))
#pragma message "content of sizeof(unsigned long): " STR(sizeof(unsigned long))
#pragma message "content of __LONG_MAX__: " STR(__LONG_MAX__)
#pragma message "content of UINTPTR_MAX: " STR(UINTPTR_MAX)
#endif

#define BASE_ADDR32	0x12345678U
#define BASE_ADDR64	0x1234567890UL
#define OFFSET_32	0x05050505U

/* Prototypes */
void nvgpu_diag(void);
uint32_t r32(uintptr_t addr);
void     w32(uintptr_t addr, uint32_t data);
uint32_t r32o(uintptr_t addr, uint32_t offset);
void     w32o(uintptr_t addr, uint32_t offset, uint32_t data);
uint64_t r64(uintptr_t addr);
void     w64(uintptr_t addr, uint64_t data);
uint64_t r64o(uintptr_t addr, uint32_t offset);
void     w64o(uintptr_t addr, uint32_t offset, uint64_t data);

/* NVGPU-DIAG implementation */
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void nvgpu_diag(void)
{
    /* Verify cache */
    uintptr_t base = BASE_ADDR64;
    size_t length = 64UL;
    riscv_addr_type_t address_type = RISCV_ADDR_TYPE_PA;

    icache_invalidate_all();
    dcache_invalidate_all();
    dcache_invalidate(&base, length);
    dcache_invalidate_address_type(&base, length, address_type);
    cache_invalidate(&base, length);

    /* Verify that 32-bit MMIO functions are MISRA clean */
    uintptr_t addr32 = BASE_ADDR32;
    uint32_t  offs32 = OFFSET_32;
    uint32_t  data32;
    data32 = r32(addr32);
    w32(addr32, data32);
    data32 = r32o(addr32, offs32);
    w32o(addr32, offs32, data32);

    /* Verify that 64-bit MMIO functions are MISRA clean */
    uintptr_t addr64 = BASE_ADDR64;
    uint64_t data64;
    data64 = r64(addr64);
    w64(addr64, data64);
    data64 = r64o(addr64, offs32);
    w64o(addr64, offs32, data64);

    /* Verify that io driver functions are MISRA clean */
    uint32_t ioAddr32 = BASE_ADDR32;
    uint32_t ioData32;

    ioData32 = localRead(ioAddr32);
    localWrite(ioAddr32, ioData32);
#if (defined(NVRISCV_HAS_CSB_MMIO) && (NVRISCV_HAS_CSB_MMIO==1))
    ioData32 = csbRead(ioAddr32);
    csbWrite(ioAddr32, ioData32);
#endif
#if (defined(NVRISCV_HAS_PRI) && (NVRISCV_HAS_PRI==1))
#if (defined(NVRISCV_HAS_CSB_OVER_PRI) && (NVRISCV_HAS_CSB_OVER_PRI==1))
    ioData32 = csbRead(ioAddr32);
    csbWrite(ioAddr32, ioData32);
#endif
    ioData32 = priRead(ioAddr32);
    priWrite(ioAddr32, ioData32);
    ioData32 = falconRead(ioAddr32);
    falconWrite(ioAddr32, ioData32);
    ioData32 = riscvRead(ioAddr32);
    riscvWrite(ioAddr32, ioData32);
#endif

    /* Verify that io dio driver functions are MISRA clean */
#if NVRISCV_HAS_DIO_SE
    uint32_t io_dio_se_addr32 = BASE_ADDR32;
    uint32_t io_dio_se_data32 = 0U;
    if (dioReadWrite(DIO_TYPE_SE, DIO_OPERATION_RD, io_dio_se_addr32, &io_dio_se_data32) != (E_SUCCESS)) {
        io_dio_se_data32 = 0U;
    }
#endif

#if NVRISCV_HAS_DIO_SNIC
    uint32_t io_dio_snic_addr32 = BASE_ADDR32;
    uint32_t io_dio_snic_data32 = 0U;
    if (dioReadWrite(DIO_TYPE_SNIC, DIO_OPERATION_RD, io_dio_snic_addr32, &io_dio_snic_data32) != (E_SUCCESS)) {
        io_dio_snic_data32 = 0U;
    }
#endif

#if NVRISCV_HAS_DIO_FBHUB
    uint32_t io_dio_fbhub_addr32 = BASE_ADDR32;
    uint32_t io_dio_fbhub_data32 = 0U;
    if (dioReadWrite(DIO_TYPE_FBHUB, DIO_OPERATION_RD, io_dio_fbhub_addr32, &io_dio_fbhub_data32) != (E_SUCCESS)) {
        io_dio_fbhub_data32 = 0U;
    }
#endif

    /* Verify that FENCE wrappers are MISRA clean */
    riscvLwfenceIO();
    riscvLwfenceRW();
    riscvLwfenceRWIO();
    riscvFenceIO();
    riscvFenceRW();
    riscvFenceRWIO();
    riscvSfenceVMA(0xFF,0xFF);

    /* Verify that barrier functions are MISRA clean */
    barrier_cache_op_complete();
    barrier_memory_order();
    barrier_memory_complete();
    barrier_instruction_synchronization();

    /* Verify that shutdown helper functions are MISRA clean */
    riscv_panic();
    riscv_shutdown();
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint32_t r32(uintptr_t addr)
{
    return ioread32(addr);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint32_t r32o(uintptr_t addr, uint32_t offset)
{
    return ioread32_offset(addr, offset);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void w32(uintptr_t addr, uint32_t data)
{
    iowrite32(addr, data);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void w32o(uintptr_t addr, uint32_t offset, uint32_t data)
{
    iowrite32_offset(addr, offset, data);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint64_t r64(uintptr_t addr)
{
    return ioread64(addr);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint64_t r64o(uintptr_t addr, uint32_t offset)
{
    return ioread64_offset(addr, offset);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void w64(uintptr_t addr, uint64_t data)
{
    iowrite64(addr, data);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void w64o(uintptr_t addr, uint32_t offset, uint64_t data)
{
    iowrite64_offset(addr, offset, data);
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
