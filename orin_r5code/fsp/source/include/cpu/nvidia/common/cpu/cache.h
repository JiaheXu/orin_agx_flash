/**
 * @file cache.h
 *
 * @brief Cache driver for NVRISC-V dev
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPU__CACHE_H
#define CPU__CACHE_H
#define FSP__CPU__CACHE_H                       1

/* Compiler headers */
#include <stddef.h>
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Late FSP headers */
#include <misc/attributes.h>               // for UNUSED_NONCONST_PTR
#include <misc/bitops.h>                   // for BIT, EXTRACT, FSP__MISC__B...
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <misc/nvmisc_drf.h>               // for DRF macros
#include <cpu/type-conversion.h>           // for fsp_c_v_ptr_to_uptr, FSP...
#include <cpu/csr.h>                       // for CSR macros

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVMISC_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__CSR_H, "Header file missing or invalid.")

/**
 * @file cache.h
 * @brief Interfaces for performing Cache operations.
 */

/**
 * @brief Cache line size
 */
#define RISCV_CACHE_LINE_SIZE    64UL

/**
 * @brief Type to specify the kind of address to use
 */
typedef enum
{
	RISCV_ADDR_TYPE_VA = 0,
	RISCV_ADDR_TYPE_PA = 1,
} riscv_addr_type_t;

/**
 * @brief Disable all caches
 *
 * This function will disable the iCache and dCache.  It will also ensure
 * before this function returns.
 *
 * @note
 * - M-mode only.
 * - Should we have to do an explicit check or assert for M-mode.
 * - Currently not implemented because there was no previous implementation in
 *   libnvriscv.
 */
static inline void
cache_disable(void) {}

/**
 * @brief Enable all caches
 *
 * This function will enable the iCache and dCache.  It will ensure that
 * the contents of the caches are invalid (e.g. there is no data or instructions
 * in the cache) before enabling the caches.
 *
 * @note
 * - M-mode only.
 * - Should we have to do an explicit check or assert for M-mode.
 * - Currently not implemented because there was no previous implementation in
 *   libnvriscv.
 */
static inline void
cache_enable(void) {}

/**
 * @brief Invalidate the entire iCache
 *
 * This function will cause the entire icache contents to be invalidated.
 */
static inline void
icache_invalidate_all(void)
{
    __asm__ volatile("fence.i");
}

/**
 * @brief Invalidate a region of dCache
 *
 * This function will cause the specified region in the dCache to be
 * invalidated.  That is, that data will no longer reside in the dCache
 * at the completion of this function.
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be invalidated than was actually specified.
 *
 * @param[in] base      Address of start of range to be invalidated
 * @param[in] length    Number of bytes in the range to be invalidated
 */
static inline void
dcache_invalidate(const void * const base, const size_t length)
{
    uintptr_t start;
    uintptr_t end;

    start = fsp_c_v_ptr_to_uptr(base);
    end = start + ((uintptr_t)length);

    start = (start & DRF_SHIFTMASK64(NV_RISCV_CSR_XDCACHEOP_ADDR)) |
                DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _ADDR_MODE, _VA) |
                DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _MODE, _INV_LINE);

    for (;start < end; start += RISCV_CACHE_LINE_SIZE)
    {
        __asm__ volatile("csrrw x0, %0, %1" : :"i"(NV_RISCV_CSR_XDCACHEOP), "r"(start));
    }
}

/**
 * @brief Invalidate region of dCache using either PA or VA
 *
 * This function will cause the specified region in the dCache to be
 * invalidated.  That is, that data will no longer reside in the dCache
 * at the completion of this function.
 *
 * The addresses to invalidate may be specified to be either a
 * virtual address or a physical address.
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be invalidated than was actually specified.
 *
 * @param[in] base              Address of start of range to be invalidated
 * @param[in] length            Number of bytes in the range to be invalidated
 * @param[out] address_type     Type of address being passed as argument
 */
static inline void
dcache_invalidate_address_type(const void * const base, const size_t length, riscv_addr_type_t address_type)
{
    uintptr_t start;
    uintptr_t end;

    start = fsp_c_v_ptr_to_uptr(base);
    end = start + ((uintptr_t)length);

    start = (start & DRF_SHIFTMASK64(NV_RISCV_CSR_XDCACHEOP_ADDR)) |
                DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _MODE, _INV_LINE);

    // Set address mode depending address_type
    if ( address_type == RISCV_ADDR_TYPE_VA)
    {
        start |= DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _ADDR_MODE, _VA);
    }
    else if ( address_type == RISCV_ADDR_TYPE_PA )
    {
        start |= DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _ADDR_MODE, _PA);
    }
    else
    {
        // Do nothing.
    }

    for (;start < end; start += RISCV_CACHE_LINE_SIZE)
    {
        __asm__ volatile("csrrw x0, %0, %1" : :"i"(NV_RISCV_CSR_XDCACHEOP), "r"(start));
    }
}

/**
 * @brief Invalidate entire dCache
 *
 * This function will cause the entire dCache to be invalidated
 */
static inline void
dcache_invalidate_all(void)
{
    __asm__ volatile("csrrw x0, %0, %1" : :"i"(NV_RISCV_CSR_XDCACHEOP), "r"(0));
}

/**
 * @brief Invalidate a region of dCache
 *
 * This function will cause the specified region in the dCache to be
 * invalidated.  That is, that data will no longer reside in the dCache
 * at the completion of this function.
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be invalidated than was actually specified.
 *
 * @param[in] base      Address of start of range to be invalidated
 * @param[in] length    Number of bytes in the range to be invalidated
 *
 * @deprecated dcache_invalidate() should be used instead.
 */
static inline void
cache_invalidate(const void * const base, const size_t length)
{
    uintptr_t start;
    uintptr_t end;

    start = fsp_c_v_ptr_to_uptr(base);
    end = start + ((uintptr_t)length);

    start = (start & DRF_SHIFTMASK64(NV_RISCV_CSR_XDCACHEOP_ADDR)) |
                DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _ADDR_MODE, _VA) |
                DRF_DEF64(_RISCV_CSR, _XDCACHEOP, _MODE, _INV_LINE);

    for (;start < end; start += RISCV_CACHE_LINE_SIZE)
    {
        __asm__ volatile("csrrw x0, %0, %1" : :"i"(NV_RISCV_CSR_XDCACHEOP), "r"(start));
    }
}

/**
 * @brief Clean a region of dCache
 *
 * This function will cause any "dirty" data in the dCache in the
 * specified range to be written back to memory.  The data may still
 * reside in the dCache (e.g. it was not invalidated).
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be "cleaned" than was actually specified.
 *
 * @param[in] base      Address of start of range to be cleaned
 * @param[in] length    Number of bytes in the range to be cleaned
 *
 * @deprecated dcache_clean() should be used instead.
 */
#define cache_clean(base, length) ({ })

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif // FSP__CPU__CACHE_H
