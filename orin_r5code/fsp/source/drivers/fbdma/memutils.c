/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <error/common-errors.h>

#include <fbdma/memutils.h>
#include <cpu/io_local.h>

//
// MISRA 4.9 RFD: function-like macros
// MISRA 15.1 RFD: Forward gotos
// MISRA 8.7 RFD: Functions with external linkage only referenced in one translation unit
//
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")

#if NVRISCV_HAS_FBIF
#include <fbdma/fbif.h>
#endif // NVRISCV_HAS_FBIF

#if NVRISCV_HAS_FBIF
error_t
memutils_riscv_pa_to_fbif_aperture
(
    uintptr_t pa,
    fbif_transcfg_target_t *p_target,
    uint64_t *p_offset
)
{
    riscv_mem_target_t mem_target;
    error_t status;

    if (p_target == NULL)
    {
        return E_INVALID_PARAM;
    }

    // Obtain the memory target and offset for the provided physical address.
    status = memutils_riscv_pa_to_target_offset(pa, &mem_target, p_offset);
    if(status != E_SUCCESS)
    {
        goto out;
    }

    // Translate the memory target to its corresponding FBIF target.
    switch (mem_target)
    {
        case RISCV_MEM_TARGET_FBGPA:
            *p_target = FBIF_TRANSCFG_TARGET_LOCAL_FB;
            break;

        case RISCV_MEM_TARGET_SYSGPA:
            //
            // Just assume coherent here as this value is meant to be
            // passed to dmaPa() anyway.
            //
            *p_target = FBIF_TRANSCFG_TARGET_COHERENT_SYSTEM;
            break;

        default:
            status = E_INVALID_PARAM;
            break;
    }

out:
    return status;
}
#endif // NVRISCV_HAS_FBIF

error_t
memutils_riscv_pa_to_target_offset
(
    uintptr_t pa,
    riscv_mem_target_t *p_target,
    uint64_t *p_offset
)
{
    error_t status = E_SUCCESS;
    riscv_mem_target_t target;
    uint64_t offset;

    // Determine which memory region pa resides in.
    if (memutils_mem_addr_in_range(pa, NV_RISCV_AMAP_IMEM_START, NV_RISCV_AMAP_IMEM_END))
    {
        target = RISCV_MEM_TARGET_IMEM;
        offset = pa - NV_RISCV_AMAP_IMEM_START;
    }
    else if (memutils_mem_addr_in_range(pa, NV_RISCV_AMAP_DMEM_START, NV_RISCV_AMAP_DMEM_END))
    {
        target = RISCV_MEM_TARGET_DMEM;
        offset = pa - NV_RISCV_AMAP_DMEM_START;
    }
#ifdef NV_RISCV_AMAP_FBGPA_START
    else if (memutils_mem_addr_in_range(pa, NV_RISCV_AMAP_FBGPA_START, NV_RISCV_AMAP_FBGPA_END))
    {
        target = RISCV_MEM_TARGET_FBGPA;
        offset = pa - NV_RISCV_AMAP_FBGPA_START;
    }
#endif // NV_RISCV_AMAP_FBGPA_START
#ifdef NV_RISCV_AMAP_SYSGPA_START
    else if (memutils_mem_addr_in_range(pa, NV_RISCV_AMAP_SYSGPA_START, NV_RISCV_AMAP_SYSGPA_END))
    {
        target = RISCV_MEM_TARGET_SYSGPA;
        offset = pa - NV_RISCV_AMAP_SYSGPA_START;
    }
#endif // NV_RISCV_AMAP_SYSGPA_START
    else
    {
        // Not a supported memory region.
        status = E_INVALID_PARAM;
        goto out;
    }

    // Return target/offset information as requested.
    if (p_target != NULL)
    {
        *p_target = target;
    }

    if (p_offset != NULL)
    {
        *p_offset = offset;
    }

out:
    return status;
}

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
