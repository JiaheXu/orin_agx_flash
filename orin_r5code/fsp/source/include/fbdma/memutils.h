/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#ifndef FBDMA__MEMUTILS_H
#define FBDMA__MEMUTILS_H
#define FSP__FBDMA__MEMUTILS_H 1

#include <stddef.h>
#include <stdint.h>

#include <error/common-errors.h>

#if NVRISCV_HAS_FBIF
#include <fbdma/fbif.h>
#endif // NVRISCV_HAS_FBIF

/**
 * @brief Memory targets supported by memutils_riscv_pa_to_target_offset()
 *
 * @typedef-title riscv_mem_target_t
 */
typedef enum
{
    RISCV_MEM_TARGET_IMEM,
    RISCV_MEM_TARGET_DMEM,
    RISCV_MEM_TARGET_FBGPA,
    RISCV_MEM_TARGET_SYSGPA,
} riscv_mem_target_t;


#if NVRISCV_HAS_FBIF

/**
 * @brief Converts a RISCV PA to an FBIF target/offset pair. Target is one of
 * the external memory types which Peregrine can access via FBIF (FB or SYSMEM)
 * and offset is the offset within the physical address range of that memory type.
 *
 * @param[in]  pa       The RISCV PA to convert.
 *
 * @param[out] p_target  The target aperture (e.g. COHERENT_SYSTEM) that the PA
 *                      resides within.
 *
 * @param[out] p_offset  The offset of the PA within the target aperture.
 *
 * @return E_SUCCESS       if successful.
 * E_INVALID_PARAM if PA is not within the range of a valid memory aperture (SYSMEM or FB).
 */
error_t memutils_riscv_pa_to_fbif_aperture(uintptr_t pa, fbif_transcfg_target_t *p_target,
    uint64_t *p_offset);

#endif // NVRISCV_HAS_FBIF

/**
 * @brief Converts a RISCV PA to a global target/offset pair. Target is one of
 * the available memory types which Peregrine can access (IMEM, DMEM, FB, SYSMEM)
 * and offset is the offset within the physical address range of that memory type.
 *
 * @param[in]  pa       The RISCV PA to convert.
 *
 * @param[out] p_target  The memory target (e.g. SYSMEM) that the PA resides
 *                      within (optional).
 *
 * @param[out] p_offset  The offset of the PA within the memory target
 *                      (optional).
 *
 * @return E_SUCCESS       if successful.
 * E_INVALID_PARAM if PA does not fall inside any of the supported memory targets.
 */
error_t memutils_riscv_pa_to_target_offset(uintptr_t pa, riscv_mem_target_t *p_target,
    uint64_t *p_offset);

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks whether an address is contained within a memory region.
 *
 * @param[in]    check_addr      The address to check.
 * @param[in]    region_start    The start address of the memory region.
 * @param[in]    region_end      The end address of the memory region.
 *
 * @return true if the check address lies within the memory region.
 *         false otherwise.
 *
 * @note The byte at region_end is not included in the memory region.
 */
static inline bool
memutils_mem_addr_in_range
(
    uintptr_t check_addr,
    uintptr_t region_start,
    uintptr_t region_end
)
{
    return ((check_addr >= region_start) && (check_addr < region_end));
}

/**
 * @brief Checks whether two memory regions overlap.
 *
 * @param[in]    first_base      Base address of the first region.
 * @param[in]    first_size      Size in bytes of the first region.
 * @param[in]    second_base     Base address of the second region.
 * @param[in]    second_size     Size in bytes of the second region.
 *
 * @return
 *    true  if the memory regions overlap.
 *    false if the memory regions do not overlap.
 *
 * @note in case of overflow, this function returns false
 */
static inline bool
memutils_mem_has_overlap
(
    uintptr_t first_base,
    size_t first_size,
    uintptr_t second_base,
    size_t second_size
)
{
    if (((UINT64_MAX - first_base) < first_size) ||
        ((UINT64_MAX - second_base) < second_size))
    {
        return false;
    }
    else
    {
        return ((second_base < (first_base  + first_size)) &&
                (first_base  < (second_base + second_size)));
    }
}

#endif //FBDMA__MEMUTILS_H
