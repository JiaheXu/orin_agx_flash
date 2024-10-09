/*
 * Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPU__RISCV_MPU_H
#define CPU__RISCV_MPU_H
#define FSP__CPU__RISCV_MPU_H                    1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <error/common-errors.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>
#include <misc/macros.h>
#include <cpu/csr.h>
// clients must provide their own DRF headers

/* Module-specific FSP headers */

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @file riscv-mpu.h
 * @brief Defines for the register layout of the MPU
 */

//
// maximum number of entries provided by hardware.
// The actual number of entries available in S mode may be smaller
//
//

START_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief This is the number of HW MPU entries.
 */
#define NV_RISCV_CSR_MPU_ENTRY_COUNT        (1U << DRF_SIZE(NV_RISCV_CSR_SMPUIDX_INDEX))

/**
 * @brief This is the index of the last HW MPU entry.
 */
#define NV_RISCV_CSR_MPU_ENTRY_MAX          (NV_RISCV_CSR_MPU_ENTRY_COUNT - 1U)

/**
 * @brief This is the page size which all to which all MPU Mappings' virtual address,
 * physical address and size must be aligned.
 */
#define NV_RISCV_CSR_MPU_PAGE_SIZE          (1ULL << DRF_SHIFT64(NV_RISCV_CSR_SMPUPA_BASE))

/**
 * @brief This can be used to mask the lower bits of an address, which are an offset
 * within the MPU page.
 */
#define NV_RISCV_CSR_MPU_PAGE_MASK          ((uint64_t)NV_RISCV_CSR_MPU_PAGE_SIZE - 1ULL)

/**
 * @brief This can be used to mask the upper bits of an address, which constitute an
 * MPU page.
 */
#define NV_RISCV_CSR_MPU_ADDRESS_MASK       DRF_SHIFTMASK64(NV_RISCV_CSR_SMPUPA_BASE)

END_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief A handle to an MPU Entry. After being reserved by the MPU driver, the client
 * can pass this handle into other MPU driver APIs to perform operations on this MPU entry.
 */
typedef uint32_t mpu_handle_t;

/**
 * @brief Context used by the MPU driver. The client should treat this as an opaque object
 * and pass it into MPU driver APIs without directly touching its contents.
 *
 * ``mpu_entry_count``: The number of MPU entries owned by this S-mode partition. This is an
 * internal field which should not be used by the client.
 *
 * ``mpu_reserved_bitmap``: Used by the driver to track the MPU Entry allocations. This is an
 * internal field which should not be used by the client.
 */
typedef struct {
    uint8_t mpu_entry_count;
    bitmap_element_t mpu_reserved_bitmap[NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE];
} mpu_context_t;

/**
 * @brief Initializes the MPU driver. This function should be called before any other MPU
 * driver APIs to initialize the MPU driver. Some MPU mappings may have been created before
 * the driver initializes (eg: Bootloader), and this function will synchronize the driver's
 * software state with the hardware state.
 *
 * This function should only be called once for each partition, and there should be only
 * a single context for each partition. The caller should not allocate/use multiple contexts.
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller.
 *      mpu_init will populate it and the caller will pass it into subsequent MPU driver calls.
 * @return E_SUCCESS on success
 *         E_INVALID_PARAM if p_ctx is NULL
 */
error_t mpu_init(mpu_context_t *p_ctx);

/**
 * @brief Enables the MPU. This function should be called in use cases where the MPU hasn't
 * already been enabled by the bootloader.
 *
 * @pre The client must call mpu_init and set up some MPU mappings for
 * code/data prior to calling this. At the very least, the client should set up an identity
 * mapping for the code region calling mpu_enable, to allow code to continue executing after
 * the enablement.
 *
 * @return E_SUCCESS on success
 */
error_t mpu_enable(void);

/**
 * @brief Reserve an MPU entry.
 *
 * @pre The client must call mpu_init before calling this with the same mpu_context_t.
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] search_origin is the lowest MPU index that can be reserved.
 * Valid range: 0 to NV_RISCV_CSR_MPU_ENTRY_MAX
 * @param[out] p_reserved_handle is a pointer to an mpu_handle_t where the reserved MPU Entry will be stored.
 * @return E_SUCCESS if allcation succeeds
 *         E_INVALID_PARAM if p_ctx or p_reserved_handle is NULL
 *         E_FAULT if there are no free entries remaining
 *
 */
error_t mpu_reserve_entry(mpu_context_t *p_ctx, uint32_t search_origin, mpu_handle_t *p_reserved_handle);

/**
 * @brief Free an MPU entry.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry.
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle is a HW MPU entry. It has been previously reserved by mpu_reserve_entry.
 * @return E_SUCCESS if freeing succeeds
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not an mpu_handle_t which has been reserved
 * via mpu_reserve_entry and not already freed with a call to mpu_free_entry.
 *
 */
error_t mpu_free_entry(mpu_context_t *p_ctx, mpu_handle_t handle);

/**
 * @brief Program a mapping into an MPU entry. pa and va and rng contain a
 * NV_RISCV_CSR_MPU_PAGE_SIZE-aligned value in bits 10-63. The lower bits contain WPRIs and Valid bits.
 * See the register manual for details.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry.
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to write. It has been previously reserved by mpu_reserve_entry.
 * @param[in] va is the virtual address of the mapping, written directly into NV_RISCV_CSR_SMPUVA.
 * Valid range: Any value. Bits 10-63 are the NV_RISCV_CSR_MPU_PAGE_SIZE-aligned address and the rest of
 * the bits are flags in NV_RISCV_CSR_SMPUVA. Typically a client will use the NV_RISCV_CSR_SMPUVA_VLD bit
 * to enable the entry.
 * @param[in] pa is the physical address of the mapping, written directly into NV_RISCV_CSR_SMPUPA.
 * Valid range: Any value. Bits 10-63 are the NV_RISCV_CSR_MPU_PAGE_SIZE-aligned address and the rest of
 * the bits are ignored by the hardware.
 * @param[in] rng is the range (length) of the mapping in bytes.
 * Valid range: Any value. Bits 10-63 are the NV_RISCV_CSR_MPU_PAGE_SIZE-aligned size and the rest of
 * the bits are ignored by the hardware.
 * @param[in] attr are the attribute flags of the mapping, written directly into NV_RISCV_CSR_SMPUATTR.
 * Valid range: Any value. Should be a combination of the NV_RISCV_CSR_SMPUATTR_x flags.
 * @return E_SUCCESS if writing succeeds
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not a valid reserved handle
 */
error_t mpu_write_entry(const mpu_context_t *p_ctx, mpu_handle_t handle, uint64_t va, uint64_t pa, uint64_t rng, uint64_t attr);

/**
 * @brief Read a mapping from an MPU entry. Any of the pointer args may be set to NULL if the caller does not care about reading that value.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry.
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to read. It has been previously reserved by mpu_reserve_entry.
 * @param[out] p_va is a pointer to a variable where the va value will be written.
 *      This value is ORd with the enable bit (NV_RISCV_CSR_SMPUVA_VLD)
 * @param[out] p_pa is a pointer to a variable where the pa value will be written
 * @param[out] p_rng is a pointer to a variable where the rng value will be written
 * @param[out] p_attr is a pointer to a variable where the attr value will be written
 * @return E_SUCCESS if reading succeeds
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not a valid reserved handle
 */
error_t mpu_read_entry(const mpu_context_t *p_ctx, mpu_handle_t handle, uint64_t *p_va, uint64_t *p_pa, uint64_t *p_rng, uint64_t *p_attr);

/**
 * @brief Enable an MPU entry. The client does not need to call this if it has already
 * called mpu_write_entry with the SMPUVA_VLD bit set. This API is mostly intended for
 * re-enabling mappings which have been previously disabled via mpu_disable_entry.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry and written it with mpu_write_entry
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to enable. It has been previously reserved by mpu_reserve_entry
 *         and written by mpu_write_entry
 * @return E_SUCCESS if enablement succeeds
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not a valid reserved handle
 */
error_t mpu_enable_entry(const mpu_context_t *p_ctx, mpu_handle_t handle);

/**
 * @brief Disable an MPU entry.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry and written it with mpu_write_entry
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to disable. It has been previously reserved by mpu_reserve_entry,
 *         and enabled with mpu_write_entry or mpu_enable_entry
 * @return E_SUCCESS if disablement succeeds
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not a valid reserved handle
 */
error_t mpu_disable_entry(const mpu_context_t *p_ctx, mpu_handle_t handle);

/**
 * @brief Check whether the region mapped by an MPU entry has been accessed (read or executed)
 * since the last time the Accessed bit was cleared.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to check
 * @param[out] b_accessed a pointer where the function will store whether the entry was accessed:
 *         true if this region was read or executed since the last call to mpu_clear_accessed_bit
 *         false if this region was not read or executed since the last call to mpu_clear_accessed_bit
 * @return E_SUCCESS if access check succeeded
 *         E_INVALID_PARAM if p_ctx or b_accessed is NULL or handle is not a valid reserved handle
 */
error_t mpu_is_accessed(const mpu_context_t *p_ctx, mpu_handle_t handle, bool *b_accessed);

/**
 * @brief Check whether the region mapped by an MPU entry has been written since the last time the
 * Dirty bit was cleared.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to check
 * @param[out] b_dirty a pointer where the function will store whether the entry is dirty:
 *         true if this region was written since the last call to mpu_clear_dirty_bit
 *         false if this region was written since the last call to mpu_clear_dirty_bit
 * @return E_SUCCESS if dirty check succeeded
 *         E_INVALID_PARAM if p_ctx or b_dirty is NULL or handle is not a valid reserved handle
 */
error_t mpu_is_dirty(const mpu_context_t *p_ctx, mpu_handle_t handle, bool *b_dirty);

/**
 * @brief Clear the 'accessed' status of an MPU entry.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to clear
 * @return E_SUCCESS if dirty check succeeded
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not a valid reserved handle
 */
error_t mpu_clear_accessed_bit(const mpu_context_t *p_ctx, mpu_handle_t handle);

/**
 * @brief Clear the 'dirty' status of an MPU entry.
 *
 * @pre The client must have previously reserved this handle with mpu_reserve_entry
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] handle indicates which entry to clear
 * @return E_SUCCESS if dirty check succeeded
 *         E_INVALID_PARAM if p_ctx is NULL or handle is not a valid reserved handle
 */
error_t mpu_clear_dirty_bit(const mpu_context_t *p_ctx, mpu_handle_t handle);

/**
 * @brief Translate a VA to a PA in software using the currently programmed MPU mappings. The translation
 * is done by reading back the register values which have been programmed into the hardware. This may be
 * an expensive operation.
 *
 * @pre The client must have previously reserved called mpu_init and created some entries.
 *
 * @param[in] p_ctx A context struct which has been allocated by the caller and initialized by mpu_init.
 * @param[in] va is the virtual address to translate
 * @param[out] p_pa is a pointer where the corresponding physical address will be stored
 * @param[in] b_only_enabled if this is True, only the enabled MPU entries are searched,
 *      otherwise, all reserved entries are searched
 * @return E_SUCCESS if dirty check succeeded
 *         E_INVALID_PARAM if p_ctx or p_pa is NULL or translation failed
 */
error_t mpu_va_to_pa(const mpu_context_t *p_ctx, uint64_t va, bool b_only_enabled, uint64_t *p_pa);

#endif // CPU__RISCV_MPU_H
