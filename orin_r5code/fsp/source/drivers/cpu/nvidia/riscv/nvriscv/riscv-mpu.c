/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */



#include <stddef.h>
#include <misc/bitops.h>            // for BIT64, bitmap ops
#include <libc-lite/libc.h>         // for memset
#include <misc/nvmisc_drf.h>        // before riscv-mpu.h
#include <cpu/riscv-mpu.h>
#include <cpu/csr.h>

//
// MISRA 1.2 RFD: Inline assembly
// MISRA 4.9 RFD: function-like macros
// MISRA 15.1 RFD: Forward gotos
// MISRA 8.7 RFD: Functions with external linkage only referenced in one translation unit
//
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")

//
// each value of SMPUIDX allows us to access this many MPU entries via
// NV_RISCV_CSR_SMPUACC and NV_RISCV_CSR_SMPUDTY
//
#define MPUIDX2_ENTRIES_PER_INDEX DRF_SIZE(NV_RISCV_CSR_SMPUACC_ACC)

/**
 * @brief Select a HW MPU Entry. subsequent accesses to SMPUVA, SMPUPA,
 * SMPURNG, and SMPUATTR will access the selected entry.
 *
 * @param[in]  index index of the HW MPU entry to select.
 *
 * @return None
 */
static void mpu_idx_select(uint64_t index)
{
    csr_write(NV_RISCV_CSR_SMPUIDX,
              DRF_NUM64(_RISCV, _CSR_SMPUIDX, _INDEX, index));
}

/**
 * @brief Select a HW MPU Entry block. subsequent accesses to SMPUACC
 * and SMPUDTY will access the selected entry.
 *
 * @param[in]  index start index of the HW MPU entry block to select.
 *
 * @return None
 */
static void mpu_idx2_select(uint8_t index)
{
    uint8_t reg = index / MPUIDX2_ENTRIES_PER_INDEX;

    csr_write(NV_RISCV_CSR_SMPUIDX2, DRF_NUM64(_RISCV, _CSR_SMPUIDX2, _IDX, reg));
}

error_t
mpu_init(mpu_context_t *p_ctx)
{
    error_t status = E_SUCCESS;
    uint64_t smpuctl;
    uint8_t idx;

    if (p_ctx == NULL)
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    (void) memset(p_ctx, 0, sizeof(mpu_context_t));

    //
    // Read the entry count. The number of entries could be
    // limited by M-mode code if some entries are used by another partition
    //
    smpuctl = csr_read(NV_RISCV_CSR_SMPUCTL);
    p_ctx->mpu_entry_count = (uint8_t) DRF_VAL64(_RISCV_CSR, _SMPUCTL, _ENTRY_COUNT, smpuctl);

    // Build entries for existing MPU settings
    for (idx = 0; idx < p_ctx->mpu_entry_count; idx++)
    {
        // Check if the MPU entry has already been set
        mpu_idx_select(idx);
        uint64_t virt_addr = csr_read(NV_RISCV_CSR_SMPUVA);
        if (FLD_TEST_DRF_NUM64(_RISCV_CSR, _SMPUVA, _VLD, 1U, virt_addr))
        {
            bitmap_set_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, idx);
        }
    }

out:
    return status;
}

error_t
mpu_enable(void)
{
    csr_write(NV_RISCV_CSR_SATP,
        DRF_DEF64(_RISCV_CSR, _SATP, _MODE, _NVMPU) |
        DRF_DEF64(_RISCV_CSR, _SATP, _ASID, _BARE)  |
        DRF_DEF64(_RISCV_CSR, _SATP, _PPN, _BARE));
    return E_SUCCESS;
}

error_t
mpu_reserve_entry(mpu_context_t *p_ctx, uint32_t search_origin, mpu_handle_t *p_reserved_handle)
{
    error_t status = E_FAULT;
    uint32_t idx;

    if ((p_ctx == NULL) || (p_reserved_handle == NULL))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    for (idx = search_origin; idx < p_ctx->mpu_entry_count; idx++)
    {
        if (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, idx))
        {
            bitmap_set_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, idx);
            *p_reserved_handle = idx;
            status = E_SUCCESS;
            goto out;
        }
    }

out:
    return status;
}

error_t
mpu_free_entry(mpu_context_t *p_ctx, mpu_handle_t handle)
{
    error_t status = E_SUCCESS;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    bitmap_clear_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle);

out:
    return status;
}

error_t
mpu_write_entry
(
    const mpu_context_t *p_ctx,
    mpu_handle_t handle,
    uint64_t va,
    uint64_t pa,
    uint64_t rng,
    uint64_t attr
)
{
    error_t status = E_SUCCESS;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx_select(handle);
    csr_clear(NV_RISCV_CSR_SMPUVA, DRF_NUM64(_RISCV_CSR, _SMPUVA, _VLD, 1));
    csr_write(NV_RISCV_CSR_SMPUPA, pa);
    csr_write(NV_RISCV_CSR_SMPURNG, rng);
    csr_write(NV_RISCV_CSR_SMPUATTR, attr);
    csr_write(NV_RISCV_CSR_SMPUVA, va);

out:
    return status;
}

error_t
mpu_read_entry
(
    const mpu_context_t *p_ctx,
    mpu_handle_t handle,
    uint64_t *p_va,
    uint64_t *p_pa,
    uint64_t *p_rng,
    uint64_t *p_attr
)
{
    error_t status = E_SUCCESS;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx_select(handle);
    if (p_va != NULL)
    {
        *p_va = csr_read(NV_RISCV_CSR_SMPUVA);
    }

    if (p_pa != NULL)
    {
        *p_pa = csr_read(NV_RISCV_CSR_SMPUPA);
    }

    if (p_rng != NULL)
    {
        *p_rng = csr_read(NV_RISCV_CSR_SMPURNG);
    }

    if (p_attr != NULL)
    {
        *p_attr = csr_read(NV_RISCV_CSR_SMPUATTR);
    }

out:
    return status;
}

error_t
mpu_enable_entry(const mpu_context_t *p_ctx, mpu_handle_t handle)
{
    error_t status = E_SUCCESS;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx_select(handle);
    csr_set(NV_RISCV_CSR_SMPUVA, DRF_NUM64(_RISCV_CSR, _SMPUVA, _VLD, 1));

out:
    return status;
}

error_t
mpu_disable_entry(const mpu_context_t *p_ctx, mpu_handle_t handle)
{
    error_t status = E_SUCCESS;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx_select(handle);
    csr_clear(NV_RISCV_CSR_SMPUVA, DRF_NUM64(_RISCV_CSR, _SMPUVA, _VLD, 1));

out:
    return status;
}

error_t
mpu_is_accessed(const mpu_context_t *p_ctx, mpu_handle_t handle, bool *b_accessed)
{
    error_t status = E_SUCCESS;
    uint64_t smpuacc;
    uint8_t idx_to_test;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)) ||
        (b_accessed == NULL))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx2_select((uint8_t)handle);
    smpuacc = csr_read(NV_RISCV_CSR_SMPUACC);
    idx_to_test = (uint8_t) handle % MPUIDX2_ENTRIES_PER_INDEX;
    *b_accessed = ((smpuacc & BIT64(idx_to_test)) != 0U);

out:
    return status;
}

error_t
mpu_is_dirty(const mpu_context_t *p_ctx, mpu_handle_t handle, bool *b_dirty)
{
    error_t status = E_SUCCESS;
    uint64_t smpudty;
    uint8_t idx_to_test;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)) ||
        (b_dirty == NULL))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx2_select((uint8_t)handle);
    smpudty = csr_read(NV_RISCV_CSR_SMPUDTY);
    idx_to_test = (uint8_t) handle % MPUIDX2_ENTRIES_PER_INDEX;
    *b_dirty = ((smpudty & BIT64(idx_to_test)) != 0U);

out:
    return status;
}

error_t
mpu_clear_accessed_bit(const mpu_context_t *p_ctx, mpu_handle_t handle)
{
    error_t status = E_SUCCESS;
    uint8_t idx_to_clear;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx2_select((uint8_t)handle);
    idx_to_clear = (uint8_t) handle % MPUIDX2_ENTRIES_PER_INDEX;
    csr_clear(NV_RISCV_CSR_SMPUACC, BIT64(idx_to_clear));

out:
    return status;
}

error_t
mpu_clear_dirty_bit(const mpu_context_t *p_ctx, mpu_handle_t handle)
{
    error_t status = E_SUCCESS;
    uint8_t idx_to_clear;

    if ((p_ctx == NULL) ||
        (handle >= p_ctx->mpu_entry_count) ||
        (!bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, handle)))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    mpu_idx2_select((uint8_t)handle);
    idx_to_clear = (uint8_t) handle % MPUIDX2_ENTRIES_PER_INDEX;
    csr_clear(NV_RISCV_CSR_SMPUDTY, BIT64(idx_to_clear));

out:
    return status;
}

error_t
mpu_va_to_pa(const mpu_context_t *p_ctx, uint64_t va, bool b_only_enabled, uint64_t *p_pa)
{
    error_t status = E_INVALID_PARAM;
    uint8_t idx;

    if ((p_ctx == NULL) || (p_pa == NULL))
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    for (idx = 0; idx < p_ctx->mpu_entry_count; idx++)
    {
        if (bitmap_test_bit(p_ctx->mpu_reserved_bitmap, NV_RISCV_CSR_MPU_ENTRY_COUNT / BITMAP_ELEMENT_SIZE, idx))
        {
            mpu_idx_select(idx);
            uint64_t va_reg = csr_read(NV_RISCV_CSR_SMPUVA);
            uint64_t va_base = va_reg & ~NV_RISCV_CSR_MPU_PAGE_MASK;
            uint64_t rng;

            //If entry is disabled or va is below range, don't check this entry
            if ((b_only_enabled && FLD_TEST_DRF_NUM64(_RISCV_CSR, _SMPUVA, _VLD, 0U, va_reg)) ||
                (va_base > va))
            {
                continue;
            }

            rng = csr_read(NV_RISCV_CSR_SMPURNG) & ~NV_RISCV_CSR_MPU_PAGE_MASK;

            // check for overflow in va_base + rng (CERT-C). Shoudln't happen in a sanely-programmed entry.
            if ((UINT64_MAX - va_base) < rng)
            {
                continue;
            }

            if (va < (va_base + rng))
            {
                //Do a manual translation
                uint64_t pa_base = csr_read(NV_RISCV_CSR_SMPUPA) & ~NV_RISCV_CSR_MPU_PAGE_MASK;

                // check for overflow in (va - va_base) + pa_base; (CERT-C). Shoudln't happen in a sanely-programmed entry.
                if ((UINT64_MAX - pa_base) < (va - va_base))
                {
                    continue;
                }

                *p_pa = (va - va_base) + pa_base;
                status = E_SUCCESS;
                break;
            }
        }
    }

out:
    return status;
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
