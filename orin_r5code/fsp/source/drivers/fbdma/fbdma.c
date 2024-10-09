/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

/*!
 * @file    fbdma.c
 * @brief   FBDMA driver
 *
 * Used to load data from FB to IMEM/DMEM and vice-versa.
 */

#include <stdint.h>
#include <stddef.h>
#include <misc/nvmisc_drf.h>

#include <fbdma/fbdma.h>
#include <cpu/io_local.h>

//
// MISRA 4.9 RFD: function-like macros
// MISRA 15.1 RFD: Forward gotos
// MISRA 8.7 RFD: Functions with external linkage only referenced in one translation unit
//
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")

#if NVRISCV_FEATURE_SCP
//TODO: scp.h has not yet been ported to libSCP. When it is ported, update this to the proper path.
#include <cpu/scp.h>
#include <misc/ct-assert.h>

#ifndef SCP_REGISTER_SIZE
#error "SCP_REGISTER_SIZE not defined!"
#endif

CT_ASSERT(SCP_REGISTER_SIZE % DMA_BLOCK_SIZE_MIN == 0U,
    "Minimum SCPDMA transfer size is not aligned to minimum FBDMA block size!");
#endif

#if (NVRISCV_IMEM_SIZE == 0U) || (NVRISCV_DMEM_SIZE == 0U)
#error "NVRISCV_IMEM_SIZE and NVRISCV_DMEM_SIZE must be configured"
#endif

static error_t fbdma_wait_completion(void);

/**
 * @brief Create a NV_PRGNLCL_FALCON_DMATRFCMD register value for a dma transfer
 *
 * @param[in] tcm_pa The RISCV PA for the TCM buffer. Must be aligned to DMA_BLOCK_SIZE_MIN,
 * but DMA_BLOCK_SIZE_MAX alignement is optimal.
 * @param[in] aperture_offset The external address (GPU PA or GPU VA) in the aperture
 * determined by dma_idx. The type of memory to access is determined by FBIF_APERTURE_CFG.memType
 * (on engines with FBIF) or by TFBIF_APERTURE_CFG.swid (on engines with TFBIF).
 * memutils_riscv_pa_to_target_offset can be used to translate a RISCV PA to a GPU PA/VA suitable for
 * this argument. Must be aligned to DMA_BLOCK_SIZE_MIN, but DMA_BLOCK_SIZE_MAX alignement
 * is optimal.
 * @param[in] size_bytes The number of bytes to transfer. Must have a granularity of
 * DMA_BLOCK_SIZE_MIN, but DMA_BLOCK_SIZE_MAX granularity is optimal.
 * @param[in] dma_idx Aperture index to use (0-7).
 * @param[in] b_read_ext Transfer direction. true: External->TCM. false: TCM->External
 * @param[out] p_dma_cmd The NV_PRGNLCL_FALCON_DMATRFCMD register value to use for this transfer.
 * @param[out] p_tcm_offset The offset of tcm_pa from the start of its TCM.
 *
 * @return E_SUCCESS on success.
 *         E_INVALID_PARAM if tcm_pa is not a valid TCM address or if size_bytes, tcm_offset,
 * or aperture_offset are not properly aligned.
 */
static error_t
create_dma_command(
    uint64_t tcm_pa,
    uint64_t size_bytes,
    uint8_t  dma_idx,
    bool     b_read_ext,
    uint32_t *p_dma_cmd,
    uint32_t *p_tcm_offset
)
{
    error_t status = E_SUCCESS;
    bool b_tcm_imem = false;
    uint32_t dma_cmd = 0;
    uint32_t tcm_offset = 0;
    uint64_t tcm_buf_end;

    // overflow check for tcm_pa + size_bytes
    if ((UINT64_MAX - tcm_pa) < size_bytes)
    {
        status = E_INVALID_PARAM;
        goto out;
    }
    else
    {
        tcm_buf_end = tcm_pa + size_bytes;
    }

    // tcm_pa is in ITCM or DTCM?
    if ((tcm_pa >= NV_RISCV_AMAP_IMEM_START) &&
        (tcm_buf_end < (NV_RISCV_AMAP_IMEM_START + (uint64_t)NVRISCV_IMEM_SIZE)))
    {
        b_tcm_imem = true;
        tcm_offset = (uint32_t)((tcm_pa - NV_RISCV_AMAP_IMEM_START) & 0xFFFFFFFFU);
    }
    else if ((tcm_pa >= NV_RISCV_AMAP_DMEM_START) &&
        (tcm_buf_end < (NV_RISCV_AMAP_DMEM_START + (uint64_t)NVRISCV_DMEM_SIZE)))
    {
        tcm_offset = (uint32_t)((tcm_pa - NV_RISCV_AMAP_DMEM_START) & 0xFFFFFFFFU);
    }
    else
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    if (b_tcm_imem)
    {
        dma_cmd = FLD_SET_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _IMEM, _TRUE, dma_cmd);
    }
    else
    {
        dma_cmd = FLD_SET_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _IMEM, _FALSE, dma_cmd);
        dma_cmd = FLD_SET_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _SET_DMTAG, _TRUE, dma_cmd);
    }

    if (b_read_ext) // Ext -> TCM
    {
        dma_cmd = FLD_SET_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _WRITE, _FALSE, dma_cmd);
    }
    else // TCM -> Ext
    {
        dma_cmd = FLD_SET_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _WRITE, _TRUE, dma_cmd);
    }

    if (dma_idx <= DMA_MAX_DMAIDX)
    {
        dma_cmd = FLD_SET_DRF_NUM(_PRGNLCL, _FALCON_DMATRFCMD, _CTXDMA, dma_idx, dma_cmd);
    }
    else
    {
        status = E_INVALID_PARAM;
        goto out;
    }

    *p_dma_cmd = dma_cmd;
    *p_tcm_offset = tcm_offset;

out:
    return status;
}

/**
 * @brief Kick off a DMA transfer between TCM PA and External memory
 *
 * @param[in] tcm_offset The offset of the TCM address from the start of that TCM.
 * @param[in] aperture_offset The external address (GPU PA or GPU VA) in the aperture
 *            determined by dma_idx. This can be a GPU VA or GPU PA.
 * @param[in] size_bytes The number of bytes to DMA.
 * @param[in] dma_cmd The value to write into the NV_PRGNLCL_FALCON_DMATRFCMD register,
 * excluding the size field. size will be determined inside this function.
 *
 * @return E_SUCCESS on success.
 *         E_INVALID_PARAM if aperture_offset is not a valid address or size_bytes, tcm_pa,
 *          or aperture_offset do not meet minimum alignment requirements.
 *
 * @pre The FBIF/TFBIF aperture referenced by dma_idx must have been configured by
 * fbifConfigureAperture or tfbifConfigureAperture. If the aperture is configured
 * to use a GPU VA, a context must be bound.
 */
static error_t
fbdma_xfer(
    uint32_t tcm_offset,
    uint64_t aperture_offset,
    uint64_t size_bytes,
    uint32_t dma_cmd
)
{
    error_t err = E_SUCCESS;
    uint32_t dma_block_size;
    uint8_t dma_enc_block_size;
    uint32_t current_tcm_offset = tcm_offset;
    uint64_t bytes_remaining = size_bytes;
    uint32_t dma_cmd_with_block_size = dma_cmd;
    uint32_t fb_offset;

    if (((aperture_offset | size_bytes | tcm_offset) & (DMA_BLOCK_SIZE_MIN - 1U)) != 0U)
    {
        err = E_INVALID_PARAM;
        goto out;
    }

    // When REQUIRE_CTX = true, DMAs without a CTX bound will report idle immediately but won't actually be issued
    localWrite(NV_PRGNLCL_FALCON_DMACTL,
        DRF_DEF(_PRGNLCL, _FALCON_DMACTL, _REQUIRE_CTX, _FALSE));

    //
    // Break up aperture_offset into a base/offset to be used in DMATRFBASE, TRFFBOFFS
    // Note: We cannot use current_tcm_offset for both TRFMOFFS and TRFFBOFFS because this
    // would require us to calculate a DMATRFBASE value by subtracting
    // (aperture_offset - current_tcm_offset) and this may result in a misaligned value
    // which we cannot program into DMATRFBASE.
    //
    fb_offset = (uint32_t) (aperture_offset & 0xFFU);

    // Check for overflows (CERT-C INT30 violation in current_tcm_offset += dma_block_size; fb_offset += dma_block_size;)
    if (((UINT64_MAX - current_tcm_offset) < size_bytes) ||
        ((UINT64_MAX - fb_offset) < size_bytes))
    {
        err = E_INVALID_PARAM;
        goto out;
    }

    localWrite(NV_PRGNLCL_FALCON_DMATRFBASE, (uint32_t)((aperture_offset >> 8U) & 0xffffffffU));
#ifdef NV_PRGNLCL_FALCON_DMATRFBASE1
    localWrite(NV_PRGNLCL_FALCON_DMATRFBASE1, (uint32_t)(((aperture_offset >> 8U) >> 32U) & 0xffffffffU));
#else //NV_PRGNLCL_FALCON_DMATRFBASE1
    if ((((aperture_offset >> 8U) >> 32U) & 0xffffffffU) != 0U)
    {
        err = E_INVALID_PARAM;
        goto out;
    }
#endif //NV_PRGNLCL_FALCON_DMATRFBASE1

    while (bytes_remaining != 0U)
    {
        // Wait if we're full
        while (FLD_TEST_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _FULL, _TRUE, localRead(NV_PRGNLCL_FALCON_DMATRFCMD)))
        {
        }

        localWrite(NV_PRGNLCL_FALCON_DMATRFMOFFS, current_tcm_offset); // This is fine as long as caller does error checking.
        localWrite(NV_PRGNLCL_FALCON_DMATRFFBOFFS, fb_offset);

        // Determine the largest pow2 block transfer size of the remaining data
        dma_block_size = LOWESTBIT(current_tcm_offset | fb_offset | DMA_BLOCK_SIZE_MAX);

        // Reduce the dma_block_size to not exceed remaining data.
        while (dma_block_size > bytes_remaining)
        {
            dma_block_size >>= 1U;
        }

        // Convert the pow2 block size to block size encoding
        dma_enc_block_size = (uint8_t)BIT_IDX_32(dma_block_size / DMA_BLOCK_SIZE_MIN);
        dma_cmd_with_block_size = FLD_SET_DRF_NUM(_PRGNLCL, _FALCON_DMATRFCMD, _SIZE, dma_enc_block_size, dma_cmd);

        localWrite(NV_PRGNLCL_FALCON_DMATRFCMD, dma_cmd_with_block_size);

        bytes_remaining -= dma_block_size;
        current_tcm_offset += dma_block_size;
        fb_offset += dma_block_size;
    }

out:
    return err;
}

error_t
fbdma_pa
(
    uint64_t tcm_pa,
    uint64_t aperture_offset,
    uint64_t size_bytes,
    uint8_t  dma_idx,
    bool     b_read_ext
)
{
    error_t ret;
    uint32_t tcm_offset;
    uint32_t dma_cmd;

    ret = create_dma_command(tcm_pa, size_bytes, dma_idx, b_read_ext, &dma_cmd, &tcm_offset);
    if (ret != E_SUCCESS)
    {
        goto out;
    }

    // Nothing to copy, bail after checks
    if (size_bytes == 0U)
    {
        ret = E_SUCCESS;
        goto out;
    }

    ret = fbdma_xfer(tcm_offset, aperture_offset, size_bytes, dma_cmd);
    if (ret != E_SUCCESS)
    {
        goto out;
    }

    INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
    ret = fbdma_wait_completion();

out:
    return ret;
}

/**
 * @brief Wait for all DMA transfers to complete
 *
 * @return E_SUCCESS on success.
 *         E_FAULT if DMA HW returned an error.
 *         E_BUSY if DMA engine has returned a NACK.
 *
 * @pre Should be called after dmaPaAsync to ensure that DMAs have completed.
 */
error_t
fbdma_wait_completion(void)
{
    error_t err = E_SUCCESS;
    uint32_t dmatrfcmd;
    uint32_t dmainfo;

    // Wait for completion of any remaining transfers
    do {
        dmatrfcmd = localRead(NV_PRGNLCL_FALCON_DMATRFCMD);
    }
    while (FLD_TEST_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _IDLE, _FALSE, dmatrfcmd));

    if (FLD_TEST_DRF(_PRGNLCL, _FALCON_DMATRFCMD, _ERROR, _TRUE, dmatrfcmd))
    {
        err = E_FAULT;
        goto out;
    }

    dmainfo = localRead(NV_PRGNLCL_FALCON_DMAINFO_CTL);
    if (FLD_TEST_DRF(_PRGNLCL, _FALCON_DMAINFO_CTL, _INTR_ERR_COMPLETION, _TRUE, dmainfo))
    {
        // Received DMA NACK, clear it and return error
        dmainfo = FLD_SET_DRF(_PRGNLCL, _FALCON_DMAINFO_CTL, _INTR_ERR_COMPLETION, _CLR, dmainfo);
        localWrite(NV_PRGNLCL_FALCON_DMAINFO_CTL, dmainfo);
        err = E_BUSY;
        goto out;
    }

out:
    return err;
}

#if NVRISCV_FEATURE_SCP || defined(TEST_SCPTOEXTMEM)
error_t
fbdma_scp_to_extmem(
    uint64_t aperture_offset,
    uint64_t size_bytes,
    uint8_t dma_idx
)
{
    error_t ret;

    // Nothing to copy
    if (size_bytes == 0U)
    {
        return E_SUCCESS;
    }

    if ((((size_bytes | aperture_offset) & (SCP_REGISTER_SIZE - 1U)) != 0U) ||
        (dma_idx > DMA_MAX_DMAIDX))
    {
        return E_INVALID_PARAM;
    }

    //
    // Set dma_cmd:
    // - write to Extmem
    // - enable direct bypass shortcut path from SCP
    // - dma_idx
    //
    uint32_t dma_cmd = DRF_DEF(_PRGNLCL, _FALCON_DMATRFCMD, _WRITE, _TRUE) |
                DRF_NUM(_PRGNLCL, _FALCON_DMATRFCMD, _SEC, 2U) |
                DRF_NUM(_PRGNLCL, _FALCON_DMATRFCMD, _CTXDMA, dma_idx);

    //
    // call fbdma_xfer with tcm_offset=0, since this value won't get used anyways:
    // Data is sourced from SCP.
    //
    ret = fbdma_xfer(0U, aperture_offset, size_bytes, dma_cmd);
    if (ret != E_SUCCESS)
    {
        return ret;
    }

    return fbdma_wait_completion();
}
#endif // NVRISCV_FEATURE_SCP

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
