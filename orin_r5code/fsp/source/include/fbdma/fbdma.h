/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#ifndef FBDMA__FBDMA_H
#define FBDMA__FBDMA_H
#define FSP__FBDMA__FBDMA_H 1

#include <stdint.h>
#include <stdbool.h>
#include <error/common-errors.h>

#if (!NVRISCV_HAS_FBDMA) || (!NVRISCV_FEATURE_DMA)
#error "This header cannot be used on an engine which does not support FBDMA."
#endif // (!NVRISCV_HAS_FBDMA) || (!NVRISCV_FEATURE_DMA)

#if NVRISCV_HAS_FBIF
#include <fbdma/fbif.h>
/**
 * @brief The highest allowed value for the dma_idx argument.
 */
#define DMA_MAX_DMAIDX (FBIF_NUM_APERTURES-1U)

#elif NVRISCV_HAS_TFBIF
#include <fbdma/tfbif.h>
/**
 * @brief The highest allowed value for the dma_idx argument.
 */
#define DMA_MAX_DMAIDX (TFBIF_NUM_APERTURES-1U)

#else
#error "No FBIF or TFBIF"
#endif

/**
 * @brief The minimum block size which FBDMA can transfer. All addresses
 * and transfer sizes must be aligned to this.
 */
#define DMA_BLOCK_SIZE_MIN   0x4U

/**
 * @brief The maximum block size which FBDMA can transfer. The client can
 * request a larger transfer than this but the driver will internally break it up
 * into smaller transfers.
 */
#define DMA_BLOCK_SIZE_MAX   0x100U

/**
 * @brief DMA transfer between TCM PA and External memory. This is a synchronous transfer.
 * The function will block until the transfer has completed.
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
 * @param[in] dma_idx Aperture index to use (0 to DMA_MAX_DMAIDX)
 * @param[in] b_read_ext Transfer direction. true: External->TCM. false: TCM->External
 *
 * @return E_SUCCESS on success
 *         E_INVALID_PARAM if tcm_pa is not within the range of the ITCM or DTCM or
 *         size_bytes, tcm_pa, or aperture_offset do not meet minimum alignment requirements.
 *         E_BUSY if there is a DMA NACK
 *         E_FAULT if there is a transfer error
 *
 * @pre The FBIF/TFBIF aperture referenced by dma_idx must have been configured by
 * fbifConfigureAperture or tfbifConfigureAperture. If the aperture is configured
 * to use a GPU VA, a context must be bound.
 */
error_t
fbdma_pa(uint64_t tcm_pa,
         uint64_t aperture_offset,
         uint64_t size_bytes,
         uint8_t  dma_idx,
         bool     b_read_ext
);

#if NVRISCV_FEATURE_SCP

/**
 * @brief DMA transfer from SCP to FB/Sysmem. This function uses the direct bypass
 * shortcut path from SCP: SCP registers --> SCPDMA --> FBDMA --> FB/SYSMEM
 *
 * @param[in] aperture_offset The external address (GPU PA or GPU VA) in the aperture
 *            determined by dma_idx. This can be a GPU VA or GPU PA. Must be aligned
              to SCP_REGISTER_SIZE.
 * @param[in] size_bytes The number of bytes to DMA. This value must be a multiple of
              SCP_REGISTER_SIZE and should generally match the size used to configure
              the corresponding SCPDMA transfer.
 * @param[in] dma_idx Aperture index to use (0 to DMA_MAX_dma_idx)
 *
 * @return E_SUCCESS on success
 *         E_INVALID_PARAM if size_bytes or aperture_offset do not meet
 *                                   minimum alignment requirements.
 *         E_BUSY if there is a DMA NACK
 *         E_FAULT if there is an error returned from the DMA hardware.
 *
 * @pre The FBIF/TFBIF aperture referenced by dma_idx must have been configured by
 * fbifConfigureAperture or tfbifConfigureAperture. If the aperture is configured
 * to use a GPU VA, a context must be bound. The caller must first call into the SCP
 * driver to configure SCPDMA for the transfer.
 */
error_t
fbdma_scp_to_extmem(
    uint64_t aperture_offset,
    uint64_t size_bytes,
    uint8_t  dma_idx
);

#endif // NVRISCV_FEATURE_SCP

#endif // FBDMA__FBDMA_H
