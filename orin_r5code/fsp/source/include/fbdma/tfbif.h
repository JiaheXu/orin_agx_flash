/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#ifndef FBDMA__TFBIF_H
#define FBDMA__TFBIF_H
#define FSP__FBDMA__TFBIF_H 1

#include <stdint.h>
#include <stdbool.h>
#include <error/common-errors.h>

#ifndef NVRISCV_HAS_TFBIF
#error "This header should only be used on engines which have a TFBIF block."
#endif // NVRISCV_HAS_TFBIF

/**
 * @brief The number of configurable TFBIF apertures.
 */
#define TFBIF_NUM_APERTURES 8U

/**
 * @brief Configuration struct for a TFBIF Aperture. This is the Tegra equivilant of
 * FBIF_APERTURE_CFGThe fields in this structure are used to configure the TFBIF_TRANSCFG
 * and TFBIF_REGIONCFG registers of the TFBIF, which determine how the FBDMA engine
 * will access external memory. More details on these fields can be found in
 * the TFBIF Hardware documentation.
 *
 * ``aperture_idx``: The aperture to configure (0-7). Programmed into the ATT_SWID
 * field of the TFBIF_TRANSCFG register corresponding to this aperture.
 *
 * ``swid``: The SWID value for this aperture. Programmed into the ATT_SWID field
 * of the TFBIF_TRANSCFG register corresponding to this aperture.
 *
 * ``b_vpr``: The VPR to use for this aperture. Programmed into the ATT_SWID field
 * of the TFBIF_TRANSCFG register corresponding to this aperture.
 *
 * ``aperture_id``: Aperture ID AKA GSC_ID. Programmed into the Tx_APERT_ID field
 * of the TFBIF_REGIONCFG register corresponding to this aperture.
 */
typedef struct {
    uint8_t aperture_idx;
    uint8_t swid;
    bool b_vpr;
    uint8_t aperture_id;
} tfbif_aperture_cfg_t;

/**
 * @brief Configure some of the 8 DMA apertures in TFBIF.
 * A similar function (fbifConfigureAperture) exists for FBIF
 * Which is used on dGPU and Tegra iGPU engines. This function is not thread
 * safe. The client must ensure that there is no chance of it being
 * preempted by another Task or ISR which may call a DMA function
 * or touch the TFBIF registers.
 *
 * @param[in] p_cfgs an array of tfbif_aperture_cfg_t - one per aperture
 * @param[in] num_cfgs is the size of p_cfgs (1-8)
 * @return E_SUCCESS on success
 *         E_INVALID_PARAM if p_cfgs is not sane
 */
error_t tfbif_configure_aperture(const tfbif_aperture_cfg_t *p_cfgs, uint8_t num_cfgs);
#endif // FBDMA__TFBIF_H