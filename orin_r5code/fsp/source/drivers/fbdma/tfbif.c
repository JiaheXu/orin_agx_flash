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
 * @file    tfbif.c
 * @brief   TFBIF driver
 *
 * Used to configure TFBIF apertures
 */

#include <stddef.h>
#include <misc/nvmisc_drf.h>

#include <fbdma/tfbif.h>
#include <cpu/io_local.h>

//
// MISRA 4.9 RFD: function-like macros
// MISRA 15.1 RFD: Forward gotos
// MISRA 8.7 RFD: Functions with external linkage only referenced in one translation unit
//
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")


//The register manuals currently don't define these as indexed fields.
//Because of this, we need to define our own. We have asked the hardware
//Folks to provide indexed field definitions in the manuals, so we don't
//need to do this here.
#define NV_PRGNLCL_TFBIF_TRANSCFG_ATT_SWID(i)     ((i)*4u+1U):((i)*4U)
#define NV_PRGNLCL_TFBIF_TRANSCFG_ATT_SWID_XMSB(i)     ((i)*4U+1U)
#define NV_PRGNLCL_TFBIF_TRANSCFG_ATT_SWID_XLSB(i)     ((i)*4U)

#define NV_PRGNLCL_TFBIF_REGIONCFG_T_VPR(i)       ((i)*4U+3U):((i)*4U+3U)
#define NV_PRGNLCL_TFBIF_REGIONCFG_T_VPR_XMSB(i)       ((i)*4U+3U)
#define NV_PRGNLCL_TFBIF_REGIONCFG_T_VPR_XLSB(i)       ((i)*4U+3U)

#define NV_PRGNLCL_TFBIF_REGIONCFG1_T_APERT_ID(i) ((i)*8U+4U):((i)*8U)
#define NV_PRGNLCL_TFBIF_REGIONCFG1_T_APERT_ID_XMSB(i) ((i)*8U+4U)
#define NV_PRGNLCL_TFBIF_REGIONCFG1_T_APERT_ID_XLSB(i) ((i)*8U)

#define NV_PRGNLCL_TFBIF_REGIONCFG2_T_APERT_ID(i) (((i)-4U)*8U+4U):(((i)-4U)*8U)
#define NV_PRGNLCL_TFBIF_REGIONCFG2_T_APERT_ID_XMSB(i) (((i)-4U)*8U+4U)
#define NV_PRGNLCL_TFBIF_REGIONCFG2_T_APERT_ID_XLSB(i) (((i)-4U)*8U)

error_t
tfbif_configure_aperture
(
    const tfbif_aperture_cfg_t *p_cfgs,
    uint8_t num_cfgs
)
{
    error_t err = E_SUCCESS;
    uint8_t idx;
    uint32_t transcfg, regioncfg, regioncfg1, regioncfg2;

    if (p_cfgs == NULL)
    {
        err = E_INVALID_PARAM;
        goto out;
    }

    transcfg = localRead(NV_PRGNLCL_TFBIF_TRANSCFG);
    regioncfg = localRead(NV_PRGNLCL_TFBIF_REGIONCFG);
    regioncfg1 = localRead(NV_PRGNLCL_TFBIF_REGIONCFG1);
    regioncfg2 = localRead(NV_PRGNLCL_TFBIF_REGIONCFG2);

    for (idx = 0; idx < num_cfgs; idx++)
    {
        if (p_cfgs[idx].aperture_idx >= TFBIF_NUM_APERTURES ||
            p_cfgs[idx].swid >= (1 << DRF_SIZE(NV_PRGNLCL_TFBIF_TRANSCFG_ATT0_SWID)) ||
            p_cfgs[idx].aperture_id >= (1 << DRF_SIZE(NV_PRGNLCL_TFBIF_REGIONCFG1_T0_APERT_ID))
        )
        {
            err = E_INVALID_PARAM;
            goto out;
        }

        transcfg = FLD_IDX_SET_DRF_NUM(_PRGNLCL, _TFBIF_TRANSCFG, _ATT_SWID, p_cfgs[idx].aperture_idx, p_cfgs[idx].swid, transcfg);
        regioncfg = FLD_IDX_SET_DRF_NUM(_PRGNLCL, _TFBIF_REGIONCFG, _T_VPR, p_cfgs[idx].aperture_idx, p_cfgs[idx].b_vpr, regioncfg);
        if (p_cfgs[idx].aperture_idx <= 3)
        {
            regioncfg1 = FLD_IDX_SET_DRF_NUM(_PRGNLCL, _TFBIF_REGIONCFG1, _T_APERT_ID, p_cfgs[idx].aperture_idx, p_cfgs[idx].aperture_id, regioncfg1);
        }
        else
        {
            regioncfg2 = FLD_IDX_SET_DRF_NUM(_PRGNLCL, _TFBIF_REGIONCFG2, _T_APERT_ID, p_cfgs[idx].aperture_idx - 4, p_cfgs[idx].aperture_id, regioncfg2);
        }
    }

    localWrite(NV_PRGNLCL_TFBIF_TRANSCFG, transcfg);
    localWrite(NV_PRGNLCL_TFBIF_REGIONCFG, regioncfg);
    localWrite(NV_PRGNLCL_TFBIF_REGIONCFG1, regioncfg1);
    localWrite(NV_PRGNLCL_TFBIF_REGIONCFG2, regioncfg2);

out:
    return err;
}
