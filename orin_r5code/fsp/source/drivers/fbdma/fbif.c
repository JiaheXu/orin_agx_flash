/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021-2022 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

/*!
 * @file    fbif.c
 * @brief   FBIF driver
 *
 * Used to configure FBIF apertures
 */

#include <stddef.h>
#include <misc/nvmisc_drf.h>

#include <fbdma/fbif.h>
#include <cpu/io_local.h>

//
// MISRA 4.9 RFD: function-like macros
// MISRA 15.1 RFD: Forward gotos
// MISRA 8.7 RFD: Functions with external linkage only referenced in one translation unit
//
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")

//The register manual currently doesn't define this as an indexed field.
//Because of this, we need to define our own. We have asked the hardware
//Folks to provide an indexed field definition in the manuals, so we don't
//need to do this here.
#define NV_PRGNLCL_FBIF_REGIONCFG_T_XMSB(i) (((i)*4U)+3U)
#define NV_PRGNLCL_FBIF_REGIONCFG_T_XLSB(i) ((i)*4U)

error_t
fbif_configure_aperture
(
    const fbif_aperture_cfg_t *p_cfgs,
    uint8_t num_cfgs
)
{
    error_t err = E_SUCCESS;
    uint8_t idx;
    uint32_t regioncfg;

    if (p_cfgs == NULL)
    {
        err = E_INVALID_PARAM;
        goto out;
    }

    regioncfg = localRead(NV_PRGNLCL_FBIF_REGIONCFG);

    for (idx = 0U; idx < num_cfgs; idx++)
    {
        if ((p_cfgs[idx].aperture_idx >= FBIF_NUM_APERTURES) ||
            (p_cfgs[idx].target >= FBIF_TRANSCFG_TARGET_COUNT) ||
            (p_cfgs[idx].l2c_wr >= FBIF_TRANSCFG_L2C_COUNT) ||
            (p_cfgs[idx].l2c_rd >= FBIF_TRANSCFG_L2C_COUNT) ||
            (p_cfgs[idx].region_id >= (1U << DRF_SIZE(NV_PRGNLCL_FBIF_REGIONCFG_T0)))
        )
        {
            err = E_INVALID_PARAM;
            goto out;
        }

        regioncfg = FLD_IDX_SET_DRF_NUM(_PRGNLCL, _FBIF_REGIONCFG, _T,
                                        p_cfgs[idx].aperture_idx, p_cfgs[idx].region_id, regioncfg);

        localWrite(NV_PRGNLCL_FBIF_TRANSCFG((uint32_t)p_cfgs[idx].aperture_idx),
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _TARGET, p_cfgs[idx].target) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _MEM_TYPE,
                ((p_cfgs[idx].b_target_va) ?
                NV_PRGNLCL_FBIF_TRANSCFG_MEM_TYPE_VIRTUAL : NV_PRGNLCL_FBIF_TRANSCFG_MEM_TYPE_PHYSICAL)) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _L2C_WR, p_cfgs[idx].l2c_wr) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _L2C_RD, p_cfgs[idx].l2c_rd) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _WACHK0, p_cfgs[idx].b_fbif_transcfg_wachk0_enable ? 1 : 0) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _WACHK1, p_cfgs[idx].b_fbif_transcfg_wachk1_enable ? 1 : 0) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _RACHK0, p_cfgs[idx].b_fbif_transcfg_rachk0_enable ? 1 : 0) |
            DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _RACHK1, p_cfgs[idx].b_fbif_transcfg_rachk1_enable ? 1 : 0)
#ifdef NV_PRGNLCL_FBIF_TRANSCFG_ENGINE_ID_FLAG
            //Some instances of the FBIF do not have this field
            | DRF_NUM(_PRGNLCL, _FBIF_TRANSCFG, _ENGINE_ID_FLAG, p_cfgs[idx].b_engine_id_flag_own ? 1 : 0)
#endif //NV_PRGNLCL_FBIF_TRANSCFG_ENGINE_ID_FLAG
            );
    }

    localWrite(NV_PRGNLCL_FBIF_REGIONCFG, regioncfg);

out:
    return err;
}

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
