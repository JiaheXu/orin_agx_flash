/* Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stddef.h>                     // for NULL
#include <stdbool.h>                    // for bool, false
#include <stdint.h>                     // for uint32_t, uint8_t, UINT8_MAX, UIN...
#include <string.h>                     // for memcpy

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <soc-common/hw-const.h>        /* Must appear before any hwinc files */

/* Hardware headers */
#include <arclk_rst.h>                  // for CLK_RST_CONTROLLER_PLLFSI_BASE_0...

/* Late FSP headers */
#include <error/common-errors.h>        // for E_SUCCESS, error_t
#include <misc/macros.h>                // for ARRAY_SIZE,...

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for hardware access, ...
#include <car/car-hpll.h>               // for HPLL functions, ...
#include <port/car-pll.h>               // for PLL index, ...
#include <port/car-clk.h>               // for CLK index, ...

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_REG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_HPLL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_PLL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_CLK_H, "Header file missing or invalid.")


SECTION_CAR_RODATA
static const struct fsi_pll_init_cfg fsi_pll_init_cfgs[] = {
    [FSI_CAR_PLL_550mV_550MHz] = {
        .hz                     = 550 * MHz,      /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x02,           /* divp */
        .divn                   = 0x37,           /* divn */
        .divn_frac              = 0x0000,         /* divn_frac */
        .frac_step              = 0x0000,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0000,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_550mV_633MHz] = {
        .hz                     = 633 * MHz,      /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x02,           /* divp */
        .divn                   = 0x3F,           /* divn */
        .divn_frac              = 0x0999,         /* divn_frac */
        .frac_step              = 0x03E8,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB03E8,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_600mV_780MHz] = {
        .hz                     = 780 * MHz,      /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x02,           /* divp */
        .divn                   = 0x4E,           /* divn */
        .divn_frac              = 0x0000,         /* divn_frac */
        .frac_step              = 0x0000,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0000,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_600mV_897MHz] = {
        .hz                     = 897 * MHz,      /* hz_out */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x2C,           /* divn */
        .divn_frac              = 0x1B33,         /* divn_frac */
        .frac_step              = 0x06CC,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB06CC,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_670mV_1130MHz] = {
        .hz                     = 1130 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x38,           /* divn */
        .divn_frac              = 0x1000,         /* divn_frac */
        .frac_step              = 0x0400,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0400,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_670mV_1300MHz] = {
        .hz                     = 1300 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x41,           /* divn */
        .divn_frac              = 0x0000,         /* divn_frac */
        .frac_step              = 0x0000,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0000,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_720mV_1130MHz] = {
        .hz                     = 1130 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x38,           /* divn */
        .divn_frac              = 0x1000,         /* divn_frac */
        .frac_step              = 0x0400,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0400,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_720mV_1300MHz] = {
        .hz                     = 1300 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x41,           /* divn */
        .divn_frac              = 0x0000,         /* divn_frac */
        .frac_step              = 0x0000,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0000,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_850mV_1400MHz] = {
        .hz                     = 1400 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x46,           /* divn */
        .divn_frac              = 0x0000,         /* divn_frac */
        .frac_step              = 0x0000,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0000,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_850mV_1610MHz] = {
        .hz                     = 1610 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x50,           /* divn */
        .divn_frac              = 0x1000,         /* divn_frac */
        .frac_step              = 0x0400,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0400,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_940mV_1400MHz] = {
        .hz                     = 1400 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x46,           /* divn */
        .divn_frac              = 0x0000,         /* divn_frac */
        .frac_step              = 0x0000,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0000,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
    [FSI_CAR_PLL_940mV_1610MHz] = {
        .hz                     = 1610 * MHz,     /* hz */
        .divm                   = 0x02,           /* divm */
        .divp                   = 0x01,           /* divp */
        .divn                   = 0x50,           /* divn */
        .divn_frac              = 0x1000,         /* divn_frac */
        .frac_step              = 0x0400,         /* frac_step */
        .reg_misc_2             = 0x1F720F05,     /* reg_misc_2 */
        .reg_misc_3             = 0x00000004,     /* reg_misc_3 */
        .reg_misc_4             = 0x01CB0400,     /* reg_misc_4 */
        .reg_misc_5             = 0x00000000,     /* reg_misc_5 */
        .reg_misc_6             = 0x00000000,     /* reg_misc_6 */
    },
};

SECTION_CAR_RODATA
static const struct car_pll_cfg fsi_hpll_cfgs[] = {
    PLL_CFG(FSI_CAR_PLL_550mV_550MHz,
            40 * MHz,       /* hz_in */
            550 * MHz,      /* hz_out */
            0x37,           /* n */
            0x02,           /* m */
            0x02,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_550mV_633MHz,
            40 * MHz,       /* hz_in */
            633 * MHz,      /* hz_out */
            0x3F,           /* n */
            0x02,           /* m */
            0x02,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_600mV_780MHz,
            40 * MHz,       /* hz_in */
            780 * MHz,      /* hz_out */
            0x4E,           /* n */
            0x02,           /* m */
            0x02,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_600mV_897MHz,
            40 * MHz,       /* hz_in */
            897 * MHz,      /* hz_out */
            0x2C,           /* n */
            0x02,           /* m */
            0x01,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_670mV_1130MHz,
            40 * MHz,       /* hz_in */
            1130 * MHz,     /* hz_out */
            0x38,           /* n */
            0x02,           /* m */
            0x01,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_670mV_1300MHz,
            40 * MHz,       /* hz_in */
            1300 * MHz,     /* hz_out */
            0x41,           /* n */
            0x02,           /* m */
            0x01,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_850mV_1400MHz,
            40 * MHz,       /* hz_in */
            1400 * MHz,     /* hz_out */
            0x46,           /* n */
            0x02,           /* m */
            0x01,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
    PLL_CFG(FSI_CAR_PLL_850mV_1610MHz,
            40 * MHz,       /* hz_in */
            1610 * MHz,     /* hz_out */
            0x50,           /* n */
            0x02,           /* m */
            0x01,           /* p */
            false,          /* en_sdm */
            false,          /* en_ssc */
            0x0000,         /* sdm_din */
            0x0000,         /* sdm_ssc_min */
            0x0000,         /* sdm_ssc_max */
            0x0000,         /* sdm_ssc_step */
            0x0000,         /* sscmax */
            0x00,           /* sscinc */
            0x00),          /* sscincintrv */
};

SECTION_CAR_RODATA
static const struct car_pll_spec fsi_pll_spec = {
    .type = CAR_PLL_TYPE_HPLL16,
    .flags = CAR_PLL_FLG_HAS_DYNAMIC_RAMP,
    .frac_width = 16,
    .min_n = 1,
    .min_m = 2,
    .min_p = 1,
    .clkin_max = 40 * MHz,
    .vco_min = 650 * MHz,
    .vco_max = 2000 * MHz,
    .cf_min = 38400 * kHz,
    .cf_max = 38400 * kHz,
    .max_n = 255,
    .max_m = 2,
    .max_p = 2,
};

SECTION_CAR_DATA
static const struct car_pll_spec *fsi_pll_spec_p = &fsi_pll_spec;
static const struct car_pll_cfg *fsi_hpll_cfgs_p = fsi_hpll_cfgs;
static uint32_t fsi_hpll_cfgs_n = ARRAY_SIZE(fsi_hpll_cfgs);
static uint32_t fsi_hpll_status = 0;

SECTION_CAR_RODATA
static const struct car_pll_inf fsi_hpll_infs[] = {
    HPLL_INF(0,
             &fsi_pll_spec_p,           /* **spec */
             &fsi_hpll_cfgs_p,          /* **cfgs */
             &fsi_hpll_cfgs_n,          /* *cfgs_n */
             &fsi_hpll_status,          /* *sts */
             CLK_RST_CONTROLLER_FSI_CPU_BASE_0_0, /* sel_reg */
             CLK_RST_CONTROLLER_FSI_CPU_BASE_0_0_FSICPU_CLK_SRC_SEL_FIELD, /* sel_msk */
             CLK_RST_CONTROLLER_PLLFSI_MISC_0_0, /* rst_reg */
             CLK_RST_CONTROLLER_PLLFSI_MISC_0_0_PLLFSI_RESETB_FIELD, /* rst_msk */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0, /* en_reg */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0_PLLFSI_ENABLE_FIELD, /* en_msk */
             CLK_RST_CONTROLLER_PLLFSI_MISC_1_0, /* iddq_reg */
             CLK_RST_CONTROLLER_PLLFSI_MISC_1_0_PLLFSI_IDDQ_FIELD, /* iddq_msk */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0, /* mdiv_reg */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0_PLLFSI_DIVM_FIELD, /* mdiv_msk */
             CLK_RST_CONTROLLER_PLLFSI_MISC_1_0, /* ndiv_reg */
             CLK_RST_CONTROLLER_PLLFSI_MISC_1_0_PLLFSI_DIVN_FIELD, /* ndiv_msk */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0, /* pdiv_reg */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0_PLLFSI_DIVP_FIELD, /* pdiv_msk */
             CLK_RST_CONTROLLER_PLLFSI_MISC_1_0, /* frac_reg */
             CLK_RST_CONTROLLER_PLLFSI_MISC_1_0_PLLFSI_DIVN_FRAC_FIELD, /* frac_msk */
             0, /* enfrac_reg */
             0, /* enfrac_msk */
             CLK_RST_CONTROLLER_PLLFSI_MISC_4_0, /* fracstep_reg */
             CLK_RST_CONTROLLER_PLLFSI_MISC_4_0_PLLFSI_FRAC_STEP_FIELD, /* fracstep_msk */
             0, /* iref_reg */
             0, /* iref_msk */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0, /* lock_reg */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0_PLLFSI_LOCK_FIELD, /* lock_msk */
             0, /* lckdet_reg */
             0, /* lckdet_msk */
             CLK_RST_CONTROLLER_PLLFSI_BASE_0_PLLFSI_FREQ_LOCK_FIELD, /* freqlock_msk */
             0), /* ss_cntl_reg */
};


SECTION_CAR_TEXT
bool fsi_pll_is_enabled()
{
    return fsi_hpll_infs[0].op->pll_is_enabled(&fsi_hpll_infs[0]);
}

SECTION_CAR_TEXT
error_t fsi_pll_enable()
{
    return fsi_hpll_infs[0].op->pll_enable(&fsi_hpll_infs[0]);
}

SECTION_CAR_TEXT
error_t fsi_pll_disable()
{
    return fsi_hpll_infs[0].op->pll_disable(&fsi_hpll_infs[0]);
}

SECTION_CAR_TEXT
error_t fsi_pll_set_rate(int64_t hz_out)
{
    int64_t hz_in;
    error_t ret;

    ret = fsi_clk_get_rate(FSI_CLK_ID_XTAL, &hz_in);
    if (ret == E_SUCCESS) {
        ret = fsi_hpll_infs[0].op->pll_hz_set(&fsi_hpll_infs[0], hz_in, hz_out);
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_pll_get_rate(int64_t *hz_out)
{
    int64_t hz_in;
    error_t ret;

    ret = fsi_clk_get_rate(FSI_CLK_ID_XTAL, &hz_in);
    if (ret == E_SUCCESS) {
        *hz_out = fsi_hpll_infs[0].op->pll_hz_get(&fsi_hpll_infs[0], hz_in);
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_pll_spec_get(struct car_pll_spec *spec)
{
    error_t ret = E_CAR_NULL_PTR;

    if (spec != NULL) {
        memcpy(spec, *fsi_hpll_infs[0].spec, sizeof(*spec));
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_pll_spec_set(struct car_pll_spec *spec)
{
    if (spec == NULL) {
        fsi_pll_spec_p = &fsi_pll_spec;
    } else {
        fsi_pll_spec_p = spec;
    }
    return E_SUCCESS;
}

SECTION_CAR_TEXT
error_t fsi_pll_cfgs_set(struct car_pll_cfg *cfgs, uint32_t cfgs_n)
{
    if (cfgs == NULL) {
        fsi_hpll_cfgs_p = fsi_hpll_cfgs;
        fsi_hpll_cfgs_n = ARRAY_SIZE(fsi_hpll_cfgs);
    } else {
        fsi_hpll_cfgs_p = cfgs;
        fsi_hpll_cfgs_n = cfgs_n;
    }
    return E_SUCCESS;
}

SECTION_CAR_TEXT
error_t fsi_pll_init_cfg_rd(struct fsi_pll_init_cfg *cfg)
{
    error_t ret = E_CAR_NULL_PTR;

    if (cfg != NULL) {
        cfg->reg_misc_6 = car_reg_rd(CLK_RST_CONTROLLER_PLLFSI_MISC_6_0);
        cfg->reg_misc_5 = car_reg_rd(CLK_RST_CONTROLLER_PLLFSI_MISC_5_0);
        cfg->reg_misc_4 = car_reg_rd(CLK_RST_CONTROLLER_PLLFSI_MISC_4_0);
        cfg->reg_misc_3 = car_reg_rd(CLK_RST_CONTROLLER_PLLFSI_MISC_3_0);
        cfg->reg_misc_2 = car_reg_rd(CLK_RST_CONTROLLER_PLLFSI_MISC_2_0);
        cfg->divm = (uint8_t)car_reg_rd_val(fsi_hpll_infs[0].mdiv_reg,
                                            fsi_hpll_infs[0].mdiv_msk);
        cfg->divn = (uint8_t)car_reg_rd_val(fsi_hpll_infs[0].ndiv_reg,
                                            fsi_hpll_infs[0].ndiv_msk);
        cfg->divp = (uint8_t)car_reg_rd_val(fsi_hpll_infs[0].pdiv_reg,
                                            fsi_hpll_infs[0].pdiv_msk);
        cfg->divn_frac = (uint16_t)car_reg_rd_val(fsi_hpll_infs[0].frac_reg,
                                                  fsi_hpll_infs[0].frac_msk);
        cfg->frac_step = (uint16_t)car_reg_rd_val(fsi_hpll_infs[0].fracstep_reg,
                                                  fsi_hpll_infs[0].fracstep_msk);
        ret = fsi_pll_get_rate(&cfg->hz);
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_pll_init_cfg_wr(struct fsi_pll_init_cfg *cfg)
{
    error_t ret = E_CAR_NULL_PTR;

    if (cfg != NULL) {
        /* disable PLL */
        ret = fsi_pll_disable();
        if (ret == E_SUCCESS) {
            /* PLL programming */
            car_reg_wr(CLK_RST_CONTROLLER_PLLFSI_MISC_6_0,
                       cfg->reg_misc_6);
            car_reg_wr(CLK_RST_CONTROLLER_PLLFSI_MISC_5_0,
                       cfg->reg_misc_5);
            car_reg_wr(CLK_RST_CONTROLLER_PLLFSI_MISC_4_0,
                       cfg->reg_misc_4);
            car_reg_wr(CLK_RST_CONTROLLER_PLLFSI_MISC_3_0,
                       cfg->reg_misc_3);
            car_reg_wr(CLK_RST_CONTROLLER_PLLFSI_MISC_2_0,
                       cfg->reg_misc_2);
            if (fsi_hpll_infs[0].mdiv_msk != 0U) {
                car_reg_rdwr_val(fsi_hpll_infs[0].mdiv_reg,
                                 fsi_hpll_infs[0].mdiv_msk,
                                 cfg->divm);
            }
            if (fsi_hpll_infs[0].ndiv_msk != 0U) {
                car_reg_rdwr_val(fsi_hpll_infs[0].ndiv_reg,
                                 fsi_hpll_infs[0].ndiv_msk,
                                 cfg->divn);
            }
            if (fsi_hpll_infs[0].pdiv_msk != 0U) {
                car_reg_rdwr_val(fsi_hpll_infs[0].pdiv_reg,
                                 fsi_hpll_infs[0].pdiv_msk,
                                 cfg->divp);
            }
            if (fsi_hpll_infs[0].frac_msk != 0U) {
                car_reg_rdwr_val(fsi_hpll_infs[0].frac_reg,
                                 fsi_hpll_infs[0].frac_msk,
                                 cfg->divn_frac);
            }
            /* flag the PLL is initialized */
            fsi_hpll_status |= CAR_PLL_STS_HZ_SET;
            /* enable PLL */
            ret = fsi_pll_enable();
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_pll_init(uint32_t pll_vf)
{
    error_t ret = E_CAR_PLL_NO_INIT;

    if (pll_vf < ARRAY_SIZE(fsi_pll_init_cfgs)) {
        ret = fsi_pll_init_cfg_wr((struct fsi_pll_init_cfg *)&fsi_pll_init_cfgs[pll_vf]);
    }
    return ret;
}

