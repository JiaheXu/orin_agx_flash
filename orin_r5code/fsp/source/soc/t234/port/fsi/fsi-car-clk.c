/* Copyright (c) 2020-2021 NVIDIA CORPORATION.  All rights reserved.
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

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <soc-common/hw-const.h>        /* Must appear before any hwinc files */

/* Hardware headers */
#include <arclk_rst.h>                  // for CLK_RST_CONTROLLER_FMON_CONFIG_PLL_FSI_0...

/* Late FSP headers */
#include <misc/macros.h>                // for ARRAY_SIZE,...
#include <error/common-errors.h>        // for E_SUCCESS, error_t
#include <misc/bitops.h>                // for BIT, bit_number, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for core functions, ...
#include <car/car-fmon.h>               // for clock functions, ...
#include <car/car-clk.h>                // for clock functions, ...
#include <port/car-clk.h>               // for CLK index, ...
#include <port/car-pll.h>               // for PLL index, ...
#include <port/car-fmon.h>              // for FMON index, ...

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_CLK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_CLK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_PLL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_FMON_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
static bool fsi_clk_op_xtal_is_enabled(const struct car_clk_inf *clk)
{
    return true;
}

SECTION_CAR_TEXT
static error_t fsi_clk_op_xtal_enable(const struct car_clk_inf *clk)
{
    return E_SUCCESS;
}

SECTION_CAR_TEXT
static error_t fsi_clk_op_xtal_disable(const struct car_clk_inf *clk)
{
    return E_CAR_CLOCK_OP_NOT_SUPPORTED;
}

SECTION_CAR_RODATA
static const struct car_clk_op_able fsi_clk_op_able_xtal = {
    .clk_is_enabled = fsi_clk_op_xtal_is_enabled,
    .clk_enable = fsi_clk_op_xtal_enable,
    .clk_disable = fsi_clk_op_xtal_disable,
};

SECTION_CAR_TEXT
static error_t fsi_clk_op_xtal_hz_get(const struct car_clk_inf *clk, int64_t *hz_out)
{
    error_t ret = E_CAR_NULL_PTR;

    if (hz_out != NULL) {
        *hz_out = FSI_CLK_XTAL_HZ;
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t fsi_clk_op_xtal_hz_set(const struct car_clk_inf *clk, int64_t hz_out)
{
    return E_CAR_CLOCK_OP_NOT_SUPPORTED;
}

SECTION_CAR_RODATA
static const struct car_clk_op_rate fsi_clk_op_rate_xtal = {
    .clk_hz_get = fsi_clk_op_xtal_hz_get,
    .clk_hz_set = fsi_clk_op_xtal_hz_set,
};

SECTION_CAR_TEXT
static bool fsi_clk_op_pll_is_enabled(const struct car_clk_inf *clk)
{
    return fsi_pll_is_enabled();
}

SECTION_CAR_TEXT
static error_t fsi_clk_op_pll_enable(const struct car_clk_inf *clk)
{
    return fsi_pll_enable();
}

SECTION_CAR_TEXT
static error_t fsi_clk_op_pll_disable(const struct car_clk_inf *clk)
{
    return fsi_pll_disable();
}

SECTION_CAR_RODATA
static const struct car_clk_op_able fsi_clk_op_able_pll = {
    .clk_is_enabled = fsi_clk_op_pll_is_enabled,
    .clk_enable = fsi_clk_op_pll_enable,
    .clk_disable = fsi_clk_op_pll_disable,
};

SECTION_CAR_TEXT
static uint32_t fsi_clk_div_2N_1_in(int64_t hz_in, int64_t hz_out)
{
    float divider = (float)hz_in / hz_out;
    if (divider > 0) {
        divider = 2 * (divider - 1U);
    }
    return divider;
}

SECTION_CAR_TEXT
static uint32_t fsi_clk_div_2N_1_out(const struct car_clk_inf *clk)
{
    uint32_t divider = 0;
    if ((clk != NULL)) {
        divider = car_reg_rd_val(clk->div_reg, clk->div_msk);
        divider = divider >> 1U;
        divider += 1;
    }
    return divider;
}

SECTION_CAR_RODATA
static const struct car_clk_op_div  car_clk_op_div_2N_1 = {
    .clk_div_calc_in = fsi_clk_div_2N_1_in,
    .clk_div_calc_out = fsi_clk_div_2N_1_out,
};

SECTION_CAR_TEXT
static error_t fsi_clk_op_pll_hz_get(const struct car_clk_inf *clk, int64_t *hz_out)
{
    error_t ret = E_CAR_NULL_PTR;

    if (hz_out != NULL) {
        if (fsi_pll_is_enabled() == true) {
            ret = fsi_pll_get_rate(hz_out);
        } else {
            ret = fsi_clk_op_xtal_hz_get(NULL, hz_out);
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t fsi_clk_op_pll_hz_set(const struct car_clk_inf *clk, int64_t hz_out)
{
    return fsi_pll_set_rate(hz_out);
}

SECTION_CAR_RODATA
static const struct car_clk_op_rate fsi_clk_op_rate_pll = {
    .clk_hz_get = fsi_clk_op_pll_hz_get,
    .clk_hz_set = fsi_clk_op_pll_hz_set,
};


SECTION_CAR_RODATA
const struct car_clk_inf fsi_clk_infs[] = {
    /* fsi_xtal_clk */
    CLK_INF(FSI_CLK_ID_XTAL,
            &fsi_clk_op_able_xtal,              /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &fsi_clk_op_rate_xtal,              /* op_rate */
            40000000,                           /* rate_min */
            40000000,                           /* rate_max */
            NULL,                               /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_XTAL]),  /* fmon */
    /* fsi_pll_clk */
    CLK_INF(FSI_CLK_ID_PLL,
            &fsi_clk_op_able_pll,               /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &fsi_clk_op_rate_pll,               /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_XTAL],     /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_PLL]),   /* fmon */
    /* fsi_chsm_cpu_clk */
    CLK_INF(FSI_CLK_ID_CHSM_CPU,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSICHSM_CPU_BASE_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSICHSM_CPU_BASE_0_0_FSICHSM_CPU_CE_FIELD,
            CLK_RST_CONTROLLER_FSICHSM_CPU_BASE_0_0, /* div_reg */
            CLK_RST_CONTROLLER_FSICHSM_CPU_BASE_0_0_FSICHSM_CPU_DIVRATIO_FIELD,
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_CHSM]),  /* fmon */
    /* fsi_fabric_clk */
    CLK_INF(FSI_CLK_ID_FABRIC_CLK,
            NULL,                               /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            CLK_RST_CONTROLLER_FSI_FABRIC_BASE_0_0,
            CLK_RST_CONTROLLER_FSI_FABRIC_BASE_0_0_FSI_FABRIC_DIVRATIO_FIELD,
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_fabric_pclk */
    CLK_INF(FSI_CLK_ID_FABRIC_PCLK,
            NULL,                               /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_se_fr_clk */
    CLK_INF(FSI_CLK_ID_SE_FR,
            &car_clk_op_able_bit,               /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_se_clk */
    CLK_INF(FSI_CLK_ID_SE,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0_FSI_SE_CE_SHIFT,
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_sram_clk */
    CLK_INF(FSI_CLK_ID_SRAM_CLK,
            NULL,                               /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_sram_pclk */
    CLK_INF(FSI_CLK_ID_SRAM_PCLK,
            NULL,                               /* op_able */
            0,                                  /* en_reg */
            0,                                  /* en_msk */
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_uart0_baud_clk */
    CLK_INF(FSI_CLK_ID_UART0,
            &car_clk_op_able_reg,               /* op_able */
            CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTFSI_0, /* en_reg */
            CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTFSI_0_CLK_ENB_UARTFSI_FIELD,
            CLK_RST_CONTROLLER_CLK_SOURCE_UARTFSI_0, /* div_reg */
            CLK_RST_CONTROLLER_CLK_SOURCE_UARTFSI_0_UARTFSI_CLK_DIVISOR_FIELD,
            &car_clk_op_div_2N_1,               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_XTAL],     /* pclk */
            NULL),                              /* fmon */
    /* fsi_ts_clk */
    CLK_INF(FSI_CLK_ID_TS,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_TS_BASE_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_TS_BASE_0_0_FSI_TS_CE_FIELD,
            CLK_RST_CONTROLLER_FSI_TS_BASE_0_0, /* div_reg */
            CLK_RST_CONTROLLER_FSI_TS_BASE_0_0_FSI_TS_DIVRATIO_FIELD,
            &car_clk_op_div_2N_1,               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_XTAL],     /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_TS]),    /* fmon */
    /* fsi_spi_clk */
    CLK_INF(FSI_CLK_ID_SPI0,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_SPI_BASE_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_SPI_BASE_0_0_FSI_SPI_CE_FIELD,
            CLK_RST_CONTROLLER_FSI_SPI_BASE_0_0, /* div_reg */
            CLK_RST_CONTROLLER_FSI_SPI_BASE_0_0_FSI_SPI_DIVRATIO_FIELD,
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_XTAL],     /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_SPI]),   /* fmon */
    /* fsi_mc_clk */
    CLK_INF(FSI_CLK_ID_MC,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0_FSI_MC_CE_FIELD,
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_XTAL],     /* pclk */
            NULL),                              /* fmon */
    /* fsi_csite_clk */
    CLK_INF(FSI_CLK_ID_CSITE,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0_FSI_CSITE_CE_FIELD,
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_la_clk */
    CLK_INF(FSI_CLK_ID_LA,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0_FSI_LA_CE_FIELD,
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            &fsi_clk_infs[FSI_CLK_ID_PLL],      /* pclk */
            &fsi_fmon_infs[FSI_FMON_ID_FABRIC]), /* fmon */
    /* fsi_jtag_reg_clk */
    CLK_INF(FSI_CLK_ID_JTAG,
            &car_clk_op_able_bit,               /* op_able */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0, /* en_reg */
            CLK_RST_CONTROLLER_FSI_MISC_CONFIG_0_0_FSI_JTAG_CE_FIELD,
            0,                                  /* div_reg */
            0,                                  /* div_msk */
            NULL,                               /* op_div */
            &car_clk_op_rate_dflt,              /* op_rate */
            0,                                  /* rate_min */
            0,                                  /* rate_max */
            NULL,                               /* pclk */
            NULL),                              /* fmon */
};


SECTION_CAR_DATA
static bool fsi_clk_fmon = false;

SECTION_CAR_TEXT
bool fsi_clk_fmon_is_enabled(void)
{
    return fsi_clk_fmon;
}

SECTION_CAR_TEXT
void fsi_clk_fmon_enable(void)
{
    fsi_clk_fmon = true;
}

SECTION_CAR_TEXT
void fsi_clk_fmon_disable(void)
{
    fsi_clk_fmon = false;
}

SECTION_CAR_TEXT
static struct car_clk_inf *fsi_clk_inf_get(uint32_t clk_id)
{
    struct car_clk_inf *clk = NULL;

    if (clk_id < ARRAY_SIZE(fsi_clk_infs)) {
        clk = (struct car_clk_inf *)&fsi_clk_infs[clk_id];
    }
    return clk;
}

SECTION_CAR_TEXT
error_t fsi_clk_enable(uint32_t clk_id)
{
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (clk != NULL) {
        if (clk->op_able == NULL) {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        } else {
            ret = clk->op_able->clk_enable(clk);
            if (ret == E_SUCCESS) {
                if ((clk->fmon != NULL) && (fsi_clk_fmon == true)) {
                    ret = car_fmon_startup(clk->fmon);
                }
            }
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_clk_disable(uint32_t clk_id)
{
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (clk != NULL) {
        if (clk->op_able == NULL) {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        } else {
            if (fsi_clk_fmon == true) {
                ret = car_fmon_shutdown(clk->fmon);
            } else {
                ret = E_SUCCESS;
            }
            if (ret == E_SUCCESS || ret == E_CAR_NULL_PTR) {
                ret = clk->op_able->clk_disable(clk);
            }
        }
    }
    return ret;
}

SECTION_CAR_TEXT
bool fsi_clk_is_enabled(uint32_t clk_id)
{
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    bool ret = false;

    if (clk != NULL) {
        if (clk->op_able != NULL) {
            ret = clk->op_able->clk_is_enabled(clk);
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_clk_set_rate(uint32_t clk_id, int64_t hz_out)
{
    bool enabled;
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (clk != NULL) {
        if ((clk->op_rate != NULL) &&
            (clk->op_rate->clk_hz_set != NULL)) {
            enabled = fsi_clk_is_enabled(clk_id);
            ret = fsi_clk_disable(clk_id);
            if (ret == E_SUCCESS) {
                ret = clk->op_rate->clk_hz_set(clk, hz_out);
                if (ret == E_CAR_NULL_PTR) {
                    ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
                }
                if (enabled == true) {
                    ret = fsi_clk_enable(clk_id);
                }
            }
        } else {
            ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_clk_get_rate(uint32_t clk_id, int64_t *hz_out)
{
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (clk != NULL) {
        if (hz_out == NULL) {
            ret = E_CAR_NULL_PTR;
        } else {
            if ((clk->op_rate != NULL) &&
                (clk->op_rate->clk_hz_get != NULL)) {
                ret = clk->op_rate->clk_hz_get(clk, hz_out);
                if (ret == E_CAR_NULL_PTR) {
                    ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
                }
            } else {
                ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
            }
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_clk_set_div(uint32_t clk_id, uint32_t div)
{
    bool enabled;
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (clk != NULL) {
        ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
        if ((clk->op_rate != NULL) && (clk->op_rate->clk_hz_set != NULL)) {
            enabled = fsi_clk_is_enabled(clk_id);
            ret = fsi_clk_disable(clk_id);
            if ((ret == E_SUCCESS) && clk->div_msk != 0U) {
                car_reg_rdwr_val(clk->div_reg, clk->div_msk, div);
                if (enabled == true) {
                    ret = fsi_clk_enable(clk_id);
                }
            }
        }
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_clk_get_div(uint32_t clk_id, uint32_t *div)
{
    struct car_clk_inf *clk = fsi_clk_inf_get(clk_id);
    error_t ret = E_CAR_INVALID_CLOCK_ID;

    if (clk != NULL) {
        ret = E_CAR_NULL_PTR;
        if (div != NULL) {
            if (clk->div_msk != 0U) {
                *div = car_reg_rd_val(clk->div_reg, clk->div_msk);
                ret = E_SUCCESS;
            } else {
                ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
            }
        }
    }
    return ret;
}

SECTION_CAR_RODATA
#define str_E_SUCCESS                   "Success\n"
#define str_E_CAR_NULL_PTR              "NULL pointer\n"
#define str_E_CAR_INVALID_PARAM         "Invalid parameter\n"
#define str_E_CAR_INVALID_RESET_ID      "Invalid reset ID\n"
#define str_E_CAR_INVALID_CLOCK_ID      "Invalid clock ID\n"
#define str_E_CAR_INVALID_FMON_ID       "Invalid FMON ID\n"
#define str_E_CAR_PLL_NO_INIT           "PLL not initialized\n"
#define str_E_CAR_PLL_NO_LOCK           "PLL not locked\n"
#define str_E_CAR_CLOCK_OP_NOT_SUPPORTED "Clock operation not supported\n"
#define str_E_CAR_CLOCK_HZ_NOT_SUPPORTED "Frequency not supported\n"
#define str_E_CAR_FMON_HW_LOGIC         "FMON hardware logic\n"
#define str_E_CAR_FMON_HW_TIMEOUT       "FMON hardware timeout\n"
#define str_E_CAR_FMON_FAULT            "FMON fault\n"
#define str_E_CAR_FMON_NO_DATA          "FMON no data\n"
#define str_E_CAR_ERROR_N               "Unknown\n"

static const char * const car_err_strs[] = {
    [0]                                             = str_E_SUCCESS,
    [MODULE_ERROR_CODE(E_CAR_NULL_PTR)]             = str_E_CAR_NULL_PTR,
    [MODULE_ERROR_CODE(E_CAR_INVALID_PARAM)]        = str_E_CAR_INVALID_PARAM,
    [MODULE_ERROR_CODE(E_CAR_INVALID_RESET_ID)]     = str_E_CAR_INVALID_RESET_ID,
    [MODULE_ERROR_CODE(E_CAR_INVALID_CLOCK_ID)]     = str_E_CAR_INVALID_CLOCK_ID,
    [MODULE_ERROR_CODE(E_CAR_INVALID_FMON_ID)]      = str_E_CAR_INVALID_FMON_ID,
    [MODULE_ERROR_CODE(E_CAR_PLL_NO_INIT)]          = str_E_CAR_PLL_NO_INIT,
    [MODULE_ERROR_CODE(E_CAR_PLL_NO_LOCK)]          = str_E_CAR_PLL_NO_LOCK,
    [MODULE_ERROR_CODE(E_CAR_CLOCK_OP_NOT_SUPPORTED)] = str_E_CAR_CLOCK_OP_NOT_SUPPORTED,
    [MODULE_ERROR_CODE(E_CAR_CLOCK_HZ_NOT_SUPPORTED)] = str_E_CAR_CLOCK_HZ_NOT_SUPPORTED,
    [MODULE_ERROR_CODE(E_CAR_FMON_HW_LOGIC)]        = str_E_CAR_FMON_HW_LOGIC,
    [MODULE_ERROR_CODE(E_CAR_FMON_HW_TIMEOUT)]      = str_E_CAR_FMON_HW_TIMEOUT,
    [MODULE_ERROR_CODE(E_CAR_FMON_FAULT)]           = str_E_CAR_FMON_FAULT,
    [MODULE_ERROR_CODE(E_CAR_FMON_NO_DATA)]         = str_E_CAR_FMON_NO_DATA,
    [MODULE_ERROR_CODE(E_CAR_ERROR_N)]              = str_E_CAR_ERROR_N,
};

SECTION_CAR_TEXT
const char *fsi_car_get_err_str(error_t err)
{
    uint32_t i = MODULE_ERROR_CODE(err);
    uint32_t n = MODULE_ERROR_CODE(E_CAR_ERROR_N);
    const char *str = car_err_strs[n];

    if (i < n) {
        str = car_err_strs[i];
    }

    return str;
}

