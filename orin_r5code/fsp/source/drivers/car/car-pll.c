/* Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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
#include <string.h>                     // for memset

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <soc-common/hw-const.h>        /* Must appear before any hwinc files */

/* Hardware headers */
#include <arclk_rst.h>                  // for CLK_RST_CONTROLLER_PLLFSI_BASE_0...

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for E_SUCCESS, error_t
#include <misc/bitops.h>                // for BIT, bit_number, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for register access functions...
#include <car/car-math.h>               // for math functions...
#include <car/car-pll.h>                // for PLL declarations...
#include <port/car-port.h>              // for platform functions...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_REG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_MATH_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_PLL_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
error_t car_pll_wait_lock_us(const struct car_pll_inf *pll, uint64_t t_us)
{
    uint64_t ticks_start;
    uint64_t ticks_us;
    error_t ret = E_CAR_NULL_PTR;

    if (pll != NULL) {
        ticks_start = car_port_get_time_ticks();
        do {
            if (car_reg_rd(pll->lock_reg) & pll->lock_msk) {
                ret = E_SUCCESS;
                break;
            }

            ticks_us = car_port_get_time_delta_us(ticks_start);
            if (ticks_us > t_us) {
                ret = E_CAR_PLL_NO_LOCK;
                break;
            }

            car_port_udelay(5ULL);
        } while (true);
    }

    return ret;
}

SECTION_CAR_TEXT
error_t car_pll_wait_lock(const struct car_pll_inf *pll)
{
    return car_pll_wait_lock_us(pll, CAR_PLL_LOCK_DELAY);
}

/* calc MDIV and adjust if VCO range cannot be met
 * with possible range of NDIV.
 */
SECTION_CAR_TEXT
static int64_t car_pll_calc_m(const struct car_pll_spec *spec, int64_t hz_in)
{
    const int64_t max_m = (int64_t)spec->max_m;
    const int64_t max_n = (int64_t)spec->max_n;
    const int64_t min_m = (int64_t)spec->min_m;
    const int64_t min_n = (int64_t)spec->min_n;
    int64_t vmin;
    int64_t vmax;
    int64_t cf;
    int64_t m;

    if (spec->min_m != spec->max_m) {
        m = DIV_ROUND_UP(hz_in, spec->cf_max);
        vmin = (hz_in * min_n) / m;
        if (vmin > spec->vco_min) {
            m = DIV_ROUND_UP(hz_in * min_n, spec->vco_min);
        }
        vmax = (hz_in * max_n) / m;
        cf = hz_in / m;
        if ((m < min_m) || (m > max_m) || (cf < spec->cf_min) || (cf > spec->cf_max) || (vmax < spec->vco_max)) {
            m = E_CAR_INVALID_PARAM;
        }
    } else {
        m = spec->min_m;
    }
    return m;
}

SECTION_CAR_TEXT
static int64_t car_pll_calc_n(const struct car_pll_spec *spec, int64_t f_in,
                              int64_t target_vco, uint32_t flags, int32_t *frac)
{
    const bool round_down = ((flags & CAR_PLL_FLG_ROUND_DOWN) != 0U);
    const bool nofrac = ((flags & CAR_PLL_FLG_FRAC_NDIV) == 0U);
    const int64_t min_n = (int64_t)spec->min_n;
    const int64_t max_n = (int64_t)spec->max_n;
    const int64_t mul = (int64_t)1LL << spec->frac_width;
    const int64_t dy = mul / 2LL;
    int64_t n;
    int64_t f;

    /* note that target_vco is actually the VCO target multiplied
     * by fractional "accuracy" multiplied by m
     */
    if (round_down) {
        n = car_div_s64(target_vco, f_in);
    } else {
        n = car_div64_round(target_vco, f_in);
    }

    if (nofrac) {
        f = 0;
    } else if (spec->type == CAR_PLL_TYPE_ANALOG) {
        f = car_sub_s64(n % mul, dy);
        /* intentionally round down */
        n = n / mul;
    } else {
        f = car_add_s64(n, dy) % mul;
        f = car_sub_s64(f, dy);
        n = car_div64_round(n, mul);
    }
    n = car_range_s64(n, min_n, max_n);
    *frac = (int32_t)(f);
    return n;
}

/* if p_pref can satisfy vco range, return that */
SECTION_CAR_TEXT
static int64_t car_pll_calc_p(const struct car_pll_spec *spec, int64_t rate,
                              int64_t vco_min, int64_t vco_max, int32_t p_pref, uint32_t flags)
{
    const bool round_down = ((flags & CAR_PLL_FLG_ROUND_DOWN) != 0U);
    const int64_t max_p = (int64_t)spec->max_p;
    const int64_t min_p = (int64_t)spec->min_p;
    int64_t vco_mid;
    int64_t p = 0;
    int64_t xd;
    int64_t nd;

    if ((p_pref > 0) && (car_range_s64(rate, vco_min / p_pref, vco_max / p_pref)) != 0LL) {
        p = p_pref;
    } else if (((flags & CAR_PLL_FLG_USE_MID_VCO) != 0U) && (rate != 0)) {
        vco_mid = car_add_s64(vco_max, vco_min) / 2LL;
        p = car_div64_round(vco_mid, rate);
    } else if (rate != 0) {
        p = vco_max / rate;
    }
    p = max(p, min_p);
    /* handle a special case that can happen if vco range cannot
     * cover all possible frequencies. In other words, case when
     * vco_max < 2 * vco_min
     */
    if (car_mul_s64(p, rate) < vco_min) {
        /* when not rounding to nearest, always select higher
         * P. Otherwise select P that gives closer match */
        xd = car_sub_s64(rate, (vco_max / (p + 1)));
        nd = (vco_min / p) - rate;
        if (round_down || (xd <= nd)) {
            ++p;
        }
    }
    p = min(p, max_p);
    return p;
}

/* scale rate, vco_min, and vco_max to precision of
 * fractional step (if fractions are enabled)
 */
SECTION_CAR_TEXT
static int64_t car_pll_scale(const struct car_pll_spec *spec, uint32_t flags)
{
    const bool nofrac = ((flags & CAR_PLL_FLG_FRAC_NDIV) == 0U);
    int64_t mul;

    if (nofrac) {
        mul = 1LL;
    } else {
        mul = (int64_t)1LL << spec->frac_width;
    }
    return mul;
}

SECTION_CAR_TEXT
static int64_t car_pll_round(int64_t hz, int64_t dy, bool up)
{
    const int64_t r = hz % dy;
    int64_t ret_hz = hz;

    if (r > 0) {
        if (up) {
            ret_hz += dy - r;
        } else {
            ret_hz -= r;
        }
    }
    return ret_hz;
}

SECTION_CAR_TEXT
error_t car_pll_nmp_calc(const struct car_pll_spec *spec, struct car_pll_nmp *nmp,
                         int64_t hz_out, int64_t hz_in, int32_t p_pref, uint32_t flags)
{
    int64_t m, n, p;
    int64_t target_vco;
    const int64_t fx = car_pll_scale(spec, flags);
    int64_t vco_min = spec->vco_min;
    int64_t vco_max = spec->vco_max;
    int64_t rate;
    int64_t mul;
    int32_t frac;
    error_t ret = E_SUCCESS;

    if (hz_in > spec->clkin_max) {
        ret = E_CAR_INVALID_PARAM;
        goto out;
    }

    m = car_pll_calc_m(spec, hz_in);
    if (m < 0) {
        ret = (int32_t)m;
        goto out;
    }

    /* rate cannot ever go beyond vco_max */
    rate = car_range_s64(hz_out, 1LL, vco_max);
    /* preserve accuracy with following calculations */
    mul = car_mul_s64(fx, m);
    vco_min = car_mul_s64(vco_min, mul);
    vco_max = car_mul_s64(vco_max, mul);
    rate = car_mul_s64(rate, mul);
    /* clamp vco_min and vco_max to frequencies that can be
     * actually achieved. In practice this makes difference only
     * when fractions are disabled because only then we could end
     * up violating vco ranges in any meaningful amount
     */
    vco_min = car_pll_round(vco_min, hz_in, true);
    vco_max = car_pll_round(vco_max, hz_in, false);
    p = car_pll_calc_p(spec, rate, vco_min, vco_max, p_pref, flags);
    target_vco = car_mul_s64(p, rate);
    target_vco = car_range_s64(target_vco, vco_min, vco_max);
    n = car_pll_calc_n(spec, hz_in, target_vco, flags, &frac);
    nmp->n = (uint32_t)n;
    nmp->m = (uint32_t)m;
    nmp->p = (uint32_t)p;
    nmp->frac = frac;

out:
    return ret;
}

SECTION_CAR_TEXT
static int64_t libpll_calc_rate(const struct car_pll_inf *pll,
                                const struct car_pll_nmp *nmp,
                                int64_t hz_in, uint32_t flags)
{
    const struct car_pll_spec *spec = *pll->spec;
    const bool enfrac = ((flags & CAR_PLL_FLG_FRAC_NDIV) != 0U);
    const bool hpll = (spec->type != CAR_PLL_TYPE_ANALOG);
    const int64_t fx = (int64_t)1LL << spec->frac_width;
    int64_t hz_out;
    int64_t n;

    n = car_mul_s64(nmp->n, fx);
    if (hpll) {
        n = car_add_s64(n, nmp->frac);
    } else if (enfrac) {
        n = car_add_s64(n, nmp->frac);
        n = car_add_s64(n, fx / 2LL);
    } else {
        /* nothing */
    }
    hz_out = hz_in / nmp->m;
    hz_out = car_mul_s64(hz_out, n);
    hz_out /= nmp->p;
    hz_out /= fx;
    return hz_out;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_CAR_TEXT int64_t car_pll_hz_calc(const struct car_pll_inf *pll, struct car_pll_cfg *cfg, int64_t hz_in)
{
    const uint32_t flags = cfg->en_sdm ? CAR_PLL_FLG_FRAC_NDIV : 0U;
    const struct car_pll_nmp nmp = {
        .n = max(cfg->n, 1U),
        .m = max(cfg->m, 1U),
        .p = max(cfg->p, 1U),
        .frac = cfg->sdm_din,
    };

    return libpll_calc_rate(pll, &nmp, hz_in, flags);
}

SECTION_CAR_TEXT
static bool car_pll_en_frac(const struct car_pll_inf *pll)
{
    bool en;

    if (pll->op->frac_allowed != NULL) {
        en = pll->op->frac_allowed(pll);
    } else {
        en = (pll->frac_msk != 0U);
    }
    return en;
}

error_t car_pll_cfg_calc(const struct car_pll_inf *pll,
                         int64_t hz_in, int64_t hz_out, int64_t hz_max,
                         struct car_pll_cfg *cfg, int32_t p_curr)
{
    const bool enfrac = car_pll_en_frac(pll);
    uint32_t flags = enfrac ? CAR_PLL_FLG_FRAC_NDIV : 0U;
    struct car_pll_nmp nmp;
    int64_t rrate;
    error_t ret;

    ret = car_pll_nmp_calc(*pll->spec, &nmp, hz_out, hz_in, p_curr, flags);
    if (ret == E_SUCCESS) {
        /* if hz_max is exceeded, recalculate with rounding down */
        rrate = libpll_calc_rate(pll, &nmp, hz_in, flags);
        if (rrate > hz_max) {
            /*
             * the car_pll_nmp_calc below differs from the one above only in
             * flags, which doesn't affect return status. so, if the
             * previous call of car_pll_nmp_calc returns 0, this one will
             * return 0 too.
             */
            CAR_RET_UNUSED(car_pll_nmp_calc(*pll->spec, &nmp, hz_in, hz_out, p_curr,
                                            flags | CAR_PLL_FLG_ROUND_DOWN));
        }
        memset(cfg, 0, sizeof(*cfg));
        cfg->n = nmp.n;
        cfg->m = nmp.m;
        cfg->p = nmp.p;
        /* for calculated PLL parameters, en_ssc is always 0 */
        cfg->sdm_din = (int16_t)(nmp.frac);
        cfg->en_sdm = enfrac;
    }
    return ret;
}

SECTION_CAR_TEXT
void car_pll_nmp_get(const struct car_pll_inf *pll, struct car_pll_cfg *cfg)
{
    uint32_t v;

    cfg->n = car_reg_rd_val(pll->ndiv_reg, pll->ndiv_msk);
    cfg->m = car_reg_rd_val(pll->mdiv_reg, pll->mdiv_msk);
    if (pll->pdiv_msk != 0U) {
        cfg->p = car_reg_rd_val(pll->pdiv_reg, pll->pdiv_msk);
    } else {
        cfg->p = 1U;
    }
    if (pll->frac_msk != 0U) {
        v = car_reg_rd_val(pll->frac_reg, pll->frac_msk);
        cfg->sdm_din = (int16_t)v;
    } else {
        cfg->sdm_din = 0;
    }
    if (pll->enfrac_msk != 0) {
        v = car_reg_rd_val(pll->enfrac_reg, pll->enfrac_msk);
        cfg->en_sdm = (v != 0U);
    } else {
        cfg->en_sdm = false;
    }
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_CAR_TEXT void car_pll_nmp_set(const struct car_pll_inf *pll, struct car_pll_cfg *cfg)
{
    if (pll->mdiv_msk != 0U) {
        car_reg_rdwr_val(pll->mdiv_reg, pll->mdiv_msk, cfg->m);
    }
    if (pll->ndiv_msk != 0U) {
        car_reg_rdwr_val(pll->ndiv_reg, pll->ndiv_msk, cfg->n);
    }
    if (pll->pdiv_msk != 0U) {
        car_reg_rdwr_val(pll->pdiv_reg, pll->pdiv_msk, cfg->p);
    }
}

SECTION_CAR_TEXT
int64_t car_pll_hz_get(const struct car_pll_inf *pll, int64_t hz_in)
{
    struct car_pll_cfg cfg;
    error_t ret = E_CAR_NULL_PTR;

    if (pll != NULL) {
        car_pll_nmp_get(pll, &cfg);
        ret = car_pll_hz_calc(pll, &cfg, hz_in);
    }
    return (int64_t)ret;
}

SECTION_CAR_TEXT
const struct car_pll_cfg *car_pll_cfg_get(const struct car_pll_inf *pll,
                                          int64_t hz_in, int64_t hz_out)
{
    const struct car_pll_cfg *cfgs = *pll->cfgs;
    const struct car_pll_cfg *nearest = NULL;
    int64_t err = INT64_MAX;
    int64_t diff;
    uint32_t i;

    if (cfgs != NULL) {
        for (i = 0; (i < *pll->cfgs_n) && (err > 0); i = i + 1U) {
            if (cfgs[i].hz_in == hz_in) {
                diff = car_abs_s64(hz_out - cfgs[i].hz_out);
                if (diff <= err) {
                    nearest = &cfgs[i];
                    err = diff;
                }
            }
        }
    }

    return nearest;
}

SECTION_CAR_TEXT
error_t car_pll_select(const struct car_pll_inf *pll, uint32_t sel)
{
    error_t ret = E_CAR_NULL_PTR;

    if (pll != NULL) {
        if (pll->sel_msk != 0U) {
            car_reg_rdwr_val(pll->sel_reg, pll->sel_msk, sel);
        }
        ret = E_SUCCESS;
    }
    return ret;
}

SECTION_CAR_TEXT
bool car_pll_is_enabled(const struct car_pll_inf *pll)
{
    uint32_t en = false;

    if (pll != NULL) {
        if (pll->en_msk == 0U) {
            en = true;
        } else {
            en = (bool)(car_reg_rd(pll->en_reg) & pll->en_msk);
        }
    }
    return en;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

