/* Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <soc-common/hw-const.h>        // Must appear before any hwinc files

/* Hardware headers */
#include <arclk_rst.h>                  // for CLK_RST_CONTROLLER_?_BASE_0...

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <error/common-errors.h>        // for E_SUCCESS, error_t

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_ERR_NULL_PTR, E_CAR_ERR_NO_...
#include <car/car-reg.h>                // for register access functions...
#include <car/car-math.h>               // for math functions, ...
#include <car/car-pll.h>                // for pll functions, ...
#include <car/car-hpll.h>               // for hpll functions, ...
#include <port/car-port.h>              // for car_port_udelay...

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_REG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_MATH_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_PLL_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_HPLL_H, "Header file missing or invalid.")


SECTION_CAR_TEXT
static error_t car_hpll_disable(const struct car_pll_inf *pll)
{
    error_t ret = E_CAR_NULL_PTR;

    if (pll != NULL) {
        ret = car_pll_select(pll, CAR_PLL_SEL_XTAL);
        if (ret == E_SUCCESS) {
            car_reg_rdwr(pll->en_reg, 0U, pll->en_msk);
            car_reg_rdwr(pll->iddq_reg, pll->iddq_msk, pll->iddq_msk);
            car_port_udelay(2ULL);
            car_reg_rdwr(pll->rst_reg, 0U, pll->rst_msk);
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static error_t car_hpll_enable(const struct car_pll_inf *pll)
{
    error_t ret = E_CAR_NULL_PTR;

    if (pll != NULL) {
        if ((*pll->sts & CAR_PLL_STS_HZ_SET) == CAR_PLL_STS_HZ_SET) {
            car_reg_rdwr(pll->iddq_reg, 0U, pll->iddq_msk);
            car_port_udelay(5ULL);
            car_reg_rdwr(pll->rst_reg, pll->rst_msk, pll->rst_msk);
            car_reg_rdwr(pll->en_reg, pll->en_msk, pll->en_msk);
            if (car_reg_rd_val(pll->mdiv_reg, pll->mdiv_msk) == MK_U32_CONST(1)) {
                car_port_udelay(50ULL);
            } else {
                car_port_udelay(100ULL);
            }
            ret = car_pll_wait_lock(pll);
            if (ret == E_SUCCESS) {
                ret = car_pll_select(pll, CAR_PLL_SEL_PLL);
            }
        } else {
            ret = E_CAR_PLL_NO_INIT;
        }
    }
    return ret;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_CAR_TEXT static void car_hpll_fracstep_set(const struct car_pll_inf *pll, struct car_pll_cfg *cfg_cur, struct car_pll_cfg *cfg_new)
{
    int16_t dx;
    int32_t ndivf;
    int32_t fstep;
    int32_t s;
    uint32_t val;

    if (pll->fracstep_msk != 0U) {
        dx = cfg_new->sdm_din - cfg_cur->sdm_din;
        if ((cfg_new->n != cfg_cur->n) && (dx != 0)) {
            ndivf = cfg_new->sdm_din;
        } else {
            ndivf = dx;
        }
        if (ndivf != 0) {
            s = (ndivf < 0) ? -1 : 1;
            fstep = car_abs_s32(ndivf) / 4;
            fstep = s * max(fstep, 1000);
        } else {
            fstep = 0;
        }
        val = (uint32_t)fstep;
        car_reg_rdwr_val(pll->fracstep_reg, pll->fracstep_msk, val);
    }
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_CAR_TEXT static void car_hpll_frac_set(const struct car_pll_inf *pll, struct car_pll_cfg *cfg_new)
{
    uint32_t val;

    if (pll->frac_msk != 0U) {
        val = (uint32_t)cfg_new->sdm_din;
        car_reg_rdwr_val(pll->frac_reg, pll->frac_msk, val);
        if (pll->enfrac_msk != 0U) {
            if (val == 0U) {
                car_reg_rdwr_val(pll->enfrac_reg, pll->enfrac_msk, val);
            } else {
                car_reg_rdwr_val(pll->enfrac_reg, 0U, pll->enfrac_msk);
            }
        }
    }
}

SECTION_CAR_TEXT
static void car_hpll_freq_set(const struct car_pll_inf *pll,
                              struct car_pll_cfg *cfg_cur, struct car_pll_cfg *cfg_new)
{
    car_hpll_fracstep_set(pll, cfg_cur, cfg_new);
    car_pll_nmp_set(pll, cfg_new);
    car_hpll_frac_set(pll, cfg_new);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_CAR_TEXT static bool car_hpll_nmp_is_equal(struct car_pll_cfg *a, struct car_pll_cfg *b)
{
    return (a->n == b->n) &&
           (a->m == b->m) &&
           (a->p == b->p) &&
           (a->sdm_din == b->sdm_din);
}

SECTION_CAR_TEXT
static error_t car_hpll_nmp_set (const struct car_pll_inf *pll,
                                 struct car_pll_cfg *cfg_cur, struct car_pll_cfg *cfg_new)
{
    uint32_t newp;
    error_t ret = E_SUCCESS;

    if (car_hpll_nmp_is_equal(cfg_cur, cfg_new)) {
        /* nothing */
    } else if (!pll->op->pll_is_enabled(pll)) {
        car_hpll_freq_set (pll, cfg_cur, cfg_new);
    } else if (cfg_new->p >= cfg_cur->p) {
        car_hpll_freq_set (pll, cfg_cur, cfg_new);
        ret = car_pll_wait_lock(pll);
    } else {
        newp = cfg_new->p;
        cfg_new->p = cfg_cur->p;
        car_hpll_freq_set (pll, cfg_cur, cfg_new);
        ret = car_pll_wait_lock(pll);
        if (ret == E_SUCCESS) {
            cfg_new->p = newp;
            car_pll_nmp_set(pll, cfg_new);
        }
    }
    return ret;
}

static error_t car_hpll_nmp_calc(const struct car_pll_inf *pll,
                                 struct car_pll_cfg *cfg_cur, struct car_pll_cfg *cfg_new,
                                 int64_t hz_in, int64_t hz_out)
{
    const struct car_pll_spec *spec = *pll->spec;
    const bool en = pll->op->pll_is_enabled(pll);
    const struct car_pll_cfg *cfg;
    int32_t p;
    error_t ret;

    if (en && !(spec->flags & CAR_PLL_FLG_HAS_DYNAMIC_RAMP)) {
        ret = E_CAR_CLOCK_OP_NOT_SUPPORTED;
    } else {
        car_pll_nmp_get(pll, cfg_cur);
        cfg = car_pll_cfg_get(pll, hz_in, hz_out);
        if (cfg != NULL) {
            *cfg_new = *cfg;
            ret = E_SUCCESS;
        } else {
            p = en ? (int32_t)cfg_cur->p : -1;
            ret = car_pll_cfg_calc(pll, hz_in, hz_out, INT64_MAX, cfg_new, p);
        }
    }
    return ret;
}

SECTION_CAR_TEXT
static int64_t car_hpll_hz_set(const struct car_pll_inf *pll,
                               int64_t hz_in, int64_t hz_out)
{
    struct car_pll_cfg cfg_cur;
    struct car_pll_cfg cfg_new;
    error_t ret;
    int64_t hz_ret = E_CAR_NULL_PTR;

    if (pll != NULL) {
        memset(&cfg_cur, 0, sizeof(cfg_cur));
        memset(&cfg_new, 0, sizeof(cfg_new));
        ret = car_hpll_nmp_calc(pll, &cfg_cur, &cfg_new, hz_in, hz_out);
        if (ret == E_SUCCESS) {
            ret = car_hpll_nmp_set (pll, &cfg_cur, &cfg_new);
        }
        if (ret == E_SUCCESS) {
            hz_ret = car_pll_hz_calc(pll, &cfg_new, hz_in);
            *pll->sts |= CAR_PLL_STS_HZ_SET;
        } else {
            hz_ret = (int64_t)ret;
        }
    }
    return hz_ret;
}

SECTION_CAR_RODATA
const struct car_pll_op car_pll_op_hpll = {
    .frac_allowed = NULL,
    .pll_is_enabled = car_pll_is_enabled,
    .pll_enable = car_hpll_enable,
    .pll_disable = car_hpll_disable,
    .pll_hz_set = car_hpll_hz_set,
    .pll_hz_get = car_pll_hz_get,
};
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

