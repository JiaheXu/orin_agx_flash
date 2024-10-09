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

#ifndef CAR__CAR_PLL_H
#define CAR__CAR_PLL_H
#define FSP__CAR__CAR_PLL_H             1

/**
 * @file car/car-fsi-priv.h
 * @brief Structures that are internal to the CAR driver
 */

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for error_t
#include <misc/bitops.h>                // for BIT, bit_number, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")

/**
 * @file car/car-pll.h
 * @brief Structures that are internal to the CAR driver
 */
#define CAR_PLL_LOCK_DELAY              (50)

#define CAR_PLL_FLG_USE_MID_VCO         BIT(0)
#define CAR_PLL_FLG_ROUND_DOWN          BIT(1)
#define CAR_PLL_FLG_FRAC_NDIV           BIT(2)
#define CAR_PLL_FLG_HAS_DYNAMIC_RAMP    BIT(3)

#define CAR_PLL_STS_HZ_SET              BIT(4)

#define CAR_PLL_SEL_XTAL                (0)
#define CAR_PLL_SEL_PLL                 (1)

enum car_pll_type {
    CAR_PLL_TYPE_ANALOG = 1,
    CAR_PLL_TYPE_HPLL12 = 2,
    CAR_PLL_TYPE_HPLL16 = 3,
    CAR_PLL_TYPE_HPLL40 = 4,
};

struct car_pll_spec {
    enum car_pll_type type;
    uint32_t flags;
    uint32_t lock_time;
    uint32_t frac_width;
    uint32_t min_n;
    uint32_t max_n;
    uint32_t min_m;
    uint32_t max_m;
    uint32_t min_p;
    uint32_t max_p;
    int64_t cf_min;
    int64_t cf_max;
    int64_t vco_min;
    int64_t vco_max;
    int64_t clkin_max;
};

struct car_pll_nmp {
    uint32_t n;
    uint32_t m;
    uint32_t p;
    int32_t frac;
};

/**
 * @brief Structure defining PLL values for a specific
 *        frequency.
 */
struct car_pll_cfg {
    int64_t hz_in;
    int64_t hz_out;
    uint32_t n;
    uint32_t m;
    uint32_t p;
    bool en_sdm;
    bool en_ssc;
    int16_t sdm_din;
    int16_t sdm_ssc_min;
    int16_t sdm_ssc_max;
    int16_t sdm_ssc_step;
    uint16_t sscmax;
    uint8_t sscinc;
    uint8_t sscincintrv;
};

/**
 * @brief Define PLL frequency
 *
 * Fills the PLL configuration structure with the info
 * associated with the PLL.
 */
#define PLL_CFG(_i, _hz_in, _hz_out, _n, _m, _p,                              \
                _en_sdm, _sdm_din, _sdm_ssc_min, _sdm_ssc_max, _sdm_ssc_step, \
                _en_ssc, _sscmax, _sscinc, _sscincintrv)                      \
    [_i] = {                                                                  \
        .hz_in = _hz_in,                                                      \
        .hz_out = _hz_out,                                                    \
        .n = _n,                                                              \
        .m = _m,                                                              \
        .p = _p,                                                              \
        .en_sdm = _en_sdm,                                                    \
        .en_ssc = _en_ssc,                                                    \
        .sdm_din = _sdm_din,                                                  \
        .sdm_ssc_min = _sdm_ssc_min,                                          \
        .sdm_ssc_max = _sdm_ssc_max,                                          \
        .sdm_ssc_step = _sdm_ssc_step,                                        \
        .sscmax = _sscmax,                                                    \
        .sscinc = _sscinc,                                                    \
        .sscincintrv = _sscincintrv,                                          \
    }

struct car_pll_op;

struct car_pll_inf {
    const struct car_pll_op *op;
    const struct car_pll_spec **spec;
    const struct car_pll_cfg **cfgs;
    uint32_t *cfgs_n;
    uint32_t *sts;
    uint32_t sel_reg;
    uint32_t sel_msk;
    uint32_t rst_reg;
    uint32_t rst_msk;
    uint32_t en_reg;
    uint32_t en_msk;
    uint32_t iddq_reg;
    uint32_t iddq_msk;
    uint32_t mdiv_reg;
    uint32_t mdiv_msk;
    uint32_t ndiv_reg;
    uint32_t ndiv_msk;
    uint32_t pdiv_reg;
    uint32_t pdiv_msk;
    uint32_t frac_reg;
    uint32_t frac_msk;
    uint32_t enfrac_reg;
    uint32_t enfrac_msk;
    uint32_t fracstep_reg;
    uint32_t fracstep_msk;
    uint32_t iref_reg;
    uint32_t iref_msk;
    uint32_t lock_reg;
    uint32_t lock_msk;
    uint32_t lckdet_reg;
    uint32_t lckdet_msk;
    uint32_t freqlock_msk;
    uint32_t ss_cntl_reg;
};

/**
 * @brief Structure defining PLL operations
 *
 */
struct car_pll_op {
    bool (*frac_allowed)(const struct car_pll_inf *pll);
    bool (*pll_is_enabled)(const struct car_pll_inf *pll);
    error_t (*pll_enable)(const struct car_pll_inf *pll);
    error_t (*pll_disable)(const struct car_pll_inf *pll);
    int64_t (*pll_hz_set)(const struct car_pll_inf *pll, int64_t hz_in, int64_t hz_out);
    int64_t (*pll_hz_get)(const struct car_pll_inf *pll, int64_t hz_in);
};


error_t car_pll_wait_lock_us(const struct car_pll_inf *pll, uint64_t t_us);
error_t car_pll_wait_lock(const struct car_pll_inf *pll);
int64_t car_pll_hz_calc(const struct car_pll_inf *pll,
                        struct car_pll_cfg *cfg, int64_t hz_in);
void car_pll_nmp_get(const struct car_pll_inf *pll, struct car_pll_cfg *cfg);
void car_pll_nmp_set(const struct car_pll_inf *pll, struct car_pll_cfg *cfg);
error_t car_pll_nmp_calc(const struct car_pll_spec *spec, struct car_pll_nmp *nmp,
                         int64_t hz_in, int64_t f_in, int32_t p_pref, uint32_t flags);
const struct car_pll_cfg *car_pll_cfg_get(const struct car_pll_inf *pll,
                                          int64_t hz_in, int64_t hz_out);
error_t car_pll_cfg_calc(const struct car_pll_inf *pll,
                         int64_t hz_in, int64_t hz_out, int64_t hz_max,
                         struct car_pll_cfg *cfg, int32_t p_curr);
int64_t car_pll_hz_get(const struct car_pll_inf *pll, int64_t hz_in);
error_t car_pll_select(const struct car_pll_inf *pll, uint32_t sel);
bool car_pll_is_enabled(const struct car_pll_inf *pll);
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* CAR__CAR_PLL_H */

