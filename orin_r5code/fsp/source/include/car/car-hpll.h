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

#ifndef CAR__CAR_HPLL_H
#define CAR__CAR_HPLL_H
#define FSP__CAR__CAR_HPLL_H            1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...

/* Module-specific FSP headers */
#include <car/car-pll.h>                // for pll functions, ...

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

/**
 * @file car/car-hpll.h
 * @brief Structures that are internal to the CAR driver
 */

extern const struct car_pll_op car_pll_op_hpll;

/**
 * @brief Define HPLL information
 *
 * Fills the PLL configuration structure with the info
 * associated with the PLL.
 */
#define HPLL_INF(_i, _spec, _cfg, _cfg_n, _sts,                                     \
                 _sel_r, _sel_m, _rst_r, _rst_m, _en_r, _en_m, _iddq_r, _iddq_m,    \
                 _mdiv_r, _mdiv_m, _ndiv_r, _ndiv_m, _pdiv_r, _pdiv_m,              \
                 _frac_r, _frac_m, _enfrac_r, _enfrac_m, _fracstep_r, _fracstep_m,  \
                 _iref_r, _iref_m, _lock_r, _lock_m, _lckdet_r, _lckdet_m,          \
                 _freqlock_m, _ss_cntl_r)                                           \
    [_i] = {                                                                        \
        .op = &car_pll_op_hpll,                                                     \
        .spec = _spec,                                                              \
        .cfgs = _cfg,                                                               \
        .cfgs_n = _cfg_n,                                                           \
        .sts = _sts,                                                                \
        .sel_reg = _sel_r,                                                          \
        .sel_msk = _sel_m,                                                          \
        .rst_reg = _rst_r,                                                          \
        .rst_msk = _rst_m,                                                          \
        .en_reg = _en_r,                                                            \
        .en_msk = _en_m,                                                            \
        .iddq_reg = _iddq_r,                                                        \
        .iddq_msk = _iddq_m,                                                        \
        .mdiv_reg = _mdiv_r,                                                        \
        .mdiv_msk = _mdiv_m,                                                        \
        .ndiv_reg = _ndiv_r,                                                        \
        .ndiv_msk = _ndiv_m,                                                        \
        .pdiv_reg = _pdiv_r,                                                        \
        .pdiv_msk = _pdiv_m,                                                        \
        .frac_reg = _frac_r,                                                        \
        .frac_msk = _frac_m,                                                        \
        .enfrac_reg = _enfrac_r,                                                    \
        .enfrac_msk = _enfrac_m,                                                    \
        .fracstep_reg = _fracstep_r,                                                \
        .fracstep_msk = _fracstep_m,                                                \
        .iref_reg = _iref_r,                                                        \
        .iref_msk = _iref_m,                                                        \
        .lock_reg = _lock_r,                                                        \
        .lock_msk = _lock_m,                                                        \
        .lckdet_reg = _lckdet_r,                                                    \
        .lckdet_msk = _lckdet_m,                                                    \
        .freqlock_msk = _freqlock_m,                                                \
        .ss_cntl_reg = _ss_cntl_r,                                                  \
    }
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* CAR__CAR_HPLL_H */

