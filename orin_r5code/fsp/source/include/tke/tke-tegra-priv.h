/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TKE__TKE_TEGRA_PRIV_H
#define TKE__TKE_TEGRA_PRIV_H
#define FSP__TKE__TKE_TEGRA_PRIV_H                      1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <tke/tke-tegra.h>

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

extern const uint32_t tke_top_base;

#define TEGRA_TKE_NO_IRQ (~UINT32_C(0))

/**
 * @brief HW details of the TKE Timer.
 *
 * It contains the cluster and SOC-specific details of the timer. These details
 * are present in the cluster specific tke-tegra-ids.c. This structure is used
 * to communicate with the tke HW via the registers. Note that, no public api
 * directly uses this structure to interface with a timer unit.
 *
 * @base_addr   Base address of TKE Timer
 * @irq TKE     Timer IRQ number
 */
struct tegra_tke_conf
{
	uint32_t base_addr;
    uint32_t irq;
};

/**
 * @brief Tegra specific details of the TKE Timer.
 *
 * This structure contains the SW and HW details of the timer. The tke unit apis
 * use this structure to interface with the timer HW and the SW APIs registered
 * to the timers.
 *
 * @conf        Configuration for the tke context
 * @callback    TKE Timer IRQ callback function.
 * @data        Opaque data pointer for the timer callback function.
 */
struct tegra_tke_id
{
    struct tegra_tke_conf const conf;
    tegra_tke_timer_callback callback;
    void *data;
};

#endif  /* TKE__TKE_TEGRA_PRIV_H */
