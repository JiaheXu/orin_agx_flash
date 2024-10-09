/*
 * Copyright (c) 2015-2020 NVIDIA CORPORATION.  All rights reserved.
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
#include <stdint.h>                 // for uint64_t, uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>        // for NV_ADDRESS_MAP_TSC_IMPL_BASE

/* Late FSP headers */
#include <misc/bitops.h>            // for hilo_to_64, FSP__MISC__BITOPS_H
#include <reg-access/reg-access.h>  // for readl, FSP__REG_ACCESS__REG_ACCESS_H

/* Module-specific FSP headers */
#include <tke/sections-tke.h>       // Immune from CT_ASSERT protection
#include <tke/tke-tegra-regs.h>     // for FSP__TKE__TKE_TEGRA_REGS_H, TSC_B...
#include <tke/tke-tegra.h>          // for tegra_tke_get_tsc64, FSP__TKE__TK...

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
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")

static uint32_t cntfid0         SECTION_TKE_DATA;
static uint32_t cntperiod       SECTION_TKE_DATA;

/* TSC timestamp at nanosecond resolution */
SECTION_TKE_TEXT uint64_t
tegra_tke_get_tsc_ns(void)
{
        /* TSC (cntfid0) is usually 31.25MHz, cntperiod is 32 ns */
        return tegra_tke_get_tsc64() * cntperiod;
}

/* Convert TSC timestamp to nanosecond resolution */
SECTION_TKE_TEXT uint64_t
tegra_tke_tsc_to_ns(uint64_t tsc)
{
        /* TSC is 31.25MHz = 32 ns */
        return tsc * cntperiod;
}

SECTION_TKE_TEXT uint64_t
tegra_tke_get_elapsed_usec(uint32_t prev_tsc_hi, uint32_t prev_tsc_low)
{
        uint64_t result, curr_tsc, prev_tsc;

        curr_tsc = tegra_tke_get_tsc64();
        prev_tsc = hilo_to_64(prev_tsc_hi, prev_tsc_low);

        /* cntfid0 holds the base frequency of the tsc.
         * This will always be the frequency tsc runs.
         * There is no plan to change it for tegra
         */
        result = (curr_tsc - prev_tsc) * cntperiod / 1000;

        return result;
}

/**
 * tegra_tke_get_elapsed_usecs64()  - Return elapsed microseconds
 *
 * @start:      value of TSC at the start of the period of when
 *              the elapsed time is being computed.
 *
 * Return Value:
 *      the number of microseconds that have elapsed between when
 *      the start time was taken and this function called.  This is
 *      a 64-bit value.
 */
SECTION_TKE_TEXT uint64_t
tegra_tke_get_elapsed_usecs64(const uint64_t start)
{
    uint64_t    elapsed;
    uint64_t    now;
    uint64_t    elapsed_tsc;

    now = tegra_tke_get_tsc64();

    elapsed_tsc = now - start;

    elapsed = (elapsed_tsc * cntperiod) / 1000;

    return elapsed;
}

SECTION_TKE_INIT_TEXT void
tegra_tsc_init(void)
{
        uint32_t m0, r0, d0;
        uint32_t val;

        val = readl(NV_ADDRESS_MAP_TSC_IMPL_BASE + TSC_MTSCANNR_0);

        m0 = (val & TSC_MTSCANNR_0_M0_MASK) >> TSC_MTSCANNR_0_M0_SHIFT;
        r0 = (val & TSC_MTSCANNR_0_R0_MASK) >> TSC_MTSCANNR_0_R0_SHIFT;

        val = readl(NV_ADDRESS_MAP_TSC_IMPL_BASE + TSC_MTSCANDR_0);
        d0 = (val & TSC_MTSCANDR_0_D0_MASK) >> TSC_MTSCANDR_0_D0_SHIFT;

        cntfid0 = (TSC_BASE_RATE / d0) * (d0 * r0 + m0);
        cntperiod = 1000000000U / cntfid0;
}
