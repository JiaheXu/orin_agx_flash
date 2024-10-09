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

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <soc-common/hw-const.h>        /* Must appear before any hwinc files */

/* Hardware headers */
#include <arclk_rst.h>                  // for CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0...

/* Late FSP headers */
#include <misc/macros.h>                // for ARRAY_SIZE,...
#include <chipid/chip-id.h>             // for tegra_platform_is_silicon...
#include <delay/delay.h>                // for udelay...

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-errors.h>             // for E_CAR_INVALID_RESET_ID,_...
#include <car/car-rst.h>                // for core functions, ...
#include <port/car-port.h>              // for car_port_udelay...
#include <port/car-rst.h>               // for reset index, ...

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
CT_ASSERT(FSP__CHIPID__CHIP_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DELAY__DELAY_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_RST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__FSI__CAR_RST_H, "Header file missing or invalid.")


SECTION_CAR_RODATA
static const struct car_rst_inf fsi_rst_infs[] = {
    /* fsi_ncorereset0_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCORE_0,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCORERESET0_RST_FIELD),
    /* fsi_ncorereset1_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCORE_1,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCORERESET1_RST_FIELD),
    /* fsi_ncorereset2_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCORE_2,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCORERESET2_RST_FIELD),
    /* fsi_ncorereset3_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCORE_3,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCORERESET3_RST_FIELD),
    /* fsi_ncpuporreset0_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCPUPOR_0,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCPUPORRESET0_RST_FIELD),
    /* fsi_ncpuporreset1_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCPUPOR_1,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCPUPORRESET1_RST_FIELD),
    /* fsi_ncpuporreset2_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCPUPOR_2,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCPUPORRESET2_RST_FIELD),
    /* fsi_ncpuporreset3_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NCPUPOR_3,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NCPUPORRESET3_RST_FIELD),
    /* fsi_ntopreset_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NTOPRESET,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NTOPRESET_RST_FIELD),
    /* fsi_npresetdbg_reset_fsi_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_NPRESETDBG,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CPU_0_SWR_FSI_NPRESETDBG_RST_FIELD),
    /* fsi_chsm_sysporreset_reset_fsi_chsm_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CHSM_SYSPORRESET,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0_SWR_CHSM_SYSPORESET_RST_FIELD),
    /* fsi_chsm_nreset_reset_fsi_chsm_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CHSM_NRESET,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0_SWR_CHSM_NRESET_RST_FIELD),
    /* fsi_chsm_dbgresetn_reset_fsi_chsm_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CHSM_DBGRESETN,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0_SWR_CHSM_DBGRESETN_RST_FIELD),
    /* fsi_chsm_presetdbgn_reset_fsi_chsm_cpu_clk_srstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CHSM_PRESETDBGN,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CHSM_0_SWR_CHSM_PRESETDBGN_RST_FIELD),
    /* fsi_csite_reset_rstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CSITE,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_DFD_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_DFD_0_SWR_FSI_CSITE_RST_FIELD),
    /* fsi_la_reset_rstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_LA,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_DFD_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_DFD_0_SWR_FSI_LA_RST_FIELD),
    /* fsi_can0_reset_rstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CAN0,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CAN_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CAN_0_SWR_FSI_CAN0_RST_FIELD),
    /* fsi_can1_reset_rstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_CAN1,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CAN_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_CAN_0_SWR_FSI_CAN1_RST_FIELD),
    /* fsi_spi0_reset_rstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_SPI0,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_SPI_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_SPI_0_SWR_FSI_SPI0_RST_FIELD),
    /* fsi_uart0_reset_rstn */
    CAR_RST_TYPE_R(FSI_CAR_RST_ID_UART0,
                   0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_UART_0,
                   CLK_RST_CONTROLLER_RST_DEV_FSI_UART_0_SWR_FSI_UART0_RST_FIELD),
};


SECTION_CAR_TEXT
static struct car_rst_inf *rst_inf_get(uint32_t rst_id)
{
    struct car_rst_inf *rst = NULL;

    if ((rst_id < ARRAY_SIZE(fsi_rst_infs)) && (fsi_rst_infs[rst_id].op != NULL)) {
        rst = (struct car_rst_inf *)&fsi_rst_infs[rst_id];
    }
    return rst;
}

SECTION_CAR_TEXT
bool fsi_rst_is_asserted(uint32_t rst_id)
{
    bool ret = false;
    struct car_rst_inf *rst = rst_inf_get(rst_id);

    if (rst != NULL) {
        ret = rst->op->rst_is_asserted(rst);
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_rst_assert(uint32_t rst_id)
{
    error_t ret = E_CAR_INVALID_RESET_ID;
    struct car_rst_inf *rst = rst_inf_get(rst_id);

    if (rst != NULL) {
        ret = rst->op->rst_assert(rst);
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_rst_deassert(uint32_t rst_id)
{
    error_t ret = E_CAR_INVALID_RESET_ID;
    struct car_rst_inf *rst = rst_inf_get(rst_id);

    if (rst != NULL) {
        ret = rst->op->rst_deassert(rst);
    }
    return ret;
}

SECTION_CAR_TEXT
error_t fsi_rst_pulse(uint32_t rst_id, uint64_t delay_us)
{
    error_t ret = E_CAR_INVALID_RESET_ID;
    struct car_rst_inf *rst = rst_inf_get(rst_id);

    if (rst != NULL) {
        ret = rst->op->rst_assert(rst);
        car_port_udelay(delay_us);
        ret |= rst->op->rst_deassert(rst);
    }
    return ret;
}

