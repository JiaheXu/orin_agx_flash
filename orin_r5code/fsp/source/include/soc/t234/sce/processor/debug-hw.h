/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef PROCESSOR__DEBUG_HW_H
#define PROCESSOR__DEBUG_HW_H
#define FSP__PROCESSOR__DEBUG_HW_H                      1

#include <address_map_new.h>

#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>

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

#ifdef _NV_BUILD_LINSIM_
#define TEGRA_DBG_UART_BASE     NV_ADDRESS_MAP_UARTA_BASE
#elif defined(_NV_BUILD_FPGA_)
#define TEGRA_DBG_UART_BASE     NV_ADDRESS_MAP_UARTC_BASE
#else
#define TEGRA_DBG_UART_BASE     NV_ADDRESS_MAP_UARTC_BASE
#define TEGRA_DBG_UART_CLK      tegra_clk_uartc
#define TEGRA_DBG_UART_RST      tegra_rst_uartc
#define TEGRA_DBG_UART_BAUD     115200
#endif

#endif
