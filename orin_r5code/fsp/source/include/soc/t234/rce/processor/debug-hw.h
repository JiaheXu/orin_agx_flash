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

#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>

/* Late FSP headers */
#include <processor/tegra-uart-hw-params.h>
#include <uart/tegra-uart-priv.h>


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

#ifdef TEGRA_UARTF_ENABLED
extern struct tegra_uart_ctlr tegra_uart_ctlr_uartf;

#ifndef DEBUG_UART
#define DEBUG_UART      tegra_uart_ctlr_uartf
#endif

#endif

#ifdef TEGRA_UARTC_ENABLED
extern struct tegra_uart_ctlr tegra_uart_ctlr_uartc;

#ifndef DEBUG_UART
#define DEBUG_UART      tegra_uart_ctlr_uartc
#endif

#endif

#endif
