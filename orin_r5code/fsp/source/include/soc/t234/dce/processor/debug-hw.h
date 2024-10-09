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

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
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
CT_ASSERT(FSP__PROCESSOR__TEGRA_UART_HW_PARAMS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__TEGRA_UART_PRIV_H, "Header file missing or invalid.")

#define DEBUG_COMB_UART      comb_uart_id_dce

extern const struct comb_uart_id comb_uart_id_dce;

#ifdef TEGRA_UARTA_ENABLED
extern struct tegra_uart_ctlr tegra_uart_ctlr_uarta;

#ifndef DEBUG_UART
#define DEBUG_UART      tegra_uart_ctlr_uarta
#endif
#define TEGRA_DBG_UART_BASE     NV_ADDRESS_MAP_UARTA_BASE

#endif

#ifdef TEGRA_UARTC_ENABLED
extern struct tegra_uart_ctlr tegra_uart_ctlr_uartc;

#ifndef DEBUG_UART
#define DEBUG_UART      tegra_uart_ctlr_uartc
#endif
#define TEGRA_DBG_UART_BASE     NV_ADDRESS_MAP_UARTC_BASE

#endif

#endif
