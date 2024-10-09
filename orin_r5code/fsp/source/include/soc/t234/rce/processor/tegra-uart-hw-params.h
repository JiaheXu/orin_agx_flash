/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef PROCESSOR__UART_TEGRA_HW_PARAMS_H
#define PROCESSOR__UART_TEGRA_HW_PARAMS_H
#define FSP__PROCESSOR__TEGRA_UART_HW_PARAMS_H          1

#include <misc/ct-assert.h>
#include <processor/irqs-hw.h>

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/macros.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")

#define TEGRA_UARTF_ENABLED
#define TEGRA_UARTF_IRQ         UINT32_MAX
#define UART_CLOCKS
#define UART_CAR_ACCESS

#endif
