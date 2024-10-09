/*
 * Copyright (c) 2016-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>

/* Late FSP headers */
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <uart/tegra-uart-priv.h>

/* Module-specific FSP header files */
#include <soc-common/clk-tegra-hw.h>
#include <port/uart-port-priv.h>
#include <processor/tegra-uart-hw-params.h>

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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__CLK_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__TEGRA_UART_HW_PARAMS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__TEGRA_UART_PRIV_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#ifdef TEGRA_UARTC_ENABLED
const struct uart_hw_handle uartc_hw_handle = {
        .clk = tegra_clk_uartc,
        .rst = tegra_rst_uartc,
};

struct tegra_uart_ctlr tegra_uart_ctlr_uartc = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTC_BASE,
                .irq = TEGRA_UARTC_IRQ,
                .hw_handle  = &uartc_hw_handle,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uartc __attribute__((alias("tegra_uart_ctlr_uartc")));
#endif

#ifdef TEGRA_UARTF_ENABLED
const struct uart_hw_handle uartf_hw_handle = {
        .clk = tegra_clk_uartf,
        .rst = tegra_rst_uartf,
};

struct tegra_uart_ctlr tegra_uart_ctlr_uartf = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTF_BASE,
                .irq = TEGRA_UARTF_IRQ,
                .hw_handle  = &uartf_hw_handle,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uartf __attribute__((alias("tegra_uart_ctlr_uartf")));
#endif

#ifdef TEGRA_UARTG_ENABLED
const struct uart_hw_handle uartg_hw_handle = {
        .clk = tegra_clk_uartg,
        .rst = tegra_rst_uartg,
};

struct tegra_uart_ctlr tegra_uart_ctlr_uartg = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTG_BASE,
                .irq = TEGRA_UARTG_IRQ,
                .hw_handle  = &uartg_hw_handle,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uartg __attribute__((alias("tegra_uart_ctlr_uartg")));
#endif

#ifdef TEGRA_UARTH_ENABLED
const struct uart_hw_handle uarth_hw_handle = {
        .clk = tegra_clk_uarth,
        .rst = tegra_rst_uarth,
};

struct tegra_uart_ctlr tegra_uart_ctlr_uarth = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTH_BASE,
                .irq = TEGRA_UARTH_IRQ,
                .hw_handle  = &uarth_hw_handle,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uarth __attribute__((alias("tegra_uart_ctlr_uarth")));
#endif
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
