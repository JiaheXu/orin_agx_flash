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
#include <stddef.h>                          // for NULL

/* Early FSP headers */
#include <misc/ct-assert.h>                  // for CT_ASSERT
#include <soc-common/hw-const.h>             // Must appear before any hwinc files

/* Hardware headers */
#include <address_map_new.h>                 // for NV_ADDRESS_MAP_UARTF_BASE

/* Late FSP headers */
#include <misc/macros.h>                     // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <processor/tegra-uart-hw-params.h>  // for TEGRA_UARTF_ENABLED, TEG...
#include <uart/tegra-uart-priv.h>            // for FSP__UART__TEGRA_UART_PR...

/* Module-specific FSP header files */
#include <soc-common/clk-tegra-hw.h>         // for FSP__SOC_COMMON__CLK_TEG...


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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__TEGRA_UART_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__CLK_TEGRA_HW_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#ifdef TEGRA_UARTA_ENABLED
#ifdef UART_CLOCKS
const struct uart_hw_handle uarta_hw_handle = {
        .clk = tegra_clk_uarta,
        .rst = tegra_rst_uarta,
};
#endif

struct tegra_uart_ctlr tegra_uart_ctlr_uarta = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTA_BASE,
#ifdef UART_CLOCKS
                .hw_handle  = &uarta_hw_handle,
#else
                .hw_handle = NULL,
#endif
                .irq = TEGRA_UARTA_IRQ,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uarta __attribute__((alias("tegra_uart_ctlr_uarta")));
#endif

#ifdef TEGRA_UARTC_ENABLED
#ifdef UART_CLOCKS
const struct uart_hw_handle uartc_hw_handle = {
        .clk = tegra_clk_uartc,
        .rst = tegra_rst_uartc,
};
#endif

struct tegra_uart_ctlr tegra_uart_ctlr_uartc = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTC_BASE,
#ifdef UART_CLOCKS
                .hw_handle  = &uartc_hw_handle,
#else
                .hw_handle = NULL,
#endif
                .irq = TEGRA_UARTC_IRQ,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uartc __attribute__((alias("tegra_uart_ctlr_uartc")));
#endif

#ifdef TEGRA_UARTF_ENABLED
#ifdef UART_CLOCKS
const struct uart_hw_handle uartf_hw_handle = {
        .clk = tegra_clk_uartf,
        .rst = tegra_rst_uartf,
};
#endif

struct tegra_uart_ctlr tegra_uart_ctlr_uartf = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTF_BASE,
#ifdef UART_CLOCKS
                .hw_handle  = &uartf_hw_handle,
#else
                .hw_handle = NULL,
#endif
                .irq = TEGRA_UARTF_IRQ,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uartf __attribute__((alias("tegra_uart_ctlr_uartf")));
#endif

#ifdef TEGRA_UARTH_ENABLED
#ifdef UART_CLOCKS
const struct uart_hw_handle uarth_hw_handle = {
        .clk = tegra_clk_uarth,
        .rst = tegra_rst_uarth,
};
#endif

struct tegra_uart_ctlr tegra_uart_ctlr_uarth = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTH_BASE,
#ifdef UART_CLOCKS
                .hw_handle  = &uarth_hw_handle,
#else
                .hw_handle = NULL,
#endif
                .irq = TEGRA_UARTH_IRQ,
        },
        .initialized = false,
};

extern struct tegra_uart_id tegra_uart_id_uarth __attribute__((alias("tegra_uart_ctlr_uarth")));

#endif

#ifdef TEGRA_UARTJ_ENABLED
#ifdef UART_CLOCKS
const struct uart_hw_handle uartj_hw_handle = {
        .clk = tegra_clk_uartj,
        .rst = tegra_rst_uartj,
};
#endif

struct tegra_uart_ctlr tegra_uart_ctlr_uartj = {
        .id = {
                .base_addr = NV_ADDRESS_MAP_UARTJ_BASE,
#ifdef UART_CLOCKS
                .hw_handle  = &uartj_hw_handle,
#else
                .hw_handle = NULL,
#endif
                .irq = TEGRA_UARTJ_IRQ,
        },
        .initialized = false,
};
extern struct tegra_uart_id tegra_uart_id_uartj __attribute__((alias("tegra_uart_ctlr_uartj")));
#endif

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
