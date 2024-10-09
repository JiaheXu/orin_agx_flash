/*
* Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>                         // for true
#include <stdint.h>                          // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                  // for CT_ASSERT
#include <soc-common/hw-const.h>             /* Must appear before any hwinc files */

/* Hardware headers */
#include <aruart.h>                          // for UART_LCR_0_WD_SIZE_SHIFT

/* Late FSP headers */
#include <clk/clk-tegra.h>                   // for tegra_clk_disable, tegra...
#include <error/common-errors.h>             // for E_SUCCESS, error_t, FSP_...
#include <misc/attributes.h>                 // for FSP__MISC__ATTRIBUTES_H
#include <misc/macros.h>                     // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <processor/tegra-uart-hw-params.h>  // for UART_CLOCKS, UART_CAR_AC...
#include <reg-access/reg-access.h>           // for readl, writel, FSP__REG_...

/* Module-specific FSP headers */
#include <uart/sections-uart.h>              // Immune from CT_ASSERT protection
#include <uart/tegra-uart-priv.h>            // for tegra_uart_ctlr, tegra_u...
#include <uart/tegra-uart.h>                 // for tegra_uart_conf, FSP__UA...
#include <uart/uart-errors.h>                // for E_UART_ERR_CONFIG, FSP__...

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
CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__TEGRA_UART_HW_PARAMS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__TEGRA_UART_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__TEGRA_UART_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__UART_ERRORS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define TEGRA_UART_LCR_DLAB             (UART_LCR_0_DLAB_ENABLE << UART_LCR_0_DLAB_SHIFT)
#define TEGRA_UART_IER_THRE             (UART_IER_DLAB_0_0_IE_THR_ENABLE << UART_IER_DLAB_0_0_IE_THR_SHIFT)
#define TEGRA_UART_LSR_RDY              (UART_LSR_0_RDR_DATA_IN_FIFO << UART_LSR_0_RDR_SHIFT)
#define TEGRA_UART_LSR_TX_FIFO_FULL     (UART_LSR_0_TX_FIFO_FULL_FULL << UART_LSR_0_TX_FIFO_FULL_SHIFT)
#define TEGRA_UART_LSR_THRE_EMPTY       (UART_LSR_0_THRE_EMPTY << UART_LSR_0_THRE_SHIFT)
#define TEGRA_UART_LSR_TX_SHIFT_EMPTY   (UART_LSR_0_TMTY_EMPTY << UART_LSR_0_TMTY_SHIFT)
#define TEGRA_UART_IIR_NO_INT           (UART_IIR_FCR_0_IS_STA_NO_INTR_PEND << UART_IIR_FCR_0_IS_STA_SHIFT)


static inline uint32_t
tegra_uart_readl(const struct tegra_uart_ctlr * const ctlr,
                 uint32_t offset)
{
    offset += ctlr->id.base_addr;

    return readl(offset);
}

static inline void
tegra_uart_writel(const struct tegra_uart_ctlr * const ctlr,
                  uint32_t val,
                  uint32_t offset)
{
        offset += ctlr->id.base_addr;
        writel(val, offset);
}

SECTION_UART_INIT_TEXT error_t
tegra_uart_init_hw(struct tegra_uart_ctlr * const ctlr,
                   const struct tegra_uart_conf * const conf)
{
    error_t     rc              = E_SUCCESS;
    uint32_t    fcr;
    uint32_t    divisor;
    uint32_t    comm_param;

#ifdef UART_CAR_ACCESS
    tegra_clk_enable(ctlr->id.hw_handle->clk);
    tegra_clk_reset_pulse(ctlr->id.hw_handle->rst, 10);
#endif

    /* Program the FCR register */
    fcr = (UART_IIR_FCR_0_FCR_EN_FIFO_ENABLE << UART_IIR_FCR_0_FCR_EN_FIFO_SHIFT);
    fcr |= (UART_IIR_FCR_0_TX_TRIG_FIFO_COUNT_GREATER_8 <<
            UART_IIR_FCR_0_TX_TRIG_SHIFT);
    fcr |= (UART_IIR_FCR_0_RX_TRIG_FIFO_COUNT_GREATER_4 <<
            UART_IIR_FCR_0_RX_TRIG_SHIFT);
    tegra_uart_writel(ctlr, fcr, UART_IIR_FCR_0);

#ifdef UART_CLOCKS
        tegra_clk_set_rate((ctlr->id.hw_handle->clk), conf->baud * 16);
#endif

    divisor = 1;

    switch (conf->parity) {
    case TEGRA_UART_NO_PARITY:
        comm_param = (UART_LCR_0_PAR_NO_PARITY << UART_LCR_0_PAR_SHIFT);
        break;
    case TEGRA_UART_ODD_PARITY:
        comm_param = (UART_LCR_0_EVEN_DISABLE << UART_LCR_0_EVEN_SHIFT) |
                     (UART_LCR_0_PAR_PARITY << UART_LCR_0_PAR_SHIFT);
        break;
    case TEGRA_UART_EVEN_PARITY:
        comm_param = (UART_LCR_0_EVEN_ENABLE << UART_LCR_0_EVEN_SHIFT) |
                     (UART_LCR_0_PAR_PARITY << UART_LCR_0_PAR_SHIFT);
        break;
    default:
        rc = E_UART_ERR_CONFIG;
        goto conf_err;
    }

    switch (conf->data_bits) {
    case TEGRA_UART_DATA_BITS_5:
        comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_5 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    case TEGRA_UART_DATA_BITS_6:
        comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_6 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    case TEGRA_UART_DATA_BITS_7:
        comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_7 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    case TEGRA_UART_DATA_BITS_8:
        comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_8 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    default:
        rc = E_UART_ERR_CONFIG;
        goto conf_err;
    }

    if (conf->stop_bits == TEGRA_UART_STOP_BITS_2)
        comm_param |= (UART_LCR_0_STOP_ENABLE << UART_LCR_0_STOP_SHIFT);

    tegra_uart_writel(ctlr, 0x0, UART_LCR_0);
    tegra_uart_writel(ctlr, 0x0, UART_IER_DLAB_0_0);

    tegra_uart_writel(ctlr, comm_param | TEGRA_UART_LCR_DLAB, UART_LCR_0);
    tegra_uart_writel(ctlr, divisor & 0xFF, UART_THR_DLAB_0_0);
    tegra_uart_writel(ctlr, ((divisor >> 8) & 0xFF), UART_IER_DLAB_0_0);
    tegra_uart_writel(ctlr, comm_param & ~TEGRA_UART_LCR_DLAB, UART_LCR_0);

    /* dummy read to ensure write is posted */
    tegra_uart_readl(ctlr, UART_SPR_0);

    ctlr->initialized = true;

  conf_err:
    if (rc != E_SUCCESS) {
#ifdef UART_CLOCKS
        tegra_clk_disable(ctlr->id.hw_handle->clk);
#endif
    }

    return rc;
}

SECTION_UART_TEXT void
tegra_uart_flush_tx_hw_fifo(const struct tegra_uart_ctlr * const ctlr)
{
    if (ctlr->initialized) {
        while ((tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_TX_SHIFT_EMPTY) == 0U) {
        }
    }
}

SECTION_UART_TEXT void
tegra_uart_write_char(const struct tegra_uart_ctlr * const ctlr,
                      const char c)
{
    if (!ctlr->initialized) {
        goto out;
    }

    while ((tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_THRE_EMPTY) == 0U) {
    }
    tegra_uart_writel(ctlr, c, UART_THR_DLAB_0_0);

  out:
    return;
}

SECTION_UART_TEXT void
tegra_uart_write(const struct tegra_uart_ctlr * const ctlr,
                 const char *buf,
                 uint32_t count)
{
    uint32_t    i;

    for (i = 0; i < count; i += 1U) {
        tegra_uart_write_char(ctlr, *buf++);
    }
}

SECTION_UART_TEXT void
tegra_uart_write_string(const struct tegra_uart_ctlr * const ctlr,
                        const char *buf)
{
    while (*buf != '\0') {
        tegra_uart_write_char(ctlr, *buf++);
    }
}

SECTION_UART_TEXT char
tegra_uart_read_char(const struct tegra_uart_ctlr * const ctlr)
{
    char        ch              = '\0';

    if (!ctlr->initialized) {
        goto out;
    }

    while ((tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_RDY) == 0U) {
    }

    ch = (char)tegra_uart_readl(ctlr, UART_THR_DLAB_0_0);

  out:
    return ch;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
