/*
 * Copyright (c) 2021-2022 NVIDIA CORPORATION.  All rights reserved.
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
#include <stddef.h>                          // for NULL

/* HW headers */

/* Early FSP headers */
#include <misc/ct-assert.h>                  // for CT_ASSERT
#include <soc-common/hw-const.h>             /* Must appear before any hwinc files */

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>             // for E_SUCCESS, error_t, FSP_...
#include <misc/attributes.h>                 // for FSP__MISC__ATTRIBUTES_H
#include <misc/bitops.h>                     // for BIT
#include <reg-access/reg-access.h>           // for readl, writel, FSP__REG_...
#include <irq/irqs.h>                        // for irq_enable, irq_disable...
#include <delay/delay.h>                     // for get_time_ticks

/* Module-specific FSP headers */
#include <uart/sections-uart.h>              // Immune from CT_ASSERT protection
#include <uart/uart-errors.h>                // for E_UART_ERR_CONFIG, ...
#include <uart/uart.h>                       // for tegra_uart_conf, ...
#include <uart/tegra-uart-priv.h>            // for tegra_uart_ctlr, tegra_u...
#include <uart/uart-port.h>                  // for uart_port_...
#include <uart-sbsa/uart-sbsa.h>             // for sbsa_uart_...
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
    CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")

#define SBSA_UARTDR_0                  0x00UL
#define SBSA_UARTRSR_0                 0x04UL
#define SBSA_UARTIBRD_0                0x24UL
#define SBSA_UARTFR_0                  0x18UL
#define SBSA_UARTLCR_H_0               0x2CUL
#define SBSA_UARTCR_0                  0x30UL
#define SBSA_UARTIFLS_0                0x34UL
#define SBSA_UARTIMSC_0                0x38UL
#define SBSA_UARTRIS_0                 0x3CUL
#define SBSA_UARTMIS_0                 0x40UL
#define SBSA_UARTICR_0                 0x44UL

#define SBSA_UARTIFLS_0_TX_TRIG_1_8    0x0U
#define SBSA_UARTIFLS_0_TX_TRIG_1_4    0x1U
#define SBSA_UARTIFLS_0_TX_TRIG_1_2    0x2U
#define SBSA_UARTIFLS_0_TX_TRIG_3_4    0x3U
#define SBSA_UARTIFLS_0_TX_TRIG_7_8    0x4U
#define SBSA_UARTIFLS_0_RX_TRIG_1_8    0x0U
#define SBSA_UARTIFLS_0_RX_TRIG_1_4    0x1U
#define SBSA_UARTIFLS_0_RX_TRIG_1_2    0x2U
#define SBSA_UARTIFLS_0_RX_TRIG_3_4    0x3U
#define SBSA_UARTIFLS_0_RX_TRIG_7_8    0x4U

#define SBSA_UARTIFLS_0_TXIFLS_SHIFT   0x0U
#define SBSA_UARTIFLS_0_RXIFLS_SHIFT   0x3U
#define SBSA_UARTCR_0_TXE_SHIFT        0x8U
#define SBSA_UARTCR_0_RXE_SHIFT        0x9U
#define SBSA_UARTCR_0_UARTEN_SHIFT     0x0U
#define SBSA_UARTLCR_H_0_PEN_SHIFT     0x1U
#define SBSA_UARTLCR_H_0_EPS_SHIFT     0x2U
#define SBSA_UARTLCR_H_0_FEN_SHIFT     0x4U
#define SBSA_UARTLCR_H_0_STP2_SHIFT    0x3U
#define SBSA_UARTLCR_H_0_WLEN_SHIFT    0x5U

#define SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_5 0x0U
#define SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_6 0x1U
#define SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_7 0x2U
#define SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_8 0x3U

#define SBSA_UARTFR_0_TXFE                  BIT(7U)
#define SBSA_UARTFR_0_RXFE                  BIT(4U)
#define SBSA_UARTFR_0_BUSY                  BIT(3U)
#define SBSA_UARTIMSC_0_TXIM                BIT(5U)
#define SBSA_UARTIMSC_0_RXIM                BIT(4U)
#define SBSA_UARTIMSC_0_RTIM                BIT(6U)
#define SBSA_UARTRIS_RTRIS                  BIT(6U)
#define SBSA_UARTRIS_RXRIS                  BIT(4U)
#define SBSA_UARTRIS_TXRIS                  BIT(5U)
#define SBSA_UART_ISR_NO_INT                0x0U
#define SBSA_UART_IRQ_MASK                  0x7FFUL
#define SBSA_UARTCR_0_TXE_TX_ENABLE         0x1U
#define SBSA_UARTCR_0_RXE_RX_ENABLE         0x1U
#define SBSA_UARTCR_0_UARTEN_UART_ENABLE    0x1U
#define SBSA_UARTLCR_H_0_PEN_NO_PARITY      0x0U
#define SBSA_UARTLCR_H_0_PEN_PARITY         0x1U
#define SBSA_UARTLCR_H_0_EPS_ODD            0x0U
#define SBSA_UARTLCR_H_0_EPS_EVEN           0x1U
#define SBSA_UARTLCR_H_0_STP2_ONE           0x0U
#define SBSA_UARTLCR_H_0_STP2_TWO           0x1U
#define SBSA_UARTLCR_H_0_FEN_ENABLE         0x1U

static inline uint32_t
sbsa_uart_readl(const struct tegra_uart_ctlr * const ctlr,
                 uint32_t offset)
{
    return readl_base_offset(ctlr->id.base_addr, offset);
}

static inline void
sbsa_uart_writel(const struct tegra_uart_ctlr * const ctlr,
                  uint32_t val,
                  uint32_t offset)
{
        writel_base_offset(val, ctlr->id.base_addr, offset);
}

static inline void
sbsa_uart_putc(const struct tegra_uart_ctlr * const ctlr,
                char c)
{
    uint32_t fr;

    do {
        fr = sbsa_uart_readl(ctlr, SBSA_UARTFR_0);
    } while ((fr & SBSA_UARTFR_0_BUSY) != 0UL);

    sbsa_uart_writel(ctlr, c, SBSA_UARTDR_0);
}

static inline char
sbsa_uart_getc(const struct tegra_uart_ctlr * const ctlr)
{
    while ((sbsa_uart_readl(ctlr, SBSA_UARTFR_0) & SBSA_UARTFR_0_RXFE) != 0U);

    return sbsa_uart_readl(ctlr, SBSA_UARTDR_0);
}

SECTION_UART_TEXT error_t
sbsa_uart_flush_tx_hw_fifo(const struct tegra_uart_ctlr * const ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    while ((sbsa_uart_readl(ctlr, SBSA_UARTFR_0) &
                   SBSA_UARTFR_0_TXFE) == 0UL);

out:
    return ret;
}

SECTION_UART_TEXT error_t
sbsa_uart_write_now(const struct tegra_uart_ctlr * const ctlr,
                    const char *buf,
                    uint32_t count)
{
    error_t     ret = E_SUCCESS;
    uint32_t    i;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (buf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    for (i = 0U; i < count; i += 1U) {
        sbsa_uart_putc(ctlr, *buf++);
    }

out:
    return ret;
}

SECTION_UART_TEXT error_t
sbsa_uart_write_char(const struct tegra_uart_ctlr * const ctlr,
                     const char c)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    sbsa_uart_putc(ctlr, c);

out:
    return ret;
}

SECTION_UART_TEXT error_t
sbsa_uart_write_string(const struct tegra_uart_ctlr * const ctlr,
                       const char *buf)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (buf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    while (*buf != '\0') {
        sbsa_uart_putc(ctlr, *buf++);
    }

out:
    return ret;
}

static inline uint64_t
sbsa_uart_get_elapsed_ticks(uint64_t tstart, uint64_t timeout)
{
    uint64_t tcurr = uart_port_get_elapsed_usecs(0ULL);
    uint64_t elapsed = 0UL;

    if ((tcurr - tstart) < timeout) {
        elapsed = (timeout - (tcurr - tstart));
    }

    return elapsed;
}

static void
sbsa_uart_disable_tx_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t irq_mask;

    irq_disable(ctlr->id.irq);
    irq_mask = sbsa_uart_readl(ctlr, SBSA_UARTIMSC_0);
    if ((irq_mask & SBSA_UARTIMSC_0_TXIM) != 0UL) {
        sbsa_uart_writel(ctlr,
                         irq_mask & ~SBSA_UARTIMSC_0_TXIM,
                         SBSA_UARTIMSC_0);
    }
    irq_enable(ctlr->id.irq);
}

static void
sbsa_uart_enable_tx_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t irq_mask;

    irq_disable(ctlr->id.irq);
    irq_mask = sbsa_uart_readl(ctlr, SBSA_UARTIMSC_0);
    if ((irq_mask & SBSA_UARTIMSC_0_TXIM) == 0UL) {
        sbsa_uart_writel(ctlr,
                         irq_mask | SBSA_UARTIMSC_0_TXIM,
                         SBSA_UARTIMSC_0);
    }
    irq_enable(ctlr->id.irq);
}

static void
sbsa_uart_enable_rx_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t irq_mask;

    /*
     * RX interrupt is enabled once in init. No need to place it in
     * the critical section.
     */
    irq_mask = sbsa_uart_readl(ctlr, SBSA_UARTIMSC_0);

    /* Enable the rx timeout and rx interrupt */
    sbsa_uart_writel(ctlr,
                     irq_mask | SBSA_UARTIMSC_0_RXIM | SBSA_UARTIMSC_0_RTIM,
                     SBSA_UARTIMSC_0);
}

SECTION_UART_TEXT error_t
sbsa_uart_write(const struct tegra_uart_ctlr * const ctlr,
                const char *buf,
                uint32_t count,
                uint32_t *written,
                uint64_t timeout)
{
    uint64_t    tstart;
    uint64_t    timeout_left = 0;
    uint32_t    write_cnt = 0UL;
    error_t     ret = E_SUCCESS;
    uint32_t    fr;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (buf == NULL || written == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    if (count == 0UL) {
        goto out;
    }

    tstart = uart_port_get_elapsed_usecs(0ULL);

    if (ctlr->id.irq == UINT32_MAX) {
        for (write_cnt = 0UL; write_cnt < count; write_cnt++) {
            do {
                fr = sbsa_uart_readl(ctlr, SBSA_UARTFR_0);
                if ((fr & SBSA_UARTFR_0_BUSY) == 0UL) {
                    sbsa_uart_writel(ctlr, *buf, SBSA_UARTDR_0);
                    buf++;
                }
                timeout_left = sbsa_uart_get_elapsed_ticks(tstart, timeout);
            } while (timeout_left != 0);
        }
        goto out;
    }

    for (write_cnt = 0UL; write_cnt < count; write_cnt++) {
        ret = uart_port_tx_queue_send(ctlr, buf, 0);
        if (ret == E_UART_QUEUE_FULL) {
            sbsa_uart_enable_tx_interrupt(ctlr);
            timeout_left = sbsa_uart_get_elapsed_ticks(tstart, timeout);
            ret = uart_port_tx_queue_send(ctlr, buf, timeout_left);
            if (ret != E_SUCCESS) {
                break;
            }
        }
        buf++;
    }

    if (write_cnt != 0UL) {
        sbsa_uart_enable_tx_interrupt(ctlr);
    }

out:
    if (written != NULL) {
        *written = write_cnt;
    }
    return ret;
}

SECTION_UART_TEXT error_t
sbsa_uart_flush_tx_fifo(const struct tegra_uart_ctlr * const ctlr)
{
    error_t   ret = E_SUCCESS;
    char      c;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    sbsa_uart_disable_tx_interrupt(ctlr);
    while (1) {
        ret = uart_port_tx_queue_receive(ctlr, &c, 0);
        if (ret != E_SUCCESS) {
            ret = E_UART_FLUSH_TX_FIFO_FAIL;
            break;
        }
        sbsa_uart_putc(ctlr, c);
    }

out:
    return ret;
}

SECTION_UART_TEXT error_t
sbsa_uart_read_char(const struct tegra_uart_ctlr * const ctlr,
                    char *c)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (c == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    *c = sbsa_uart_getc(ctlr);

out:
    return ret;
}

SECTION_UART_TEXT error_t
sbsa_uart_read(const struct tegra_uart_ctlr * const ctlr,
               char *buf,
               uint32_t count,
               uint32_t *read,
               uint64_t timeout)
{
    error_t     ret = E_SUCCESS;
    uint64_t    tstart;
    uint64_t    timeout_left = 0;
    uint32_t    read_cnt = 0UL;
    uint32_t    fr;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (buf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (!ctlr->initialized) {
        ret = E_UART_NO_INIT;
        goto out;
    }

    if (count == 0UL) {
        goto out;
    }

    tstart = uart_port_get_elapsed_usecs(0ULL);

    if (ctlr->id.irq == UINT32_MAX) {
        for (read_cnt = 0; read_cnt < count; read_cnt += 1UL) {
            do {
                fr = sbsa_uart_readl(ctlr, SBSA_UARTFR_0);
                if ((fr & SBSA_UARTFR_0_RXFE) == 0U) {
                    *buf = sbsa_uart_readl(ctlr, SBSA_UARTDR_0);
                    buf++;
                }
                timeout_left = sbsa_uart_get_elapsed_ticks(tstart, timeout);
            } while (timeout_left != 0);
        }
        goto out;
    }

    timeout_left = timeout;
    for (read_cnt = 0UL; read_cnt < count; read_cnt += 1UL) {
        ret = uart_port_rx_queue_receive(ctlr,
                                         buf,
                                         timeout_left);
        if (ret != E_SUCCESS) {
            break;
        }
        buf++;
        timeout_left = sbsa_uart_get_elapsed_ticks(tstart, timeout);
    }

out:
    *read = read_cnt;
    return ret;
}

SECTION_UART_TEXT static void
sbsa_uart_irq_handler(void *data)
{
    const struct tegra_uart_ctlr *ctlr = (const struct tegra_uart_ctlr *)data;
    uint32_t  isr;
    uint32_t  lsr;
    uint32_t  irq_mask;
    char      c_in, c_out;
    error_t   status;
    bool      isr_yield_flag = false;

    isr = sbsa_uart_readl(ctlr, SBSA_UARTRIS_0) & SBSA_UART_IRQ_MASK;
    if (isr == SBSA_UART_ISR_NO_INT) {
        goto out;
    }

    /* RX timeout or RX data available */
    if (((isr & SBSA_UARTRIS_RTRIS) != 0UL) ||
                ((isr & SBSA_UARTRIS_RXRIS) != 0UL)) {
        do {
            c_in = sbsa_uart_readl(ctlr, SBSA_UARTDR_0) & 0xFFU;
            status = uart_port_rx_queue_send_from_isr(ctlr,
                                                      &c_in,
                                                      &isr_yield_flag);
            if (status != E_SUCCESS) {
                break;
            }
            lsr = sbsa_uart_readl(ctlr, SBSA_UARTFR_0);
        } while ((lsr & SBSA_UARTFR_0_RXFE) == 0UL);
    }

    /* TX holding register empty */
    if ((isr & SBSA_UARTRIS_TXRIS) != 0UL) {
        do {
            status = uart_port_tx_queue_receive_from_isr(ctlr,
                                                         &c_out,
                                                         &isr_yield_flag);
            if (status != E_SUCCESS) {
                /* if no data in queue, disable the TX interrupts */
                  irq_mask = sbsa_uart_readl(ctlr, SBSA_UARTIMSC_0);
                  sbsa_uart_writel(ctlr,
                                   irq_mask & ~SBSA_UARTIMSC_0_TXIM,
                                   SBSA_UARTIMSC_0);
                  break;
            }
            sbsa_uart_writel(ctlr, c_out, SBSA_UARTDR_0);
            lsr = sbsa_uart_readl(ctlr, SBSA_UARTFR_0);
        } while ((lsr & SBSA_UARTFR_0_TXFE) != 0UL);
    }

out:
    uart_port_isr_exit(ctlr, isr_yield_flag);
}

static inline void
sbsa_uart_configure_tx_fifo_level(const struct tegra_uart_ctlr * const ctlr)
{
    uint32_t fifo_lvl;

    fifo_lvl = (SBSA_UARTIFLS_0_TX_TRIG_1_8 << SBSA_UARTIFLS_0_TXIFLS_SHIFT);

    sbsa_uart_writel(ctlr, fifo_lvl, SBSA_UARTIFLS_0);
}

static inline void
sbsa_uart_configure_fifo_level(const struct tegra_uart_ctlr * const ctlr)
{
    uint32_t fifo_lvl;

    fifo_lvl = (SBSA_UARTIFLS_0_TX_TRIG_1_8 << SBSA_UARTIFLS_0_TXIFLS_SHIFT) |
               (SBSA_UARTIFLS_0_RX_TRIG_1_8 << SBSA_UARTIFLS_0_RXIFLS_SHIFT);

    sbsa_uart_writel(ctlr, fifo_lvl, SBSA_UARTIFLS_0);
}

static inline void
sbsa_uart_configure_tx_ctrl(const struct tegra_uart_ctlr * const ctlr)
{
    uint32_t cr;

    cr = (SBSA_UARTCR_0_TXE_TX_ENABLE << SBSA_UARTCR_0_TXE_SHIFT);

    sbsa_uart_writel(ctlr, cr, SBSA_UARTCR_0);
}

static inline void
sbsa_uart_configure_ctrl(const struct tegra_uart_ctlr * const ctlr)
{
    uint32_t cr;

    cr = (SBSA_UARTCR_0_TXE_TX_ENABLE << SBSA_UARTCR_0_TXE_SHIFT) |
         (SBSA_UARTCR_0_RXE_RX_ENABLE << SBSA_UARTCR_0_RXE_SHIFT);

    sbsa_uart_writel(ctlr, cr, SBSA_UARTCR_0);
}

static inline void
sbsa_uart_enable(const struct tegra_uart_ctlr * const ctlr)
{
    uint32_t cr;

    cr = sbsa_uart_readl(ctlr, SBSA_UARTCR_0);
    cr |= (SBSA_UARTCR_0_UARTEN_UART_ENABLE << SBSA_UARTCR_0_UARTEN_SHIFT);

    sbsa_uart_writel(ctlr, cr, SBSA_UARTCR_0);
}

static inline error_t
sbsa_uart_get_parity_mode(const struct tegra_uart_ctlr * const ctlr,
                          uint32_t parity_conf,
                          uint32_t *parity)
{
    error_t ret = E_SUCCESS;

    switch (parity_conf) {
    case SBSA_UART_NO_PARITY:
        *parity = (SBSA_UARTLCR_H_0_PEN_NO_PARITY << SBSA_UARTLCR_H_0_PEN_SHIFT);
        break;
    case SBSA_UART_ODD_PARITY:
        *parity = (SBSA_UARTLCR_H_0_EPS_ODD << SBSA_UARTLCR_H_0_EPS_SHIFT) |
                  (SBSA_UARTLCR_H_0_PEN_PARITY << SBSA_UARTLCR_H_0_PEN_SHIFT);
        break;
    case SBSA_UART_EVEN_PARITY:
        *parity = (SBSA_UARTLCR_H_0_EPS_EVEN << SBSA_UARTLCR_H_0_EPS_SHIFT) |
                  (SBSA_UARTLCR_H_0_PEN_PARITY << SBSA_UARTLCR_H_0_PEN_SHIFT);
        break;
    default:
        ret = E_UART_ERR_CONFIG;
        break;
    }

    return ret;
}

static inline error_t
sbsa_uart_get_data_frame_len(const struct tegra_uart_ctlr * const ctlr,
                             uint32_t data_conf,
                             uint32_t *data_bits)
{
    error_t ret = E_SUCCESS;

    switch (data_conf) {
    case SBSA_UART_DATA_BITS_5:
        *data_bits = (SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_5 <<
                      SBSA_UARTLCR_H_0_WLEN_SHIFT);
        break;
    case SBSA_UART_DATA_BITS_6:
        *data_bits = (SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_6 <<
                      SBSA_UARTLCR_H_0_WLEN_SHIFT);
        break;
    case SBSA_UART_DATA_BITS_7:
        *data_bits = (SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_7 <<
                      SBSA_UARTLCR_H_0_WLEN_SHIFT);
        break;
    case SBSA_UART_DATA_BITS_8:
        *data_bits = (SBSA_UARTLCR_H_0_WLEN_WORD_LENGTH_8 <<
                      SBSA_UARTLCR_H_0_WLEN_SHIFT);
        break;
    default:
        ret = E_UART_ERR_CONFIG;
        break;
    }

    return ret;
}

static inline error_t
sbsa_uart_get_stop_bits_len(const struct tegra_uart_ctlr * const ctlr,
                            uint32_t stop_conf,
                            uint32_t *stop_bits)
{
    error_t ret = E_SUCCESS;

    switch (stop_conf) {
    case SBSA_UART_STOP_BITS_1:
        *stop_bits = (SBSA_UARTLCR_H_0_STP2_ONE << SBSA_UARTLCR_H_0_STP2_SHIFT);
        break;
    case SBSA_UART_STOP_BITS_2:
        *stop_bits = (SBSA_UARTLCR_H_0_STP2_TWO << SBSA_UARTLCR_H_0_STP2_SHIFT);
        break;
    default:
        ret = E_UART_ERR_CONFIG;
        break;
    }

    return ret;
}

static inline void
sbsa_uart_configure_line_ctrl(const struct tegra_uart_ctlr * const ctlr,
                              uint32_t parity,
                              uint32_t data_bits,
                              uint32_t stop_bits)
{
    uint32_t lcr;

    /*
     * configure the data line attributes - data bits, stop bits and parity
     * also enable the FIFO.
     */
    lcr = parity | data_bits | stop_bits |
            (SBSA_UARTLCR_H_0_FEN_ENABLE << SBSA_UARTLCR_H_0_FEN_SHIFT);

    sbsa_uart_writel(ctlr, lcr, SBSA_UARTLCR_H_0);
}

static error_t
sbsa_uart_config(struct tegra_uart_ctlr * const ctlr,
                 const struct tegra_uart_conf * const conf)
{
    error_t   ret = E_SUCCESS;
    uint32_t  parity;
    uint32_t  data_bits = 0UL;
    uint32_t  stop_bits = 0UL;

    /* reset the UART controller */
    uart_port_clk_reset_pulse(ctlr, 10);

    /* configure the clock rate based on the baud rate */
    uart_port_clk_set_rate(ctlr, conf->baud * 16);

    /* enable the clock for the UART controller */
    uart_port_clk_enable(ctlr);

    /* port specific init such as pinmux settings to select the UART */
    uart_port_init(ctlr);

    /* generate the parity mode based on input configuration */
    ret = sbsa_uart_get_parity_mode(ctlr, conf->parity, &parity);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* fetch the data frame length in bits */
    ret = sbsa_uart_get_data_frame_len(ctlr, conf->data_bits, &data_bits);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* fetch the stop bits length */
    ret = sbsa_uart_get_stop_bits_len(ctlr, conf->stop_bits, &stop_bits);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* reset the data line attributes by clearing line control register */
    sbsa_uart_writel(ctlr, 0x0, SBSA_UARTLCR_H_0);

    /* clear all the interrupts */
    sbsa_uart_writel(ctlr, 0x0, SBSA_UARTICR_0);

    /* configure the line control register */
    sbsa_uart_configure_line_ctrl(ctlr, parity, data_bits, stop_bits);

out:
    return ret;
}

SECTION_UART_INIT_TEXT error_t
sbsa_uart_init_hw(struct tegra_uart_ctlr * const ctlr,
                   const struct tegra_uart_conf * const conf)
{
    error_t   ret = E_SUCCESS;

    /* sanity check the controller pointer */
    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    /* sanity check for invalid conf parameter */
    if (conf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    /* configure the UART */
    sbsa_uart_config(ctlr, conf);

    /* configure the uart control register */
    sbsa_uart_configure_ctrl(ctlr);

    /* configure the uart fifo level */
    sbsa_uart_configure_fifo_level(ctlr);

    /* enable the uart */
    sbsa_uart_enable(ctlr);

    /* enable the rx interrupt */
    sbsa_uart_enable_rx_interrupt(ctlr);

    if (ctlr->id.irq != UINT32_MAX) {
        /* register irq handler and enable irq */
        irq_set_handler(ctlr->id.irq, sbsa_uart_irq_handler, ctlr);
        irq_enable(ctlr->id.irq);
    }

    /* indicate the controller is initialized */
    ctlr->initialized = true;

out:
    if (ret != E_SUCCESS) {
        uart_port_clk_disable(ctlr);
    }

    return ret;
}

SECTION_UART_INIT_TEXT error_t
sbsa_uart_3rdparty_init(struct tegra_uart_ctlr *ctlr,
                        const struct tegra_uart_conf * const conf)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (conf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    /*
     * Do the minimum required for configuring the UART if not already done.
     */
    if (!uart_port_is_configured(ctlr)) {
        ret = sbsa_uart_config(ctlr, conf);
        if (ret != E_SUCCESS) {
            goto out;
        }

        /* configure the uart control register */
        sbsa_uart_configure_tx_ctrl(ctlr);

        /* configure the uart fifo level */
        sbsa_uart_configure_tx_fifo_level(ctlr);

        /* enable the uart */
        sbsa_uart_enable(ctlr);
    }

    /* indicate the controller is initialized */
    ctlr->initialized = true;

out:
    return ret;
}

SECTION_UART_INIT_TEXT error_t
sbsa_uart_init(struct tegra_uart_ctlr *ctlr,
               const struct tegra_uart_conf * const conf)
{
    error_t     ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (conf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    /* check if UART is already configured by a 3rd party and skip the init */
    if (uart_port_is_configured(ctlr)) {
        ctlr->initialized = true;
        goto out;
    }

    /* check if irq is supported */
    if (ctlr->id.irq == UINT32_MAX) {
        goto skip_queues;
    }

    /* create the TX sw buffer for interrupt driven TX */
    ret = uart_port_tx_queue_create(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* create the RX sw buffer for interrupt driven RX */
    ret = uart_port_rx_queue_create(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

skip_queues:
    ret = sbsa_uart_init_hw(ctlr, conf);

out:
    return ret;
}

error_t sbsa_uart_deinit(struct tegra_uart_ctlr *ctlr)
{
    error_t     ret = E_SUCCESS;

    ctlr->initialized = false;

    ret = uart_port_tx_queue_delete(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    ret = uart_port_rx_queue_delete(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    ret = uart_port_clk_disable(ctlr);

out:
    return ret;
}
