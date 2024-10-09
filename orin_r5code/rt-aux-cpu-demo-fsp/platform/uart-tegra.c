/*
 * Copyright (c) 2019-2022 NVIDIA CORPORATION.  All rights reserved.
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

/* local os headers */
#include <osa/osa-task.h>
#include <osa/osa-errors.h>

/* Early FSP headers */
#include <misc/ct-assert.h>                  // for CT_ASSERT
#include <soc-common/hw-const.h>             /* Must appear before any hwinc files */

/* Hardware headers */
#include <aruart.h>                          // for UART_LCR_0_WD_SIZE_SHIFT
#include <address_map_new.h>                 // NV_ADDRESS_MAP_...
#include <arpadctl_AO.h>                     // PADCTL_AO_UART3_RX_0

/* Late FSP headers */
#include <chipid/chip-id.h>                  // tegra_platform_is_fpga...
#include <clk/clk-tegra.h>                   // for tegra_clk_disable, tegra...
#include <error/common-errors.h>             // for E_SUCCESS, error_t, FSP_...
#include <misc/attributes.h>                 // for FSP__MISC__ATTRIBUTES_H
#include <reg-access/reg-access.h>           // for readl, writel, FSP__REG_...
#include <tke/tke-tegra.h>                   // for tegra_tke_ APIs
#include <irq/irqs.h>                        // for irq_enable, irq_disable...
#include <delay/delay.h>                     // for get_time_ticks

/* Module-specific FSP headers */
#include <uart/sections-uart.h>              // Immune from CT_ASSERT protection
#include <uart/uart.h>                       // for tegra_uart_conf,
#include <uart/tegra-uart-priv.h>            // for tegra_uart_ctlr, tegra_u...
#include <uart/uart-errors.h>                // for E_UART_ERR_CONFIG, ...

/* local headers */
#include <uart-tegra.h>                      // for tegra_uart_init, tegra_uart_..
#include <uart-macros.h>                     // for SPE_UART_...

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
    CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")

#define TEGRA_UART_LCR_DLAB             (UART_LCR_0_DLAB_ENABLE << \
                                                UART_LCR_0_DLAB_SHIFT)
#define TEGRA_UART_IER_THRE             (UART_IER_DLAB_0_0_IE_THR_ENABLE << \
                                                UART_IER_DLAB_0_0_IE_THR_SHIFT)
#define TEGRA_UART_LSR_RDY              (UART_LSR_0_RDR_DATA_IN_FIFO << \
                                                UART_LSR_0_RDR_SHIFT)
#define TEGRA_UART_LSR_TX_FIFO_FULL     (UART_LSR_0_TX_FIFO_FULL_FULL << \
                                                UART_LSR_0_TX_FIFO_FULL_SHIFT)
#define TEGRA_UART_LSR_THRE_EMPTY       (UART_LSR_0_THRE_EMPTY << \
                                                UART_LSR_0_THRE_SHIFT)
#define TEGRA_UART_LSR_TX_SHIFT_EMPTY   (UART_LSR_0_TMTY_EMPTY << \
                                                UART_LSR_0_TMTY_SHIFT)
#define TEGRA_UART_IIR_NO_INT           (UART_IIR_FCR_0_IS_STA_NO_INTR_PEND \
                                                << UART_IIR_FCR_0_IS_STA_SHIFT)

#define TEGRA_UART_IRQ_MASK             0x0FU
#define TEGRA_UART_RX_TIMEOUT           0x0CU
#define TEGRA_UART_RX_DATA_AVAILABLE    0x04U
#define TEGRA_UART_THRE_EMPTY           0x02U
#define TEGRA_UART_LINE_STS_ERR_INT     0x06U
#define TEGRA_UART_MODEM_STS_INT        0x00U

#define TEGRA_UART_BUF_SIZE          256

static struct tegra_uart_conf uart_conf;

static bool en_rx_irq = false;

static inline uint32_t
tegra_uart_readl(const struct tegra_uart_ctlr * const ctlr,
                 uint32_t offset)
{
    return readl_base_offset(ctlr->id.base_addr, offset);
}

static inline void
tegra_uart_writel(const struct tegra_uart_ctlr * const ctlr,
                  uint32_t val,
                  uint32_t offset)
{
        writel_base_offset(val, ctlr->id.base_addr, offset);
}

static inline void
tegra_uart_putc(const struct tegra_uart_ctlr * const ctlr,
                char c)
{
    uint32_t lsr;

    do {
        lsr = tegra_uart_readl(ctlr, UART_LSR_0);
    } while ((lsr & TEGRA_UART_LSR_THRE_EMPTY) == 0UL);

    tegra_uart_writel(ctlr, c, UART_THR_DLAB_0_0);
}

static inline char
tegra_uart_getc(const struct tegra_uart_ctlr * const ctlr)
{
    while ((tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_RDY) == 0U);

    return tegra_uart_readl(ctlr, UART_THR_DLAB_0_0);
}

SECTION_UART_TEXT error_t
tegra_uart_flush_tx_hw_fifo(const struct tegra_uart_ctlr * const ctlr)
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

    while ((tegra_uart_readl(ctlr, UART_LSR_0) &
                   TEGRA_UART_LSR_TX_SHIFT_EMPTY) == 0UL);

out:
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_write_now(const struct tegra_uart_ctlr * const ctlr,
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
        tegra_uart_putc(ctlr, *buf++);
    }

out:
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_write_char(const struct tegra_uart_ctlr * const ctlr,
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

    tegra_uart_putc(ctlr, c);

out:
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_write_string(const struct tegra_uart_ctlr * const ctlr,
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
        tegra_uart_putc(ctlr, *buf++);
    }

out:
    return ret;
}

static inline rtosTick
tegra_uart_get_elapsed_ticks(uint64_t tstart, uint64_t timeout)
{
    uint64_t tcurr = get_time_ticks();

    if (timeout >= rtosMAX_DELAY) {
        return rtosMAX_DELAY;
    } else if ((tcurr - tstart) >= timeout) {
        return 0;
    } else {
        return tegra_tke_convert_usecs_to_ticks(timeout - (tcurr - tstart));
    }
}

static void
tegra_uart_disable_thre_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t ier;

    irq_disable(ctlr->id.irq);
    ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
    if ((ier & TEGRA_UART_IER_THRE) != 0UL) {
        tegra_uart_writel(ctlr,
                         ier & ~TEGRA_UART_IER_THRE,
                         UART_IER_DLAB_0_0);
    }
    irq_enable(ctlr->id.irq);
}

static void
tegra_uart_enable_thre_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t ier;

    irq_disable(ctlr->id.irq);
    ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
    if ((ier & TEGRA_UART_IER_THRE) == 0UL) {
        tegra_uart_writel(ctlr,
                         ier | TEGRA_UART_IER_THRE,
                         UART_IER_DLAB_0_0);
    }
    irq_enable(ctlr->id.irq);
}

static void
tegra_uart_disable_rx_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t ier;

    irq_disable(ctlr->id.irq);
    ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
    /* clear RX Interrupts */
    ier &= ~((UART_IER_DLAB_0_0_IE_RHR_ENABLE << UART_IER_DLAB_0_0_IE_RHR_SHIFT) |
             (UART_IER_DLAB_0_0_IE_RXS_ENABLE << UART_IER_DLAB_0_0_IE_RXS_SHIFT) |
             (UART_IER_DLAB_0_0_IE_RX_TIMEOUT_ENABLE <<
                                   UART_IER_DLAB_0_0_IE_RX_TIMEOUT_SHIFT));
    tegra_uart_writel(ctlr, ier, UART_IER_DLAB_0_0);
    en_rx_irq = false;
    irq_enable(ctlr->id.irq);
}

static void
tegra_uart_enable_rx_interrupt(const struct tegra_uart_ctlr *ctlr)
{
    uint32_t ier;

    irq_disable(ctlr->id.irq);
    ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
    /* Enable RX Interrupts */
    ier |= (UART_IER_DLAB_0_0_IE_RHR_ENABLE << UART_IER_DLAB_0_0_IE_RHR_SHIFT) |
           (UART_IER_DLAB_0_0_IE_RXS_ENABLE << UART_IER_DLAB_0_0_IE_RXS_SHIFT) |
           (UART_IER_DLAB_0_0_IE_RX_TIMEOUT_ENABLE <<
                                   UART_IER_DLAB_0_0_IE_RX_TIMEOUT_SHIFT);
    tegra_uart_writel(ctlr, ier, UART_IER_DLAB_0_0);
    en_rx_irq = true;
    irq_enable(ctlr->id.irq);
}

SECTION_UART_TEXT error_t
tegra_uart_write(const struct tegra_uart_ctlr * const ctlr,
                 const char *buf,
                 uint32_t count,
                 uint32_t *written,
                 uint64_t timeout)
{
    uint64_t    tstart;
    rtosTick    timeout_left = 0;
    uint32_t    write_cnt = 0UL;
    error_t     status;
    error_t     ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (buf == NULL || written == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (count == 0UL) {
        goto out;
    }

    tstart = get_time_ticks();

    for (write_cnt = 0UL; write_cnt < count; write_cnt++) {
        status = rtosQueueSend(ctlr->tx_queue, buf, 0);
        if (status == E_RTOS_QUEUE_FULL) {
            tegra_uart_enable_thre_interrupt(ctlr);
            timeout_left = tegra_uart_get_elapsed_ticks(tstart, timeout);
            status = rtosQueueSend(ctlr->tx_queue, buf, timeout_left);
            if (status != rtosPASS) {
                break;
            }
        }
        buf++;
    }

    if (write_cnt != 0UL) {
        tegra_uart_enable_thre_interrupt(ctlr);
    }

out:
    *written = write_cnt;
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_flush_tx_fifo(const struct tegra_uart_ctlr * const ctlr)
{
    error_t ret = E_SUCCESS;
    error_t status;
    char    c;


    tegra_uart_disable_thre_interrupt(ctlr);
    while (1) {
        status = rtosQueueReceive(ctlr->tx_queue, &c, 0);
        if (status != rtosPASS) {
            ret = E_UART_FLUSH_TX_FIFO_FAIL;
            break;
        }
        tegra_uart_putc(ctlr, c);
    }

    return ret;
}


SECTION_UART_TEXT error_t
tegra_uart_read_char(const struct tegra_uart_ctlr * const ctlr,
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

    *c = tegra_uart_getc(ctlr);

out:
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_read(const struct tegra_uart_ctlr * const ctlr,
                char *buf,
                uint32_t count,
                uint32_t *read,
                uint64_t timeout)
{
    error_t     ret = E_SUCCESS;
    uint64_t    tstart;
    rtosTick    timeout_left = 0;
    uint32_t    read_cnt;
    error_t     status;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (buf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }

    if (!en_rx_irq) {
        tegra_uart_enable_rx_interrupt(ctlr);
    }

    timeout_left = tegra_tke_convert_usecs_to_ticks(timeout);
    tstart = get_time_ticks();

    for (read_cnt = 0UL; read_cnt < count; read_cnt += 1UL) {
        status = rtosQueueReceive(ctlr->rx_queue,
                                    buf,
                                    timeout_left);
        if (status != rtosPASS) {
            ret = status;
            break;
        }
        buf++;
        timeout_left = tegra_uart_get_elapsed_ticks(tstart, timeout);
    }
    *read = read_cnt;

out:
    return ret;
}

SECTION_UART_TEXT static void
tegra_uart_irq_handler(void *data)
{
    const struct tegra_uart_ctlr *ctlr = (const struct tegra_uart_ctlr *)data;
    uint32_t  iir, ier;
    uint32_t  lsr;
    char      c_in, c_out;
    rtosBool higher_prio_task_woken = rtosFALSE;
    error_t   status;

    while (1) {
        iir = tegra_uart_readl(ctlr, UART_IIR_FCR_0) & TEGRA_UART_IRQ_MASK;
        if (iir & TEGRA_UART_IIR_NO_INT) {
            break;
        }

        switch (iir) {
            case TEGRA_UART_LINE_STS_ERR_INT:
                tegra_uart_readl(ctlr, UART_LSR_0);
                break;
            case TEGRA_UART_RX_TIMEOUT:
            case TEGRA_UART_RX_DATA_AVAILABLE:
                do {
                    c_in = tegra_uart_readl(ctlr, UART_THR_DLAB_0_0) & 0xFFU;
                    status = rtosQueueSendFromISR(ctlr->rx_queue,
                                                      &c_in,
                                                      &higher_prio_task_woken);
                    if (status != rtosPASS) {
                        break;
                    }
                    lsr = tegra_uart_readl(ctlr, UART_LSR_0);
                } while ((lsr & TEGRA_UART_LSR_RDY) != 0UL);
                break;
            case TEGRA_UART_THRE_EMPTY:
                do {
                    status = rtosQueueReceiveFromISR(ctlr->tx_queue,
                                                         &c_out,
                                                         &higher_prio_task_woken);
                    if (status != rtosPASS) {
                        ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
                        tegra_uart_writel(ctlr,
                                          ier & ~TEGRA_UART_IER_THRE,
                                          UART_IER_DLAB_0_0);
                        break;
                    }
                    tegra_uart_writel(ctlr, c_out, UART_THR_DLAB_0_0);
                    lsr = tegra_uart_readl(ctlr, UART_LSR_0);
                } while ((lsr & TEGRA_UART_LSR_THRE_EMPTY) != 0UL);
                break;
            case TEGRA_UART_MODEM_STS_INT:
                tegra_uart_readl(ctlr, UART_MSR_0);
                break;
        }
    }

   rtosTaskYieldFromISR(higher_prio_task_woken);
}

static inline void
tegra_uart_configure_fifo_ctlr(const struct tegra_uart_ctlr * const ctlr)
{
    uint32_t fcr;

    fcr = (UART_IIR_FCR_0_FCR_EN_FIFO_ENABLE <<
                UART_IIR_FCR_0_FCR_EN_FIFO_SHIFT);
    fcr |= (UART_IIR_FCR_0_RX_CLR_CLEAR <<
                UART_IIR_FCR_0_RX_CLR_SHIFT);
    fcr |= (UART_IIR_FCR_0_TX_CLR_CLEAR <<
                UART_IIR_FCR_0_TX_CLR_SHIFT);
    fcr |= (UART_IIR_FCR_0_RX_TRIG_FIFO_COUNT_GREATER_8 <<
                UART_IIR_FCR_0_RX_TRIG_SHIFT);
    fcr |= (UART_IIR_FCR_0_TX_TRIG_FIFO_COUNT_GREATER_4 <<
                UART_IIR_FCR_0_TX_TRIG_SHIFT);

    tegra_uart_writel(ctlr, fcr, UART_IIR_FCR_0);
}

static inline error_t
tegra_uart_get_parity_mode(const struct tegra_uart_ctlr * const ctlr,
                           uint32_t parity_conf,
                           uint32_t *parity)
{
    error_t ret = E_SUCCESS;

    switch (parity_conf) {
    case TEGRA_UART_NO_PARITY:
        *parity = (UART_LCR_0_PAR_NO_PARITY << UART_LCR_0_PAR_SHIFT);
        break;
    case TEGRA_UART_ODD_PARITY:
        *parity = (UART_LCR_0_EVEN_DISABLE << UART_LCR_0_EVEN_SHIFT) |
                     (UART_LCR_0_PAR_PARITY << UART_LCR_0_PAR_SHIFT);
        break;
    case TEGRA_UART_EVEN_PARITY:
        *parity = (UART_LCR_0_EVEN_ENABLE << UART_LCR_0_EVEN_SHIFT) |
                     (UART_LCR_0_PAR_PARITY << UART_LCR_0_PAR_SHIFT);
        break;
    default:
        ret = E_UART_ERR_CONFIG;
        goto out;
    }

out:
    return ret;
}

static inline error_t
tegra_uart_get_data_frame_len(const struct tegra_uart_ctlr * const ctlr,
                              uint32_t data_conf,
                              uint32_t *data_bits)
{
    error_t ret = E_SUCCESS;

    switch (data_conf) {
    case TEGRA_UART_DATA_BITS_5:
        *data_bits = (UART_LCR_0_WD_SIZE_WORD_LENGTH_5 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    case TEGRA_UART_DATA_BITS_6:
        *data_bits = (UART_LCR_0_WD_SIZE_WORD_LENGTH_6 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    case TEGRA_UART_DATA_BITS_7:
        *data_bits = (UART_LCR_0_WD_SIZE_WORD_LENGTH_7 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    case TEGRA_UART_DATA_BITS_8:
        *data_bits = (UART_LCR_0_WD_SIZE_WORD_LENGTH_8 <<
                       UART_LCR_0_WD_SIZE_SHIFT);
        break;
    default:
        ret = E_UART_ERR_CONFIG;
        goto out;
    }

out:
    return ret;
}

static inline error_t
tegra_uart_get_stop_bits_len(const struct tegra_uart_ctlr * const ctlr,
                             uint32_t stop_conf,
                             uint32_t *stop_bits)
{
    error_t ret = E_SUCCESS;

    switch (stop_conf) {
    case TEGRA_UART_STOP_BITS_1:
        *stop_bits = 0x0UL;
        break;
    case TEGRA_UART_STOP_BITS_2:
        *stop_bits = (UART_LCR_0_STOP_ENABLE << UART_LCR_0_STOP_SHIFT);
        break;
    default:
        ret = E_UART_ERR_CONFIG;
        goto out;
    }

out:
    return ret;
}

static void
tegra_uart_baud_config(struct tegra_uart_ctlr * const ctlr,
                       const uint32_t baud_divisor)
{
    /*
     * program the baud rate divisor using DLL/DLM registers
     * THR, RBR and DLL share the same address space.
     * IER, DLM share the same address space.
     * DLx registers are slected based on LCR.DLAB bit
     */
    tegra_uart_writel(ctlr, baud_divisor & 0xFFU, UART_THR_DLAB_0_0);
    tegra_uart_writel(ctlr, ((baud_divisor >> 8) & 0xFFU), UART_IER_DLAB_0_0);
}

static void
tegra_uart_pinmux_config(struct tegra_uart_ctlr * const ctlr)
{
    UNUSED(ctlr);

	writel(0x440U, NV_ADDRESS_MAP_PADCTL_A14_BASE + PADCTL_AO_UART3_RX_0);
	writel(0x440U, NV_ADDRESS_MAP_PADCTL_A14_BASE + PADCTL_AO_UART3_TX_0);
}

static void
tegra_uart_clk_config(struct tegra_uart_ctlr * const ctlr)
{
    UNUSED(ctlr);

    /*
     * Source UARTC from OSC, which is 38.4 MHz except on FPGA,
     * where it is 19.2 MHz.
     */
    if (tegra_platform_is_fpga()) {
        /**
         * Target baud rate = 115200
         * Baud clock needed = 1.8432MHz
         * OSC = 19.2MHz
         * Value of divisor = 10.5 (smallest error)
         * Divisor setting = (divisor - 1) * 2 = 19
         */
        SPE_UART_ENABLE(UARTC, OSC_UNDIV, ENABLE, 19U);
    } else {
        /**
         * Target baud rate = 115200
         * Baud clock needed = 1.8432MHz
         * OSC = 38.4MHz
         * Value of divisor = 21 (smallest error)
         * Divisor setting = (divisor - 1) * 2 = 40
         */
        SPE_UART_ENABLE(UARTC, OSC_UNDIV, ENABLE, 40U);
    }
}

SECTION_UART_INIT_TEXT static error_t
tegra_uart_config(struct tegra_uart_ctlr * const ctlr,
                  const struct tegra_uart_conf * const conf)
{
    error_t   ret = E_SUCCESS;
    uint32_t  divisor = 0x1UL;
    uint32_t  parity;
    uint32_t  data_bits = 0UL;
    uint32_t  stop_bits = 0UL;
    uint32_t  comm_param;

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

    if (!tegra_platform_is_silicon()) {
        /* program the pinmux settings */
        tegra_uart_pinmux_config(ctlr);
    }

    /* configure the uart clock */
    tegra_uart_clk_config(ctlr);

    /* configure the FIFO control register */
    tegra_uart_configure_fifo_ctlr(ctlr);

    /* generate the parity mode based on input configuration */
    ret = tegra_uart_get_parity_mode(ctlr, conf->parity, &parity);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* fetch the data frame length in bits */
    ret = tegra_uart_get_data_frame_len(ctlr, conf->data_bits, &data_bits);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* fetch the stop bits length */
    ret = tegra_uart_get_stop_bits_len(ctlr, conf->stop_bits, &stop_bits);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* reset the data line attributes by clearing line control register */
    tegra_uart_writel(ctlr, 0x0, UART_LCR_0);

    /* clear all the interrupts */
    tegra_uart_writel(ctlr, 0x0, UART_IER_DLAB_0_0);

    /*
     * configure the data line attributes - data bits, stop bits and parity
     * also set the divisor latch access bit to program the divisor in the next
     * step.
     */
    comm_param = parity | data_bits | stop_bits | TEGRA_UART_LCR_DLAB,
    tegra_uart_writel(ctlr, comm_param, UART_LCR_0);

    /* configure the baud rate divisor */
    tegra_uart_baud_config(ctlr, divisor);

    /* clear the LCR.DLAB to switch to the THR and IER address space */
    tegra_uart_writel(ctlr, comm_param & ~TEGRA_UART_LCR_DLAB, UART_LCR_0);

    /* dummy read to ensure write is posted */
    tegra_uart_readl(ctlr, UART_SPR_0);

out:
    return ret;
}

SECTION_UART_INIT_TEXT static error_t
tegra_uart_init_hw(struct tegra_uart_ctlr * const ctlr,
                   const struct tegra_uart_conf * const conf)
{
    error_t   ret = E_SUCCESS;

    ret = tegra_uart_config(ctlr, conf);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* register irq handler and enable irq */
    irq_set_handler(ctlr->id.irq, tegra_uart_irq_handler, ctlr);
    irq_enable(ctlr->id.irq);

    /* indicate the controller is initialized */
    ctlr->initialized = true;

out:
    return ret;
}

static bool is_uart_c_configured(void)
{
    bool is_clk_enabled = SPE_UART_IS_CLK_ENABLED(UARTC);
    bool is_reset_deasserted = SPE_UART_IS_RESET_DEASSERTED(UARTC);

    return (is_clk_enabled && is_reset_deasserted);
}

SECTION_UART_INIT_TEXT error_t
tegra_uart_3rdparty_init(struct tegra_uart_ctlr *ctlr,
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
     * If SPE ends up booting first without TCU, do the minimum
     * required for configuring the UART before BPMP takes over.
     */
    if (!is_uart_c_configured()) {
        ret = tegra_uart_config(ctlr, conf);
        if (ret != E_SUCCESS) {
            goto out;
        }
    }

    /* indicate the controller is initialized */
    ctlr->initialized = true;

out:
    return ret;
}

SECTION_UART_INIT_TEXT error_t
tegra_uart_init(struct tegra_uart_ctlr *ctlr,
                const struct tegra_uart_conf * const conf)
{
    error_t     ret = E_SUCCESS;
    error_t     status;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (conf == NULL) {
        ret = E_UART_INVALID_PARAM;
        goto out;
    }
    uart_conf.parity    =  conf->parity;
    uart_conf.stop_bits =  conf->stop_bits;
    uart_conf.data_bits =  conf->data_bits;
    uart_conf.baud      =  conf->baud;

    status = rtosQueueCreate(NULL,
                             0,
                             TEGRA_UART_BUF_SIZE,
                             sizeof(char),
                             &ctlr->tx_queue);
    if (status != rtosPASS) {
        ret = E_UART_INIT_FAIL;
        goto out;
    }

    status = rtosQueueCreate(NULL,
                             0,
                             TEGRA_UART_BUF_SIZE,
                             sizeof(char),
                             &ctlr->rx_queue);
    if (status != rtosPASS) {
        ret = E_UART_INIT_FAIL;
        goto out;
    }

    ret = tegra_uart_init_hw(ctlr, conf);

out:
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_suspend(struct tegra_uart_ctlr *ctlr)
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

    tegra_uart_disable_rx_interrupt(ctlr);

out:
    return ret;
}

SECTION_UART_TEXT error_t
tegra_uart_resume(struct tegra_uart_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    ret = tegra_uart_init_hw(ctlr, &uart_conf);
    if (ret != E_SUCCESS) {
        goto out;
    }

    if (!en_rx_irq) {
        tegra_uart_enable_rx_interrupt(ctlr);
    }

out:
    return ret;
}
