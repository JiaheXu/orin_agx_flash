/*
 * Copyright (c) 2021 NVIDIA CORPORATION.  All rights reserved.
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

/* Late FSP headers */
#include <error/common-errors.h>             // for E_SUCCESS, error_t, FSP_...
#include <misc/attributes.h>                 // for FSP__MISC__ATTRIBUTES_H
#include <misc/bitops.h>                     // for BIT
#include <reg-access/reg-access.h>           // for readl, writel, FSP__REG_...
#include <irq/irqs.h>                        // for irq_enable, irq_disable...
#include <delay/delay.h>                     // for get_time_ticks
#include <tke/tke-tegra.h>
#include <cpu/barriers.h>

/* Module-specific FSP headers */
#include <comb-uart/sections-comb-uart.h>    // Immune from CT_ASSERT protection
#include <comb-uart/comb-uart-errors.h>      // for E_COMB_UART_...
#include <comb-uart/comb-uart.h>             // for comb_uart_init, comb_uart...
#include <comb-uart/comb-uart-priv.h>        // for comb_uart_id...
#include <comb-uart/comb-uart-port.h>        // for comb_uart_port_...

/* Shared mailbox protocol bits */
#define COMB_UART_PKT_SIZE_SHIFT    (24U)
#define COMB_UART_PKT_FLUSH_BIT     BIT(26U)
#define COMB_UART_PKT_TAG_BIT       BIT(31U)
#define COMB_UART_MAX_PKTS          (3U)

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
    CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__IRQ__IRQS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__DELAY__DELAY_H, "Header file missing or invalid.")

/* RX/TX timeout */
#define COMB_UART_TIMEOUT_USEC          (10000UL)
#define COMB_UART_TIMEOUT_WAIT_USEC     (10UL)

/* init timeout */
#define COMB_UART_INIT_TIMEOUT_USEC     (500000UL)

/* status flags for tx and rx enable */
static bool tx_enabled = false;
static bool rx_enabled = false;

/* tx buffer and number of bytes in the tx buf */
static char tx_buf[64];
static uint32_t tx_bytes = 0;

/**
 * Helper function to wait for mailbox tag bit to be cleared.
 */
static inline bool
comb_uart_is_tx_empty(const struct comb_uart_id *id)
{
    uint32_t val;

    val = readl(id->tx_sm_reg);

    return (val & COMB_UART_PKT_TAG_BIT) == 0UL;
}

/**
 * Helper function to wait for mailbox tag bit to be cleared for specified
 * timeout.
 */
static inline error_t
comb_uart_wait_tx_timeout(const struct comb_uart_id *id,
                          const uint64_t timeout_usec)
{
    error_t  ret = E_SUCCESS;
    uint64_t tcurr = 0ULL;
    uint64_t start;

    if (!comb_uart_is_tx_empty(id)) {
        ret = E_COMB_UART_TIMEDOUT;
        start = comb_uart_port_get_elapsed_usecs(0ULL);

        do {
            if (comb_uart_is_tx_empty(id)) {
                ret = E_SUCCESS;
                break;
            }

            tcurr = comb_uart_port_get_elapsed_usecs(0ULL);

        } while ((tcurr - start) < timeout_usec);
    }

    return ret;
}

/**
 * Helper function to wait for mailbox tag bit to be cleared or return
 * timeout.
 */
static inline error_t
comb_uart_wait_tx(const struct comb_uart_id *id)
{
    return comb_uart_wait_tx_timeout(id, COMB_UART_TIMEOUT_USEC);
}

/**
 * Helper function to create a tx packet to be sent to SPE adhering
 * to the combined uart protocol.
 */
static inline uint32_t
comb_uart_get_tx_val(const struct comb_uart_id *id,
                     const char *buf,
                     const uint32_t buf_size,
                     const bool flush)
{
    uint32_t to_write;
    uint32_t tx;
    uint32_t ch;

    to_write = buf_size;

    /* Initialize TX value */
    tx = COMB_UART_PKT_TAG_BIT |
         (flush ? COMB_UART_PKT_FLUSH_BIT : 0u) |
         (to_write << COMB_UART_PKT_SIZE_SHIFT);

    /* Add the characters into the TX value */
    for (uint32_t i = 0U; i < to_write; i += 1UL) {
        ch = (uint8_t)buf[i];
        tx |= ch << (i * 8U);
    }

    return tx;
}

/**
 * Helper function to write 1-3 characters to SPE, wait for mailbox
 * tag bit to be  cleared and return how many bytes were written or
 * E_COMB_UART_TIMEDOUT if transfer timed out.
 */
static error_t
comb_uart_write(const struct comb_uart_id *id,
                const char *buf,
                const uint32_t nbytes,
                const bool flush,
                uint32_t *written)
{
    error_t  ret;
    uint32_t to_write;
    uint32_t tx;

    to_write = (nbytes > COMB_UART_MAX_PKTS) ? COMB_UART_MAX_PKTS : nbytes;

    /* Initialize TX value */
    tx = comb_uart_get_tx_val(id, buf, to_write, flush);

    /* Write to SPE mailbox */
    writel(tx, id->tx_sm_reg);

    /* Wait for SPE mailbox to be cleared */
    ret = comb_uart_wait_tx(id);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* update the chars written */
    *written = to_write;

out:
    return ret;
}

SECTION_COMB_UART_TEXT static error_t
comb_uart_flush(const struct comb_uart_id *id,
                const bool wait)
{
    error_t    ret = E_SUCCESS;
    const char *src;
    uint32_t   nbytes;
    uint32_t   written = 0UL;

    if (!tx_enabled) {
        ret = E_COMB_UART_NO_TX;
        goto out;
    }

    nbytes = tx_bytes;
    if (nbytes > 0u) {
        src = tx_buf;
        do {
            ret = comb_uart_write(id, src, nbytes, false, &written);
            if (ret != E_SUCCESS) {
                ret = E_COMB_UART_TIMEDOUT;
                break;
            }
            nbytes -= written;
            src = &src[written];
        } while (nbytes > 0u);

        /* Send the flush bit if requested */
        tx_bytes = 0;
        if (wait) {
            UNUSED(comb_uart_write(id, src, 0u, wait, &written));
        }
    }

out:
    return ret;
}

SECTION_COMB_UART_TEXT error_t
comb_uart_tx_flush(const struct comb_uart_id *id)
{
    return comb_uart_flush(id, true);
}

SECTION_COMB_UART_TEXT error_t
comb_uart_putc(const struct comb_uart_id *id,
               const char ch)
{
    error_t ret = E_SUCCESS;

    enter_critical();

    /* Push character to TX buffer */
    tx_buf[tx_bytes] = ch;
    tx_bytes += 1UL;

    /* Flush on newline or if buffer is full */
    if ((ch == '\n') || (tx_bytes == sizeof(tx_buf))) {
        ret = comb_uart_flush(id, false);
    }

    exit_critical();

    return ret;
}

SECTION_COMB_UART_TEXT void
comb_uart_puts(const struct comb_uart_id *id,
               const char *s)
{
    while (*s != '\0') {
        UNUSED(comb_uart_putc(id, *s++));
    }
}

SECTION_COMB_UART_TEXT void
comb_uart_send(const struct comb_uart_id *id,
               const char *d,
               const uint32_t len)
{
    uint32_t i;

    for (i = 0UL; i < len; i += 1UL) {
        UNUSED(comb_uart_putc(id, *d++));
    }
}

static error_t
comb_uart_tx_init(const struct comb_uart_id *id)
{
    error_t    ret = E_SUCCESS;
    uint32_t   tx;
    const char ch = '\n';

    /* if no tx hsp sm register, return with an error */
    if (id->tx_sm_reg == 0UL) {
        ret = E_COMB_UART_NULL_PTR;
        goto out;
    }

    /**
     * check to ensure combined uart server is alive by sending
     * a '\n' char.
     */
    tx = comb_uart_get_tx_val(id, &ch, 1, false);
    writel(tx, id->tx_sm_reg);

    /**
     * wait for server to clear the mailbox. If
     * the wait times out, server did not respond and tx failed.
     */
    ret = comb_uart_wait_tx_timeout(id, COMB_UART_INIT_TIMEOUT_USEC);
    if (ret != E_SUCCESS) {
        tx_enabled = false;
        goto out;
    }

    tx_enabled = true;

out:
    return ret;
}

static error_t
comb_uart_rx_init(const struct comb_uart_id *id)
{
    error_t ret = E_SUCCESS;

    /* okay to not use rx */
    if (id->supports_rx) {
        if (id->rx_sm_reg == 0UL) {
            ret = E_COMB_UART_NULL_PTR;
            goto out;
        } else {
            rx_enabled = true;
        }
    }

out:
    return ret;
}

SECTION_COMB_UART_INIT_TEXT error_t
comb_uart_init(const struct comb_uart_id *id)
{
    error_t ret = E_SUCCESS;

    ret = comb_uart_tx_init(id);
    if (ret != E_SUCCESS) {
        goto out;
    }

    ret = comb_uart_rx_init(id);
    if (ret != E_SUCCESS) {
        goto out;
    }

out:
    return ret;
}
