/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* local os headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>
#include <processor/debug-hw.h>
#include <uart/uart.h>

/* local headers */
#include <spe-uart.h>
#include <uart-tegra.h>

#define TEGRA194_UART_PORT_A (0UL)
#define TEGRA194_UART_PORT_B (1UL)
#define TEGRA194_UART_PORT_C (2UL)
#define TEGRA194_UART_PORT_D (3UL)
#define TEGRA194_UART_PORT_E (4UL)
#define TEGRA194_UART_PORT_F (5UL)

/* Default to UARTC */
static uint32_t uart_port = TEGRA194_UART_PORT_C;

static const struct tegra_uart_conf uart_t194_conf = {
    .parity     = TEGRA_UART_NO_PARITY,
    .stop_bits  = TEGRA_UART_STOP_BITS_1,
    .data_bits  = TEGRA_UART_DATA_BITS_8,
    .baud       = 115200UL,
};

static const struct spe_uart_ops legacy_ops = {
    .init           = &tegra_uart_init,
    .init_3rdparty  = &tegra_uart_3rdparty_init,
    .write_now      = &tegra_uart_write_now,
    .write          = &tegra_uart_write,
    .flush          = &tegra_uart_flush_tx_fifo,
    .flush_hw       = &tegra_uart_flush_tx_hw_fifo,
    .read           = &tegra_uart_read,
    .suspend        = &tegra_uart_suspend,
    .resume         = &tegra_uart_resume,
};

static struct spe_uart spe_uart_t194[] = {
    [TEGRA194_UART_PORT_A] = {
        .ctlr       = NULL,
        .conf       = NULL,
        .ops        = NULL,
    },
    [TEGRA194_UART_PORT_B] = {
        .ctlr       = NULL,
        .conf       = NULL,
        .ops        = NULL,
    },
    [TEGRA194_UART_PORT_C] = {
        .ctlr       = &tegra_uart_ctlr_uartc,
        .conf       = &uart_t194_conf,
        .ops        = &legacy_ops,
    },
    [TEGRA194_UART_PORT_D] = {
        .ctlr       = NULL,
        .conf       = NULL,
        .ops        = NULL,
    },
    [TEGRA194_UART_PORT_E] = {
        .ctlr       = NULL,
        .conf       = NULL,
        .ops        = NULL,
    },
    [TEGRA194_UART_PORT_F] = {
        .ctlr       = NULL,
        .conf       = NULL,
        .ops        = NULL,
    },
};

struct spe_uart *spe_uart_get(const uint32_t port_id)
{
    struct spe_uart *uart = NULL;

    if (port_id > ARRAY_SIZE(spe_uart_t194)) {
        goto out;
    }

    uart = &spe_uart_t194[port_id];
    uart_port = port_id;

out:
    return uart;
}

void spe_uart_write_now(const char *buf, uint32_t count)
{
    struct spe_uart *uart = &spe_uart_t194[uart_port];

    uart->ops->write_now(uart->ctlr, buf, count);
}
