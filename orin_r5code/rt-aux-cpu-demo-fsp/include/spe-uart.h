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

#ifndef SPE_UART_H
#define SPE_UART_H

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* local os headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>

/* local headers */

/*
 * Declaration for tegra_uart_ctlr that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
struct tegra_uart_ctlr;

/*
 * Declaration for tegra_uart_conf that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
struct tegra_uart_conf;

struct spe_uart_ops {
    error_t (*init)(struct tegra_uart_ctlr * const ctlr,
                    const struct tegra_uart_conf * const conf);
    error_t (*init_3rdparty)(struct tegra_uart_ctlr * const ctlr,
                            const struct tegra_uart_conf * const conf);
    error_t (*write)(const struct tegra_uart_ctlr * const ctlr,
                     const char *buf,
                     uint32_t count,
                     uint32_t *written,
                     uint64_t timeout);
    error_t (*write_now)(const struct tegra_uart_ctlr * const ctlr,
                         const char *buf,
                         uint32_t count);
    error_t (*read)(const struct tegra_uart_ctlr * const ctlr,
                    char *buf,
                    uint32_t count,
                    uint32_t *read,
                    uint64_t timeout);
    error_t (*flush)(const struct tegra_uart_ctlr * const ctlr);
    error_t (*flush_hw)(const struct tegra_uart_ctlr * const ctlr);
    error_t (*suspend)(struct tegra_uart_ctlr * const ctlr);
    error_t (*resume)( struct tegra_uart_ctlr * const ctlr);
};

struct spe_uart {
    struct tegra_uart_ctlr        *ctlr;
    const struct tegra_uart_conf  *const conf;
    const struct spe_uart_ops     *ops;
};

struct spe_uart *spe_uart_get(const uint32_t port_id);

void spe_uart_write_now(const char *buf, uint32_t count);

#endif
