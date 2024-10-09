/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef UART_TEGRA_PRIV_H
#define UART_TEGRA_PRIV_H

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>
#include <clk/clk-tegra.h>
#include <osa/rtos-queue.h>

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
    CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")

/**
 * @brief UART controller identification structure
 *
 * Structure that describes the identification of a UART controller.
 *
 * @base_addr   base address of the controller
 * @clk         controller clock config data
 * @reset       controller reset config data
 * @irq         controller irq
 */
struct tegra_uart_id {
    uint32_t                    base_addr;
    const struct tegra_clk      *clk;
    const struct tegra_rst      *reset;
    uint32_t                    irq;
};

/**
 * @brief UART controller context structure
 *
 * Structure that is used by the UART driver to manage the UART controller
 * during run time.
 *
 * @conf         UART controller instance parameters
 * @tx_queue     UART SW TX FIFO
 * @rx_queue     UART SW RX FIFO
 * @tx_sem       tx semaphore for mutual exlusion
 * @initialized  UART controller initialization status
 */
struct tegra_uart_ctlr {
    const struct tegra_uart_id    id;
	rtosQueueHandle                  tx_queue;
	rtosQueueHandle                  rx_queue;
    bool                          initialized;
};

#endif