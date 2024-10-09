/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */
#include <address_map_new.h>
#include <arpadctl_AO.h>

/* Late FSP headers */
#include <reg-access/reg-access.h>
#include <osa/rtos-values.h>
#include <osa/rtos-errors.h>
#include <osa/rtos-task.h>
#include <osa/rtos-queue.h>
#include <error/common-errors.h>             // for E_SUCCESS, error_t, FSP_...
#include <clk/clk-tegra.h>                   // for tegra_clk_enable, ...

/* Module-specific FSP headers */
#include <uart/sections-uart.h>              // for SECTION_UART_ ...
#include <uart/uart-errors.h>                // for E_UART_ERR_CONFIG, ...
#include <uart/tegra-uart-priv.h>            // for tegra_uart_ctlr, ...
#include <uart/uart-port.h>                  // for uart_port_...
#include <port/uart-port-priv.h>             // for uart_queue_t , struct uart_hw_handle

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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_QUEUE_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

#define UART_QUEUE_SIZE        rtosQueueSize(TEGRA_UART_BUF_SIZE, \
                                             sizeof(char))

error_t uart_port_rx_queue_send_from_isr(const struct tegra_uart_ctlr *ctlr,
                                         char *c_in,
                                         bool *isr_yield_flag)
{
    error_t ret = E_SUCCESS;
    rtosError status;
    rtosBool higher_prio_task_woken = rtosFALSE;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    status = rtosQueueSendFromISR(ctlr->rx_queue,
                                  c_in,
                                  &higher_prio_task_woken);
    if (status != E_SUCCESS) {
        ret = status;
        goto out;
    }
    *isr_yield_flag = higher_prio_task_woken;

out:
    return ret;
}

error_t uart_port_tx_queue_receive_from_isr(const struct tegra_uart_ctlr *ctlr,
                                            char *c_out,
                                            bool *isr_yield_flag)
{
    error_t ret = E_SUCCESS;
    rtosError status;
    rtosBool higher_prio_task_woken = rtosFALSE;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    status = rtosQueueReceiveFromISR(ctlr->tx_queue,
                                     c_out,
                                     &higher_prio_task_woken);
    if (status != rtosPASS) {
        ret = status;
        goto out;
    }
    *isr_yield_flag = higher_prio_task_woken;

out:
    return ret;
}

error_t uart_port_tx_queue_receive(const struct tegra_uart_ctlr *ctlr,
                                   char *c,
                                   uint32_t timeout_left)
{
    error_t   ret = E_SUCCESS;
    rtosError status;
    rtosTick  timeout = tegra_tke_convert_usecs_to_ticks(timeout_left);

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (timeout >= rtosMAX_DELAY) {
        timeout = rtosMAX_DELAY;
    }

    status = rtosQueueReceive(ctlr->tx_queue,
                              c,
                              timeout);
    if (status != rtosPASS) {
        ret = status;
        goto out;
    }

out:
    return ret;
}

error_t uart_port_tx_queue_send(const struct tegra_uart_ctlr *ctlr,
                                const char *c,
                                uint32_t timeout_left)
{
    error_t   ret = E_SUCCESS;
    rtosError status;
    rtosTick  timeout = tegra_tke_convert_usecs_to_ticks(timeout_left);

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (timeout >= rtosMAX_DELAY) {
        timeout = rtosMAX_DELAY;
    }

    status = rtosQueueSend(ctlr->tx_queue,
                           c,
                           timeout);
    if (status != rtosPASS) {
        ret = (status == E_RTOS_QUEUE_FULL) ? E_UART_QUEUE_FULL : status;
        goto out;
    }

out:
    return ret;
}

error_t uart_port_rx_queue_receive(const struct tegra_uart_ctlr *ctlr,
                                   char *c,
                                   uint32_t timeout_left)
{
    error_t   ret = E_SUCCESS;
    rtosError status;
    rtosTick  timeout = tegra_tke_convert_usecs_to_ticks(timeout_left);

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    if (timeout >= rtosMAX_DELAY) {
        timeout = rtosMAX_DELAY;
    }

    status = rtosQueueReceive(ctlr->rx_queue,
                              c,
                              timeout);
    if (status != rtosPASS) {
        ret = status;
        goto out;
    }

out:
    return ret;
}

void uart_port_isr_exit(const struct tegra_uart_ctlr *ctlr,
                        bool isr_yield_flag)
{
    (void) ctlr;

    rtosTaskYieldFromISR(isr_yield_flag);
}

error_t uart_port_tx_queue_create(struct tegra_uart_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;
    rtosError status;
    static char uart_tx_queue[UART_QUEUE_SIZE] SECTION_UART_DATA;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    status = rtosQueueCreate(uart_tx_queue,
                             ARRAY_SIZE(uart_tx_queue),
                             TEGRA_UART_BUF_SIZE,
                             sizeof(char),
                             &ctlr->tx_queue);
    if (status != rtosPASS) {
        ret = E_UART_INIT_FAIL;
        goto out;
    }

out:
    return ret;
}

error_t uart_port_rx_queue_create(struct tegra_uart_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;
    rtosError status;
    static char uart_rx_queue[UART_QUEUE_SIZE] SECTION_UART_DATA;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    status = rtosQueueCreate(uart_rx_queue,
                             ARRAY_SIZE(uart_rx_queue),
                             TEGRA_UART_BUF_SIZE,
                             sizeof(char),
                             &ctlr->rx_queue);
    if (status != rtosPASS) {
        ret = E_UART_INIT_FAIL;
        goto out;
    }

out:
    return ret;
}

error_t uart_port_clk_disable(struct tegra_uart_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    tegra_clk_disable(ctlr->id.hw_handle->clk);

out:
    return ret;
}

error_t uart_port_clk_enable(struct tegra_uart_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    tegra_clk_enable(ctlr->id.hw_handle->clk);

out:
    return ret;
}

error_t uart_port_clk_reset_pulse(struct tegra_uart_ctlr *ctlr,
                                  uint32_t delay)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    tegra_clk_reset_pulse(ctlr->id.hw_handle->rst, delay);

out:
    return ret;
}

error_t uart_port_clk_set_rate(struct tegra_uart_ctlr *ctlr,
                               uint32_t rate)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_UART_NULL_PTR;
        goto out;
    }

    tegra_clk_set_rate(ctlr->id.hw_handle->clk, rate);

out:
    return ret;
}

bool uart_port_is_configured(const struct tegra_uart_ctlr *ctlr)
{
    bool ret = false;

    if (ctlr == NULL) {
        goto out;
    }

    ret = tegra_clk_is_clk_enabled(ctlr->id.hw_handle->clk) &&
            tegra_reset_is_reset_deasserted(ctlr->id.hw_handle->rst);

out:
    return ret;
}

error_t uart_port_init(struct tegra_uart_ctlr *ctlr)
{
    writel(0x441, NV_ADDRESS_MAP_PADCTL_A14_BASE + PADCTL_AO_UART3_RX_0);
    writel(0x441, NV_ADDRESS_MAP_PADCTL_A14_BASE + PADCTL_AO_UART3_TX_0);

    return E_SUCCESS;
}

uint64_t uart_port_get_elapsed_usecs(const uint64_t start)
{
    return tegra_tke_get_elapsed_usecs64(start);
}
