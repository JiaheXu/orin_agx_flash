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

#ifndef SOC_COMMON__UART_DEPENDENCIES_H
#define SOC_COMMON__UART_DEPENDENCIES_H
#define FSP__SOC_COMMON__UART_DEPENDENCIES_H             1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>
#include <error/common-errors.h>

/* Module-specific FSP headers */

struct tegra_uart_ctlr;

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
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")

#define TEGRA_UART_BUF_SIZE          256

error_t uart_port_rx_queue_receive(const struct tegra_uart_ctlr *ctlr,
                                   char *c,
                                   uint32_t timeout_left);

error_t uart_port_rx_queue_send_from_isr(const struct tegra_uart_ctlr *ctlr,
                                         char *c_in,
                                         bool *isr_yield_flag);

error_t uart_port_tx_queue_receive_from_isr(const struct tegra_uart_ctlr *ctlr,
                                            char *c_out,
                                            bool *isr_yield_flag);

error_t uart_port_tx_queue_receive(const struct tegra_uart_ctlr *ctlr,
                                   char *c,
                                   uint32_t timeout_left);

error_t uart_port_tx_queue_send(const struct tegra_uart_ctlr *ctlr,
                                const char *c,
                                uint32_t timeout_left);

void uart_port_isr_exit(const struct tegra_uart_ctlr *ctlr,
                        bool isr_yield_flag);

bool uart_port_is_configured(const struct tegra_uart_ctlr *ctlr);

/**
 * @brief Create the UART TX sw buffer/queue for interrupt driven TX
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         tx queue creation success.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_tx_queue_create(struct tegra_uart_ctlr *ctlr);

/**
 * @brief Create the UART RX sw buffer/queue for interrupt driven RX
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         rx queue creation success.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_rx_queue_create(struct tegra_uart_ctlr *ctlr);

/**
 * @brief Delete the UART TX sw buffer/queue for interrupt driven RX
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         rx queue creation success.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_tx_queue_delete(struct tegra_uart_ctlr *ctlr);

/**
 * @brief Delete the UART RX sw buffer/queue for interrupt driven RX
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         rx queue creation success.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_rx_queue_delete(struct tegra_uart_ctlr *ctlr);

/**
 * @brief Perform the client specific controller init sequence such as pinmux
 * settings.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         init success.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_init(struct tegra_uart_ctlr *ctlr);

/**
 * @brief disable the clk for the controller corresponding to the controller ID.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         clock disabled.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_clk_disable(struct tegra_uart_ctlr *ctlr);

/**
 * @brief Enable the clk for the controller corresponding to the controller ID.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         clock enabled.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t uart_port_clk_enable(struct tegra_uart_ctlr *ctlr);

/**
 * @brief Assert, delay and deassert the reset signal.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl_id         UART Controller ID
 * @param[in] delay           Delay between assert and deassert
 * @retval    E_SUCCESS       reset successful.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 */
error_t uart_port_clk_reset_pulse(struct tegra_uart_ctlr *ctlr,
                                  uint32_t delay);

/**
 * @brief Configure the clock rate for the controller corresponding to the
 * controller ID passed.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl_id         UART Controller ID
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 */
error_t uart_port_clk_set_rate(struct tegra_uart_ctlr *ctlr, uint32_t rate);

/**
 * @brief returns the elapsed microseconds from a reference.
 *
 * @param[in] start      value of time stamp counter at the start of the period of
 *                       when the elapsed time is being computed.
 *
 * @retval               the number of microseconds that have elapsed between when
 *                       the start time was taken and this function called.
 */
uint64_t uart_port_get_elapsed_usecs(const uint64_t start);

#endif      /* UART_DEPENDENCIES_H */
