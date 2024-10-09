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

#ifndef SPI__SPI_PORT_H
#define SPI__SPI_PORT_H
#define FSP__SPI__SPI_PORT_H                      1

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

struct spi_ctlr;

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

void spi_port_isr_exit(const struct spi_ctlr *ctlr,
                       bool isr_yield_flag);

error_t  spi_port_init(struct spi_ctlr *ctlr);

void spi_port_cache_invalidate(void *base,
                               size_t length);

void spi_port_cache_clean(void *base,
                          size_t length);

/**
 * @brief Enable the clk for the controller corresponding to the controller ID.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         clock enabled.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
void  spi_port_clk_disable(struct spi_ctlr *ctlr);

/**
 * @brief Enable the clk for the controller corresponding to the controller ID.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * @param[in] ctrl              UART Controller instance
 * @retval    E_SUCCESS         clock enabled.
 * @retval    E_UART_NULL_PARAM Null ctrl parameter.
 */
error_t spi_port_clk_enable(struct spi_ctlr *ctlr);

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
error_t spi_port_clk_reset_pulse(struct spi_ctlr *ctlr,
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
error_t spi_port_clk_set_rate(struct spi_ctlr *ctlr,
                              uint32_t rate);

error_t spi_port_rx_dma_xfer_abort(struct spi_ctlr *ctlr);

error_t spi_port_tx_dma_xfer_abort(struct spi_ctlr *ctlr);

error_t spi_port_start_rx_dma(struct spi_ctlr *ctlr,
                              struct spi_xfer *xfer,
                              uint32_t len,
                              uint32_t burst_size);

error_t spi_port_start_tx_dma(struct spi_ctlr *ctlr,
                              struct spi_xfer *xfer,
                              uint32_t len,
                              uint32_t burst_size);

error_t spi_port_dma_init(struct spi_ctlr *ctlr);

/**
 * @brief spi transfer sync hook
 *
 * This function is used to block/wait until the synchronous
 * transfer is finished.
 *
 * @pre the function spi_slave_init() has been called
 *
 * @param[in] ctlr    SPI controller instance
 * @param[in] timeout Timeout to wait before bailing out
 *
 * @retval E_SUCESS                  on success
 * @retval E_SPI_INVALID_PARAM       invalid DMA channel id
 * @retval E_SPI_PORT_SYNC_TIMEOUT   timed out for synchronous transfers
 */
error_t spi_port_xfer_sync(struct spi_ctlr *ctlr,
                           uint32_t timeout);

/**
 * @brief spi transfer sync complete hook
 *
 * This function is used to notify/unblock the task that is blocked on
 * a synchronous spi transfer.
 *
 * @pre the function spi_init()/spi_slave_init() has been called
 * @note called from spi_slave_isr()/spi_isr() for synchronous transfers
 *
 * @param[in] ctlr    SPI controller instance
 *
 * @retval E_SUCESS              on success
 * @retval E_SPI_INVALID_PARAM   invalid SPI ctlr param
 */
error_t spi_port_xfer_sync_end(struct spi_ctlr *ctlr);

#endif      /* UART_DEPENDENCIES_H */
