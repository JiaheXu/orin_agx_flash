/*
 * Copyright (c) 2017-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>

#include <spi/spi-priv.h>

#define SPI_DEFAULT_MAX_CLK 10000000 /*10Mhz*/

struct spi_slave_init_setup {
    void                   *dma_id;
    spi_dma_chan           dma_chans;
	uint32_t               spi_max_clk_rate;
	uint8_t                dma_slave_req;
};

void spi_slave_isr(void *data);

/*
 * Initialize and set the parameters of the requested SPI device
 *
 * Parameters:
 * id:		SPI controller instance
 * setting:	SPI device setting
 *
 * Returns:
 * 0: Success
 * Negative: error
 */
error_t spi_slave_setup(struct spi_ctlr *ctlr, struct spi_client_setting *setting);

/*
 * Execute SPI transfer(s) on the specified SPI slave device
 *
 * This function can transfer one or multiple SPI transfers.
 *
 * The code that submits tegra_spi_xfer to the lower layers is responsible for
 * managing its memory.
 *
 * Parameters:
 * id:		SPI controller instance
 * xfer:	SPI transfer(s)
 *
 * Returns:
 * 0: Success
 * Negative: error
 */
error_t spi_slave_transfer(struct spi_ctlr *ctlr, struct spi_xfer *xfer);

/*
 * Initialize and set up the controller configuration
 *
 * Parameters:
 * id:		SPI controller instance
 * setting:	SPI controller setting
 *
 * Returns:
 * 0: Success
 * Negative: error
 */
error_t spi_slave_init(struct spi_ctlr *ctlr, struct spi_slave_init_setup *setting);

#endif
