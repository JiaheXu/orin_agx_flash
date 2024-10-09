/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef SPI__SPI_H
#define SPI__SPI_H
#define FSP__SPI__SPI_H                      1

/**
 * @file spi/spi.h
 * @brief functions for performing SPI operations.
 */

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for error_t, FSP__ERROR__COMMON...
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define TEGRA_SPI_XFER_FIRST_MSG    3
#define TEGRA_SPI_XFER_LAST_MSG     4
#define TEGRA_SPI_XFER_HANDLE_CACHE 5

#define TEGRA_SPI_NBITS_SINGLE  0x01            /* 1bit transfer */
#define TEGRA_SPI_NBITS_DUAL    0x02            /* 2bits transfer */
#define TEGRA_SPI_NBITS_QUAD    0x04            /* 4bits transfer, supported since T186 */

#define TEGRA_SPI_CPHA          0x01            /* clock phase */
#define TEGRA_SPI_CPOL          0x02            /* clock polarity */
#define TEGRA_SPI_MODE_0        (0 | 0)         /* (original MicroWire) */
#define TEGRA_SPI_MODE_1        (0 | TEGRA_SPI_CPHA)
#define TEGRA_SPI_MODE_2        (TEGRA_SPI_CPOL | 0)
#define TEGRA_SPI_MODE_3        (TEGRA_SPI_CPOL | TEGRA_SPI_CPHA)
#define TEGRA_SPI_CS_HIGH       0x04            /* chipselect active high */
#define TEGRA_SPI_LSB_FIRST     0x08            /* per-word bits-on-wire */
#define TEGRA_SPI_3WIRE         0x10            /* SI/SO signals shared */
#define TEGRA_SPI_LSBYTE_FIRST  0x1000          /* per-word bytes-on-wire */

/*
 * Declaration for spi_ctlr that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
struct spi_ctlr;

/**
 * @brief SPI DMA descriptor
 *
 * Provides a data type for the SPI dma channels for half/full duplex modes.
 *
 * @tx          TX dma channel
 * @rx          TX dma channel
 */
typedef struct {
    uint32_t tx, rx;
} spi_dma_chan;

/*
 * Please refer to Tegra SPI HW manual for the meaning of following SPI timing
 * settings
 * chip_select:             specify which CS pin for this structure
 * cs_setup_clk_count:      CS pin setup clock count
 * cs_hold_clk_count:       CS pin hold clock count
 * cs_inactive_cycles:      CS pin inactive clock count
 * set_rx_tap_delay:        specify if the SPI device need to set RX tap delay
 * spi_max_clk_rate:        specify the default clock rate of SPI client
 * spi_no_dma:              flag to indicate pio or dma mode
 */
struct spi_client_setting {
    uint32_t    spi_max_clk_rate;
    int32_t     cs_setup_clk_count;
    int32_t     cs_hold_clk_count;
    int32_t     cs_inactive_cycles;
    uint8_t     chip_select;
    bool        set_rx_tap_delay;
    bool        spi_no_dma;
};

/**
 * @brief SPI master initialization configuration
 *
 * @dma_id              DMA ID of the associated GPCDMA channel
 * @dma_channel         specify which GPCDMA channels to use for tx, rx
 * @dma_slave_req       specify which GPCDMA slave req type is for this DMA
 *                      request. Please check argpcdma*.h for appropriate
 *                      setting
 * @spi_max_clk_rate    specify the max clock rate of SPI controller
 */
struct spi_master_init {
    void                   *dma_id;
    spi_dma_chan           dma_chans;
    uint32_t               spi_max_clk_rate;
    uint8_t                dma_slave_req;
};

/**
 * @brief SPI transfer descriptor
 *
 * @tx_buf          data to write to SPI device, or NULL if this is RX transfer
 *                  the buffer memory need to be aligned for DMA transfer
 * @rx_buf          buffer to hold read data, or NULL if this is TX transfer
 *                  the buffer memory need to be aligned for DMA transfer
 * @len             size of rx or tx buffer in bytes
 * @spi_clk_rate    specify clock rate for current transfer
 * @mode            spi transfer mode.
 * @flags           Indicate first/last message.
 * @tx_nbits        number of bits used for writing
 * @rx_nbits        number of bits used for reading
 * @chip select     chip select to be used for the transfer
 * @bits_per_word   select bits_per_word
 *
 * When SPI can transfer in 1x,2x or 4x. It can get this tranfer information
 * from device through tx_nbits and rx_nbits. In Bi-direction, these
 * two should both be set. User can set transfer mode with SPI_NBITS_SINGLE(1x)
 * SPI_NBITS_DUAL(2x) and SPI_NBITS_QUAD(4x) to support these three transfer.
 */
struct spi_xfer {
    const void      *tx_buf;
    void            *rx_buf;
    uint32_t        len;
    uint32_t        spi_clk_rate;
    uint16_t        mode;
    uint16_t        flags;
    uint8_t         tx_nbits;
    uint8_t         rx_nbits;
    uint8_t         chip_select;
    uint8_t         bits_per_word;
};

/**
 * @brief SPI controller irq handler when configured as master.
 *
 * @jama_func_req_id xxxxxxxx
 *
 * This function gets called whenever a SPI controller configured.
 * interrupt is triggered.
 *
 * @pre the function spi_init() has been called
 *
 * @param[in] data pointer for HSP context.
 *
 * @return None
 */
void spi_master_isr(void *data);

/**
 * @brief save the SPI controller context across low power state
 *
 * @jama_func_req_id xxxxxxxx
 *
 * This function restores the required controller context across power
 * gating the hardware(SC7).
 *
 * @pre the function spi_init() has been called
 *
 * @param[in] id        SPI controller context
 *
 * @retval E_SUCCESS      indicates success
 * @retval E_SPI_NULL_PTR invalid id paramter passed
 */
error_t spi_suspend(struct spi_ctlr *ctlr);

/**
 * @brief resume the SPI controller context across low power state
 *
 * @jama_func_req_id xxxxxxxx
 *
 * This function restores the required controller context across power
 * gating the hardware(SC7).
 *
 * @pre the function spi_init() has been called
 *
 * @param[in] id        SPI controller context
 *
 * @retval E_SUCCESS      indicates success
 * @retval E_SPI_NULL_PTR invalid id paramter passed
 */
error_t spi_resume(struct spi_ctlr *ctlr);

/**
 * @brief Execute a SPI transfer with the controller as the master.
 *
 * @jama_func_req_id xxxxxxxxx
 *
 * @pre the function spi_init() has been called
 *
 * @param[in] id        SPI controller context
 * @param[in] xfer      SPI transfer details
 *
 * @retval E_SUCCESS                  indicates success
 * @retval E_SPI_NULL_PTR             invalid id paramter passed
 * @retval E_SPI_INVALID_PARAM        invalid DMA channel id
 * @retval E_SPI_PORT_SYNC_TIMEOUT    transfer timed out
 */
error_t spi_transfer(struct spi_ctlr *ctlr,
                     struct spi_xfer *xfer);

/**
 * @brief Client/slave setup for the SPI controller.
 *
 * @jama_func_req_id xxxxxxxxx
 *
 * @pre the function spi_init() has been called
 *
 * @param[in] id        SPI controller context
 * @param[in] setting   SPI client configuration setting
 *
 * @retval E_SUCCESS                     indicates success
 * @retval E_SPI_NULL_PTR                invalid id paramter passed
 * @retval E_SPI_INVALID_PARAM           invalid controller base address
 * @retval E_SPI_PORT_INIT_FAIL          port specific init failed
 * @retval E_SPI_PORT_CHAN_SETUP_FAIL    port specific channel setup failed
 */
error_t spi_client_setup(struct spi_ctlr *ctlr,
                         struct spi_client_setting *setting);

/**
 * @brief global initialization of the SPI controller.
 *
 * @jama_func_req_id xxxxxxxxx
 *
 * @pre None
 *
 * @param[in] id        SPI controller context
 * @param[in] setting   SPI controller configuration setting
 *
 * @retval E_SUCCESS                     indicates success
 * @retval E_SPI_NULL_PTR                invalid id paramter passed
 * @retval E_SPI_INVALID_PARAM           invalid controller base address
 * @retval E_SPI_PORT_INIT_FAIL          port specific init failed
 * @retval E_SPI_PORT_CHAN_SETUP_FAIL    port specific channel setup failed
 */
error_t spi_init(struct spi_ctlr *ctlr,
                 struct spi_master_init *setting);

#endif
