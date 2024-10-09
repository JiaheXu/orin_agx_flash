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

#ifndef SPI_PRIV_H
#define SPI_PRIV_H
#define FSP__SPI__SPI_PRIV_H                      1

/**
 * @file spi/spi-priv.h
 * @brief Structures that are internal to the SPI master and slave driver
 */

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <spi/spi.h>

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SPI__SPI_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#define MAX_CHIP_SELECT 4U

/**
 * @brief SPI controller configuration structure
 *
 * Structure that describes the configuration of a SPI controller.
 *
 * @base_addr   base address of the controller
 * @irq         controller irq
 * @hw_handle   port specific hw handle
 */
struct spi_conf {
    uint32_t                    base_addr;
    uint32_t                    irq;
    const struct spi_hw_handle  *hw_handle;
};

/**
 * @brief SPI controller context structure
 *
 * Structure that is used by the SPI driver to manage the SPI controller
 * during run time.
 *
 * @conf                    SPI controller instance configuration
 * @cdata                   client configuration data buffer
 * @dma_id                  DMA instance
 * @dma_chans               DMA channels with in the instance
 * @cur_xfer                current transfer details
 * @spi_clk_rate            controller clock rate
 * @bits_per_word           bits per packet
 * @words_per_32bit         packets per 32-bit
 * @cur_pos                 current xfer position
 * @bytes_per_word          bytes per packet
 * @cur_dma_words           current dma words count
 * @cur_direction           current xfer direction
 * @cur_rx_pos              current rx position for the rx xfer
 * @cur_tx_pos              current tx position for the tx xfer
 * @dma_buf_size            dma buffer size
 * @max_buf_size            max buffer size
 * @tx_status               tx xfer status
 * @rx_status               rx xfer status
 * @status_reg              fifo status register value
 * @cmd1_reg_val            current command1 register value
 * @dma_ctrl_reg_val        dma control register value
 * @def_cmd1_reg_val        default command1 register value
 * @def_cmd2_reg_val        default command2 register value
 * @spi_cs_timing           chip select timing
 * @rx_dma_buf              rx dma buffer
 * @tx_dma_buf              tx dma buffer
 * @def_chip_select         default chip select
 * @dma_slave_req           DMA slave request
 * @dma_status              status of the dma transfer
 * @is_curr_dma_xfer        flag indicating PIO/DMA current transfer
 * @is_hw_based_cs          HW based chip select config
 * @is_packed               current transfer packed status
 * @busy                    controller status
 * @slave_dma_support       DMA support for slave
 * @inited                  controller initialization state
 */
struct spi_ctlr {
    const struct spi_conf     conf;
    struct spi_client_setting cdata[MAX_CHIP_SELECT];
    void                      *dma_id;
    spi_dma_chan              dma_chans;
    struct spi_xfer           *cur_xfer;
    uint32_t                  spi_clk_rate;
    uint32_t                  bits_per_word;
    uint32_t                  words_per_32bit;
    uint32_t                  cur_pos;
    uint32_t                  bytes_per_word;
    uint32_t                  curr_dma_words;
    uint32_t                  cur_direction;
    uint32_t                  cur_rx_pos;
    uint32_t                  cur_tx_pos;
    uint32_t                  dma_buf_size;
    uint32_t                  max_buf_size;
    uint32_t                  tx_dma_status;
    uint32_t                  rx_dma_status;
    uint32_t                  tx_status;
    uint32_t                  rx_status;
    uint32_t                  status_reg;
    uint32_t                  cmd1_reg_val;
    uint32_t                  dma_ctrl_reg_val;
    uint32_t                  def_cmd1_reg_val;
    uint32_t                  def_cmd2_reg_val;
    uint32_t                  spi_cs_timing;
    uint32_t                  *rx_dma_buf;
    uint32_t                  *tx_dma_buf;
    uint8_t                   def_chip_select;
    uint8_t                   dma_slave_req;
    uint8_t                   dma_status;
    bool                      is_curr_dma_xfer;
    bool                      is_hw_based_cs;
    bool                      is_packed;
    bool                      busy;
    bool                      en_full_duplex;
    bool                      slave_dma_support;
    bool                      inited;
};

#endif
