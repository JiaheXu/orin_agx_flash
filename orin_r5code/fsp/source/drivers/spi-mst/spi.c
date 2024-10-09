/*
 * Copyright (c) 2015-2022, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdbool.h>                   // for bool
#include <stdint.h>                    // for uint32_t, uint8_t, UINT32_...
#include <stddef.h>                    // for NULL

/* Early FSP headers */
#include <misc/ct-assert.h>            // for CT_ASSERT
#include <soc-common/hw-const.h>       // IWYU pragma: keep

/* Hardware headers */
#include <arspi.h>

/* Late FSP headers */
#include <misc/nvrm_drf.h>             // NV_DRF_* macros
#include <cpu/type-conversion.h>       // for fsp_c_v_ptr_to_u32
#include <error/common-errors.h>       // for error_t, E_SUCCESS
#include <misc/attributes.h>           // for UNUSED
#include <irq/safe-irqs.h>             // for in_interrupt, irq_enable, irq_set...
#include <misc/bitops.h>               // for BIT, bit_number, FSP__MISC__BITOPS_H
#include <misc/macros.h>               // for ARRAY_SIZE
#include <reg-access/reg-access.h>     // for readl_base_offset, writel_base_of...
#include <delay/delay.h>               // for udelay
#include <processor/cache-hw.h>        // for CACHE_LINE_SIZE...

/* Module specific headers */
#include <spi/spi-errors.h>            // for E_SPI_NULL_PTR, E_SPI_...
#include <spi/spi.h>                   // for spi_init...
#include <spi/sections-spi.h>          // Immune from CT_ASSERT protection
#include <spi/spi-priv.h>              // for struct spi_ctlr, spi_ctrl...
#include <spi/spi-port.h>              // for spi_port_...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DELAY__DELAY_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SPI__SPI_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SPI__SPI_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SPI__SPI_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SPI__SPI_PORT_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define SPI_ERR             (NV_DRF_DEF(SPI, FIFO_STATUS, ERR, ERROR))
#define SPI_FIFO_ERROR      (NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_UNF, \
                                 ERROR) | \
                             NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_OVF, \
                                 ERROR) | \
                             NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_UNF, \
                                 ERROR) | \
                             NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_OVF, \
                                 ERROR))
#define SPI_FIFO_EMPTY      (NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_EMPTY, \
                                 EMPTY) | \
                             NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_EMPTY, \
                                 EMPTY))

#define SPI_FIFO_DEPTH              64U
#define DATA_DIR_TX                 BIT(0)
#define DATA_DIR_RX                 BIT(1)

#define SPI_FIFO_FLUSH_MAX_DELAY    2000UL  /* usec */

#define SPI_SPEED_TAP_DELAY_MARGIN  35000000UL  /* usec */
#define SPI_DEFAULT_RX_TAP_DELAY    10U  /* usec */

#define SPI_TRANSFER_TIMEOUT        3000UL /* usec */

static inline uint32_t
spi_readl(struct spi_ctlr *ctlr,
                uint32_t reg)
{
    return readl(ctlr->conf.base_addr + reg);
}

static inline void
spi_writel(struct spi_ctlr *ctlr,
                 uint32_t val,
                 uint32_t reg)
{
    writel(val, ctlr->conf.base_addr + reg);
}

SECTION_SPI_TEXT static void
handle_xfer_err(struct spi_ctlr *ctlr)
{
    uint32_t errs = 0UL;

    if ((ctlr->cur_direction & DATA_DIR_TX) && (ctlr->tx_status)) {
        errs += 1UL;
    }

    if ((ctlr->cur_direction & DATA_DIR_RX) && (ctlr->rx_status)) {
        errs += 2UL;
    }

    if (errs != 0UL) {
        spi_port_clk_reset_pulse(ctlr, 2);
    }
}

SECTION_SPI_TEXT static void
spi_clear_status(struct spi_ctlr *ctlr)
{
    uint32_t val;

    /* Write 1 to clear status register */
    val = spi_readl(ctlr, SPI_TRANSFER_STATUS_0);
    spi_writel(ctlr, val, SPI_TRANSFER_STATUS_0);

    val = spi_readl(ctlr, SPI_INTR_MASK_0);
    if ((val & NV_DRF_DEF(SPI, INTR_MASK, RDY_INTR_MASK, DEFAULT_MASK)) == 0UL) {
        val |= (NV_DRF_DEF(SPI, INTR_MASK, RDY_INTR_MASK, DEFAULT_MASK) |
                NV_DRF_DEF(SPI, INTR_MASK, RX_FIFO_UNF_INTR_MASK,
                           DEFAULT_MASK) |
                NV_DRF_DEF(SPI, INTR_MASK, TX_FIFO_UNF_INTR_MASK,
                           DEFAULT_MASK) |
                NV_DRF_DEF(SPI, INTR_MASK, RX_FIFO_OVF_INTR_MASK,
                           DEFAULT_MASK) |
                NV_DRF_DEF(SPI, INTR_MASK, TX_FIFO_OVF_INTR_MASK,
                           DEFAULT_MASK));
        spi_writel(ctlr, val, SPI_INTR_MASK_0);
    }

    /* Clear FIFO status error if any */
    val = spi_readl(ctlr, SPI_FIFO_STATUS_0);
    if (val & SPI_ERR) {
        spi_writel(ctlr, SPI_ERR | SPI_FIFO_ERROR, SPI_FIFO_STATUS_0);
    }
}

SECTION_SPI_TEXT void
spi_master_isr(void *data)
{
    struct spi_ctlr *ctlr = (struct spi_ctlr *)data;
    uint32_t        cmd1;
    bool            isr_yield_flag = false;

    cmd1 = spi_readl(ctlr, SPI_COMMAND_0);
    ctlr->status_reg = spi_readl(ctlr, SPI_FIFO_STATUS_0);

    if ((cmd1 & NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE)) != 0UL) {
        ctlr->tx_status = ctlr->status_reg &
                          (NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_UNF, ERROR) |
                           NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_OVF, ERROR));
    }

    if ((cmd1 & NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE)) != 0UL) {
        ctlr->rx_status = ctlr->status_reg &
                          (NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_UNF, ERROR) |
                           NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_OVF, ERROR));
    }

    spi_clear_status(ctlr);

    if (((cmd1 & NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE)) == 0UL) &&
            ((cmd1 & NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE)) == 0UL)) {
            /* SPI: spurious interrupt */
        goto out;
    }

    handle_xfer_err(ctlr);

    UNUSED((spi_port_xfer_sync_end(ctlr)));

out:
    spi_port_isr_exit(ctlr, isr_yield_flag);
}

static uint32_t
spi_calculate_curr_xfer_param(struct spi_ctlr *ctlr,
                              struct spi_xfer *xfer)
{
    uint32_t remain_len = xfer->len - ctlr->cur_pos;
    uint32_t max_word;
    uint32_t max_len;
    uint32_t total_fifo_words;
    bool     spi_no_dma = ctlr->cdata[xfer->chip_select].spi_no_dma;

    ctlr->bytes_per_word = (xfer->bits_per_word - 1) / 8 + 1;

    /*
     * SPI transfer length should be multiple of SPI word size
     * where SPI word size should be power-of-two multiple
     */
    if (ctlr->bytes_per_word == 3) {
        ctlr->bytes_per_word = 4;
    }

    if ((xfer->bits_per_word == 8 || xfer->bits_per_word == 16 ||
            xfer->bits_per_word == 32) && (remain_len > 3)) {
        ctlr->is_packed = 1;
        ctlr->words_per_32bit = 32 / xfer->bits_per_word;
    } else {
        ctlr->is_packed = 0;
        ctlr->words_per_32bit = 1;
    }

    if (ctlr->is_packed) {
        max_len = remain_len;
        if (spi_no_dma) {
            max_len = min(max_len, (SPI_FIFO_DEPTH * 4));
        }
        ctlr->curr_dma_words = max_len / ctlr->bytes_per_word;
        total_fifo_words = (max_len + 3) / 4;
    } else {
        max_word = (remain_len - 1) / ctlr->bytes_per_word + 1;
        if (spi_no_dma) {
            max_word = min(max_word, SPI_FIFO_DEPTH);
        }
        ctlr->curr_dma_words = max_word;
        total_fifo_words = max_word;
    }

    return total_fifo_words;
}

static uint32_t
spi_fill_tx_fifo_from_client_txbuf(struct spi_ctlr *ctlr,
                                   struct spi_xfer *xfer)
{
    uint32_t nbytes;
    uint32_t tx_empty_count;
    uint32_t fifo_status;
    uint32_t max_n_32bit;
    uint32_t i, count, byte_count;
    uint32_t x;
    uint32_t written_words;
    uint32_t fifo_words_left;
    uint8_t *tx_buf = (uint8_t *)xfer->tx_buf + ctlr->cur_tx_pos;

    fifo_status = spi_readl(ctlr, SPI_FIFO_STATUS_0);
    tx_empty_count = NV_DRF_VAL(SPI, FIFO_STATUS, TX_FIFO_EMPTY_COUNT,
                                fifo_status);

    if (ctlr->is_packed) {
        fifo_words_left = tx_empty_count * ctlr->words_per_32bit;
        written_words = min(fifo_words_left, ctlr->curr_dma_words);
        nbytes = written_words * ctlr->bytes_per_word;
        max_n_32bit = DIV_ROUND_UP(nbytes, 4);
        byte_count = 4UL;
    } else {
        max_n_32bit = min(ctlr->curr_dma_words, tx_empty_count);
        written_words = max_n_32bit;
        nbytes = written_words * ctlr->bytes_per_word;
        byte_count = ctlr->bytes_per_word;
    }

    for (count = 0UL; count < max_n_32bit; count++) {
        x = 0UL;
        for (i = 0UL; (i < byte_count) && nbytes; i += 1UL, nbytes -= 1UL) {
            x |= ((*tx_buf++) << (i * 8));
        }
        spi_writel(ctlr, x, SPI_TX_FIFO_0);
    }

    ctlr->cur_tx_pos += written_words * ctlr->bytes_per_word;

    return written_words;
}

static uint32_t
spi_read_rx_fifo_to_client_rxbuf(struct spi_ctlr *ctlr,
                                 struct spi_xfer *xfer)
{
    uint32_t rx_full_count;
    uint32_t fifo_status;
    uint32_t i, count;
    uint32_t x;
    uint32_t read_words = 0;
    uint32_t len;
    uint8_t  *rx_buf = (uint8_t *)xfer->rx_buf + ctlr->cur_rx_pos;

    fifo_status = spi_readl(ctlr, SPI_FIFO_STATUS_0);
    rx_full_count = NV_DRF_VAL(SPI, FIFO_STATUS, RX_FIFO_FULL_COUNT,
                               fifo_status);

    if (ctlr->is_packed) {
        len = ctlr->curr_dma_words * ctlr->bytes_per_word;
        for (count = 0UL; count < rx_full_count; count++) {
            x = spi_readl(ctlr, SPI_RX_FIFO_0);
            for (i = 0UL; len && (i < 4UL); i += 1UL, len -= 1UL) {
                *rx_buf++ = (x >> (i * 8U)) & 0xFFU;
            }
        }
        ctlr->cur_rx_pos += ctlr->curr_dma_words * ctlr->bytes_per_word;
        read_words += ctlr->curr_dma_words;
    } else {
        for (count = 0UL; count < rx_full_count; count++) {
            x = spi_readl(ctlr, SPI_RX_FIFO_0);
            for (i = 0UL; (i < ctlr->bytes_per_word); i += 1UL) {
                *rx_buf++ = (x >> (i * 8U)) & 0xFFU;
            }
        }
        ctlr->cur_rx_pos += rx_full_count * ctlr->bytes_per_word;
        read_words += rx_full_count;
    }

    return read_words;
}

static void
spi_copy_client_txbuf_to_spi_txbuf(struct spi_ctlr *ctlr,
                                   struct spi_xfer *xfer)
{
    uint32_t i;
    int32_t count;
    uint32_t x;
    uint32_t offset;
    uint8_t *tx_c_buf;
    uint32_t base;
    uint32_t *tx_buf = (uint32_t *)((uint8_t *)xfer->tx_buf + ctlr->cur_tx_pos);
    uint32_t nbytes = ctlr->curr_dma_words * ctlr->bytes_per_word;

    if (!ctlr->is_packed) {
        tx_c_buf = (uint8_t *)xfer->tx_buf + ctlr->cur_tx_pos +
                        nbytes - 1;
        /* curr_dma_words and fifo_words are same for unpacked mode */
        for (count = ctlr->curr_dma_words - 1; count >= 0; count--) {
            x = 0;
            for (i = 0; nbytes && (i < ctlr->bytes_per_word);
                        i++) {
                offset = (--nbytes) % (ctlr->bytes_per_word);
                x |= (*tx_c_buf--) << (offset * 8);
            }
            tx_buf[count] = x;
        }
        /**
         * The following cache flush is needed only for unpacked mode
         * as for packed mode you do not need to modify the tx buffer
         */
        if (xfer->flags & BIT(TEGRA_SPI_XFER_HANDLE_CACHE)) {
            /**
             * Align buffer address and length to cache line size
             * and flush the cache
             */
            base = (uint32_t) xfer->tx_buf & CACHE_LINE_MASK;
            spi_port_cache_clean((void *)base, (((ctlr->curr_dma_words * 4) +
                                 (CACHE_LINE_SIZE - 1)) & CACHE_LINE_MASK));
        }
    }
}

static void
spi_copy_spi_rxbuf_to_client_rxbuf(struct spi_ctlr *ctlr,
                                   struct spi_xfer *xfer)
{
    uint32_t i;
    uint32_t count;
    uint32_t x;
    uint32_t rx_mask;
    uint32_t base;
    uint32_t *rx_buf = (uint32_t *)((uint8_t *)xfer->rx_buf +
                       ctlr->cur_rx_pos);
    uint8_t  *rx_c_buf = (uint8_t *)xfer->rx_buf +
                         ctlr->cur_rx_pos;

    /**
     * Align buffer address and length to cache line size
     * and invalidate the cache
     */
    if (xfer->flags & BIT(TEGRA_SPI_XFER_HANDLE_CACHE)) {
        base = (uint32_t) xfer->rx_buf & CACHE_LINE_MASK;
        spi_port_cache_invalidate((void *)base,
                ((xfer->len + (CACHE_LINE_SIZE - 1)) &
                CACHE_LINE_MASK));
    }
    if (!ctlr->is_packed) {
        rx_mask = ((uint64_t)1 << xfer->bits_per_word) - 1;
        for (count = 0; count < ctlr->curr_dma_words; count++) {
            x = rx_buf[count];
            x &= rx_mask;
            rx_c_buf = (uint8_t *)&rx_buf[count];
            for (i = 0; (i < ctlr->bytes_per_word); i++) {
                *rx_c_buf++ = (x >> (i*8)) & 0xFF;
            }
        }
    }
    ctlr->cur_rx_pos += ctlr->curr_dma_words * ctlr->bytes_per_word;
}

SECTION_SPI_TEXT static error_t
spi_check_and_clear_fifo(struct spi_ctlr *ctlr)
{
    error_t  ret = E_SUCCESS;
    uint32_t status;
    uint32_t cnt;

    /* Make sure that Rx and Tx FIFO are empty */
    status = spi_readl(ctlr, SPI_FIFO_STATUS_0);
    if ((status & SPI_FIFO_EMPTY) != SPI_FIFO_EMPTY) {
        /* flush the FIFO */
        status = NV_FLD_SET_DRF_DEF(SPI, FIFO_STATUS,
                                    RX_FIFO_FLUSH, FLUSH, status);
        status = NV_FLD_SET_DRF_DEF(SPI, FIFO_STATUS,
                                    TX_FIFO_FLUSH, FLUSH, status);
        spi_writel(ctlr, status, SPI_FIFO_STATUS_0);
        for (cnt = 0UL; cnt != SPI_FIFO_FLUSH_MAX_DELAY; cnt += 1UL) {
            status = spi_readl(ctlr, SPI_FIFO_STATUS_0);
            if ((status & SPI_FIFO_EMPTY) == SPI_FIFO_EMPTY) {
                goto out;
            }
            udelay(1);
        }
        ret =  E_SPI_FIFO_NON_EMPTY;
    }

out:
    return ret;
}

SECTION_SPI_TEXT static error_t
spi_start_dma_transfer(struct spi_ctlr *ctlr,
                       struct spi_xfer *xfer)
{
    error_t  ret = E_SUCCESS;
    uint32_t val = 0;
    uint32_t cmd1;
    uint32_t intr_mask = 0UL;
    uint32_t len;
    uint32_t burst_size;

    /* For half duplex, ensure rx and tx transfers are not enabled.
     * If rx and tx transfers are enabled, return error.
     */
    if (ctlr->dma_chans.tx == ctlr->dma_chans.rx) {
        if ((ctlr->cur_direction & DATA_DIR_TX) &&
                (ctlr->cur_direction & DATA_DIR_RX)) {
            ret = E_SPI_XFER_NOT_SUPPORTED;
            goto out;
        }
    }

    val = NV_FLD_SET_DRF_NUM(SPI, DMA_BLK_SIZE, DMA_BLOCK_SIZE,
                             (ctlr->curr_dma_words - 1), val);
    spi_writel(ctlr, val, SPI_DMA_BLK_SIZE_0);

    if (ctlr->is_packed) {
        len = DIV_ROUND_UP(ctlr->curr_dma_words * ctlr->bytes_per_word, 4) * 4;
    } else {
        len = ctlr->curr_dma_words * 4;
    }

    /* Set attention level based on length of transfer.
     * DMA burst size and SPI trigger level should be same.
     */
    val = 0;
    if (len & 0xF) {
        val |= NV_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG1) |
               NV_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG1);
        burst_size = 1;
    } else if (((len) >> 4) & 0x1) {
        val |= NV_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG4) |
               NV_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG4);
        burst_size = 4;
    } else {
        val |= NV_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG8) |
               NV_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG8);
        burst_size = 8;
    }

    if ((ctlr->cur_direction & DATA_DIR_TX) ||
            (ctlr->cur_direction & DATA_DIR_RX)) {
        intr_mask = spi_readl(ctlr, SPI_INTR_MASK_0);
        intr_mask &= ~(NV_DRF_DEF(SPI, INTR_MASK ,
                                  RDY_INTR_MASK, DEFAULT_MASK) |
                       NV_DRF_DEF(SPI, INTR_MASK,
                                  TX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
                       NV_DRF_DEF(SPI, INTR_MASK,
                                  RX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
                       NV_DRF_DEF(SPI, INTR_MASK,
                                  TX_FIFO_OVF_INTR_MASK, DEFAULT_MASK) |
                       NV_DRF_DEF(SPI, INTR_MASK,
                                  RX_FIFO_OVF_INTR_MASK, DEFAULT_MASK));
        spi_writel(ctlr, intr_mask, SPI_INTR_MASK_0);
    }

    spi_writel(ctlr, val, SPI_DMA_CTL_0);
    ctlr->dma_ctrl_reg_val = val;
    if (ctlr->cur_direction & DATA_DIR_TX) {
        spi_copy_client_txbuf_to_spi_txbuf(ctlr, xfer);
        ret = spi_port_start_tx_dma(ctlr, xfer, len, burst_size);
        if (ret != E_SUCCESS) {
            goto out;
        }
    }
    if (ctlr->cur_direction & DATA_DIR_RX) {
        ret = spi_port_start_rx_dma(ctlr, xfer, len, burst_size);
        if (ret != E_SUCCESS) {
            goto out;
        }
    }

    ctlr->is_curr_dma_xfer = true;
    cmd1 = ctlr->cmd1_reg_val;
    if (ctlr->cur_direction & DATA_DIR_TX) {
        cmd1 |= NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE);
    }
    if (ctlr->cur_direction & DATA_DIR_RX) {
        cmd1 |= NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE);
    }
    spi_writel(ctlr, cmd1, SPI_COMMAND_0);

    val |= NV_DRF_DEF(SPI, DMA_CTL, DMA_EN, ENABLE);
    spi_writel(ctlr, val, SPI_DMA_CTL_0);

out:
    return ret;
}

SECTION_SPI_TEXT static error_t
spi_start_pio_transfer(struct spi_ctlr *ctlr,
                       struct spi_xfer *xfer)
{
    error_t  ret = E_SUCCESS;
    uint32_t val = 0UL;
    uint32_t cur_words;
    uint32_t intr_mask;

    if (ctlr->cur_direction & DATA_DIR_TX) {
        cur_words = spi_fill_tx_fifo_from_client_txbuf(ctlr, xfer);
    } else {
        cur_words = ctlr->curr_dma_words;
    }

    val = NV_FLD_SET_DRF_NUM(SPI, DMA_BLK_SIZE, DMA_BLOCK_SIZE,
                             (cur_words - 1), val);
    spi_writel(ctlr, val, SPI_DMA_BLK_SIZE_0);

    intr_mask = spi_readl(ctlr, SPI_INTR_MASK_0);
    intr_mask &= ~(NV_DRF_DEF(SPI, INTR_MASK , RDY_INTR_MASK, DEFAULT_MASK) |
                   NV_DRF_DEF(SPI, INTR_MASK,
                              TX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
                   NV_DRF_DEF(SPI, INTR_MASK,
                              RX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
                   NV_DRF_DEF(SPI, INTR_MASK,
                              TX_FIFO_OVF_INTR_MASK, DEFAULT_MASK) |
                   NV_DRF_DEF(SPI, INTR_MASK,
                              RX_FIFO_OVF_INTR_MASK, DEFAULT_MASK));
    spi_writel(ctlr, intr_mask, SPI_INTR_MASK_0);

    val = 0UL;
    spi_writel(ctlr, val, SPI_DMA_CTL_0);
    ctlr->dma_ctrl_reg_val = val;

    ctlr->is_curr_dma_xfer = false;
    val = ctlr->cmd1_reg_val;
    if (ctlr->cur_direction & DATA_DIR_TX) {
        val |= NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE);
    }
    if (ctlr->cur_direction & DATA_DIR_RX) {
        val |= NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE);
    }
    spi_writel(ctlr, val, SPI_COMMAND_0);

    val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, PIO, PIO, val);
    spi_writel(ctlr, val, SPI_COMMAND_0);

    return ret;
}

SECTION_SPI_TEXT static error_t
spi_clk_set_rate(struct spi_ctlr *ctlr,
                 uint32_t rate)
{
    error_t ret = E_SUCCESS;

    /* TODO: Change parent to support lower frequencies
     * Currently there is no option to change parent for a clock
     * minimum supported frequency on PLLP is 3.5MHz
     */

    spi_port_clk_disable(ctlr);

    ret = spi_port_clk_set_rate(ctlr, rate);
    if (ret != E_SUCCESS) {
        ret = E_SPI_CLK_SET_RATE;
        goto out;
    }

    ret = spi_port_clk_enable(ctlr);
    if (ret != E_SUCCESS) {
        ret = E_SPI_CLK_EN;
        goto out;
    }

out:
    return ret;
}

SECTION_SPI_TEXT error_t
spi_suspend(struct spi_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret =E_SPI_NULL_PTR;
        goto out;
    }

    if (ctlr->busy) {
        ret = E_SPI_CTLR_BUSY;
        goto out;
    }

    spi_port_clk_disable(ctlr);

out:
    return ret;
}

SECTION_SPI_TEXT error_t
spi_resume(struct spi_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret =E_SPI_NULL_PTR;
        goto out;
    }

    ret = spi_port_clk_set_rate(ctlr, ctlr->spi_clk_rate);

out:
    return ret;
}

SECTION_SPI_TEXT static void
spi_configure_cmd1_reg(struct spi_ctlr *ctlr,
                       struct spi_xfer *xfer)
{
    uint32_t  command1;
    uint32_t  cs_pol_bit[MAX_CHIP_SELECT] = {
                NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE0, HIGH),
                NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE1, HIGH),
                NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE2, HIGH),
                NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE3, HIGH),
    };

    command1 = ctlr->def_cmd1_reg_val;
    if (xfer->mode & TEGRA_SPI_CS_HIGH) {
        command1 &= ~cs_pol_bit[xfer->chip_select];
    } else {
        command1 |= cs_pol_bit[xfer->chip_select];
    }

    command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, CS_SEL, xfer->chip_select,
                                  command1);

    ctlr->def_cmd1_reg_val = command1;

    command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,
                                  (xfer->bits_per_word - 1), command1);
    command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, MODE,
                                  (xfer->mode & 0x3), command1);
    command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SW_HW, SOFTWARE, command1);
    command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, CS_SW_VAL,
                                  (xfer->mode & TEGRA_SPI_CS_HIGH), command1);

    if (xfer->mode & TEGRA_SPI_LSBYTE_FIRST) {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Byte,
                                      FIRST, command1);
    } else {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Byte,
                                      LAST, command1);
    }

    if (xfer->mode & TEGRA_SPI_LSB_FIRST) {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Bit, FIRST, command1);
    } else {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Bit, LAST, command1);
    }

    if (xfer->mode & TEGRA_SPI_3WIRE) {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, BIDIR, BIDIR,
                        command1);
    } else {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, BIDIR, NORMAL,
                        command1);
    }

    if ((xfer->rx_nbits == TEGRA_SPI_NBITS_DUAL) ||
            (xfer->tx_nbits == TEGRA_SPI_NBITS_DUAL)) {
        command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, BOTH_EN_BIT,
                        ENABLE, command1);
    }

    if (ctlr->is_packed) {
        command1 |= NV_DRF_DEF(SPI, COMMAND, PACKED, ENABLE);
    } else {
        command1 &= ~(NV_DRF_DEF(SPI, COMMAND, PACKED, ENABLE));
    }

    command1 &= ~(NV_DRF_DEF(SPI, COMMAND, CS_SEL, DEFAULT_MASK) |
                  NV_DRF_DEF(SPI, COMMAND, Tx_EN, DEFAULT_MASK) |
                  NV_DRF_DEF(SPI, COMMAND, Rx_EN, DEFAULT_MASK));

    ctlr->cmd1_reg_val = command1;
}

SECTION_SPI_TEXT static void
spi_init_ctlr_context(struct spi_ctlr *ctlr)
{
    ctlr->cur_pos = 0UL;
    ctlr->cur_rx_pos = 0UL;
    ctlr->cur_tx_pos = 0UL;
    ctlr->tx_status = 0UL;
    ctlr->rx_status = 0UL;
    ctlr->cur_direction = 0UL;
}

SECTION_SPI_TEXT static error_t
spi_handle_message(struct spi_ctlr *ctlr,
                   struct spi_xfer *xfer)
{
    error_t ret = E_SUCCESS;
    bool    reset = false;

    if (!ctlr->is_curr_dma_xfer) {
        if (ctlr->cur_direction & DATA_DIR_RX) {
            spi_read_rx_fifo_to_client_rxbuf(ctlr, xfer);
            ctlr->cur_pos = ctlr->cur_rx_pos;
        } else {
            ctlr->cur_pos = ctlr->cur_tx_pos;
        }
    } else {
        if (ctlr->tx_dma_status != E_SUCCESS) {
            if (ctlr->cur_direction & DATA_DIR_TX) {
                spi_port_tx_dma_xfer_abort(ctlr);
                reset = true;
            }
        }
        if (ctlr->rx_dma_status != E_SUCCESS) {
            if (ctlr->cur_direction & DATA_DIR_RX) {
                spi_port_rx_dma_xfer_abort(ctlr);
                reset = true;
            }
        }
        if (reset) {
            spi_port_clk_reset_pulse(ctlr, 2);
            ret = E_SPI_DMA_XFER_FAIL;
            goto out;
        }

        if (ctlr->cur_direction & DATA_DIR_TX) {
            ctlr->cur_tx_pos += ctlr->curr_dma_words * ctlr->bytes_per_word;
            ctlr->cur_pos = ctlr->cur_tx_pos;
        }
        if (ctlr->cur_direction & DATA_DIR_RX) {
            spi_copy_spi_rxbuf_to_client_rxbuf(ctlr, xfer);
            ctlr->cur_pos = ctlr->cur_rx_pos;
        }
    }

out:
    return ret;
}

SECTION_SPI_TEXT static error_t
spi_wait_on_message_xfer(struct spi_ctlr *ctlr)
{
    error_t  ret = E_SUCCESS;
    bool     xfer_timeout = false;

    ret = irq_safe_enable(ctlr->conf.irq);
    if (ret != E_SUCCESS) {
        spi_port_clk_disable(ctlr);
        goto out;
    }
    /*
     * For synchronous transfers, the driver relies on the port specific
     * implementation of some kind of synchronization mechanism until the
     * SPI transfer is complete.
     */
    ret = spi_port_xfer_sync(ctlr, SPI_TRANSFER_TIMEOUT);
    (void)irq_safe_disable(ctlr->conf.irq);
    if (ret != E_SUCCESS) {
        xfer_timeout = true;
        spi_port_clk_reset_pulse(ctlr, 2);
    }

    if (xfer_timeout || ctlr->rx_status || ctlr->tx_status) {
        if (ctlr->is_curr_dma_xfer) {
            if (ctlr->cur_direction & DATA_DIR_TX) {
                spi_port_tx_dma_xfer_abort(ctlr);
            }
            if (ctlr->cur_direction & DATA_DIR_RX) {
                spi_port_rx_dma_xfer_abort(ctlr);
            }
        }
        ret = E_SPI_XFER_ERR;
        goto out;
    }

out:
    return ret;
}

SECTION_SPI_TEXT static error_t
spi_transfer_remain_message(struct spi_ctlr *ctlr,
                            struct spi_xfer *xfer)
{
    error_t  ret = E_SUCCESS;
    uint32_t total_fifo_words;

    total_fifo_words = spi_calculate_curr_xfer_param(ctlr, xfer);
    if (ctlr->is_curr_dma_xfer) {
        /* Make sure that Rx and Tx FIFO are empty */
        ret = spi_check_and_clear_fifo(ctlr);
        if (ret != E_SUCCESS) {
            goto out;
        }

        if (total_fifo_words > SPI_FIFO_DEPTH) {
            ret = spi_start_dma_transfer(ctlr, xfer);
        } else {
            ret = spi_start_pio_transfer(ctlr, xfer);
        }
        if (ret != E_SUCCESS) {
            goto out;
        }
    } else {
        ret = spi_start_pio_transfer(ctlr, xfer);
        if (ret != E_SUCCESS) {
            goto out;
        }
    }

    ret = spi_wait_on_message_xfer(ctlr);

out:
    return ret;
}

SECTION_SPI_TEXT static error_t
spi_start_transfer_one(struct spi_ctlr *ctlr,
                       struct spi_xfer *xfer)
{
    error_t   ret = E_SUCCESS;
    uint32_t  total_fifo_words;
    uint32_t  speed;

    spi_init_ctlr_context(ctlr);

    ctlr->cur_xfer = xfer;
    total_fifo_words = spi_calculate_curr_xfer_param(ctlr, xfer);

    if (xfer->rx_buf != NULL) {
        ctlr->cur_direction |= DATA_DIR_RX;
    }
    if (xfer->tx_buf != NULL) {
        ctlr->cur_direction |= DATA_DIR_TX;
    }

    /* Return if neither RX or TX is requested OR
     * not all words are available
     */
    if ((ctlr->cur_direction == 0UL) ||
        ((xfer->len % ctlr->bytes_per_word) != 0UL)) {
        ret = E_SPI_INVALID_PARAM;
        goto out;
    }

    speed = (xfer->spi_clk_rate != 0UL) ? xfer->spi_clk_rate :
            ctlr->cdata[xfer->chip_select].spi_max_clk_rate;
    if (speed != ctlr->spi_clk_rate) {
        ret = spi_clk_set_rate(ctlr, speed);
        if (ret != E_SUCCESS) {
            goto out;
        } else {
            ctlr->spi_clk_rate = speed;
        }
    }

    spi_clear_status(ctlr);

    spi_configure_cmd1_reg(ctlr, xfer);

    /* Make sure that Rx and Tx FIFO are empty */
    ret = spi_check_and_clear_fifo(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    if (total_fifo_words > SPI_FIFO_DEPTH) {
        ret = spi_start_dma_transfer(ctlr, xfer);
    } else {
        ret = spi_start_pio_transfer(ctlr, xfer);
    }

out:
    return ret;
}


SECTION_SPI_TEXT static error_t
spi_dma_init(struct spi_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    ret = spi_port_dma_init(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /* Full duplex transactions require 2 channels. Check your POR
     * to ensure 2 channels are supported.
     */
    if (ctlr->en_full_duplex) {
        /* check for 2 different dma channels if full duplex
         * is supported.
         */
        if (ctlr->dma_chans.tx == ctlr->dma_chans.rx) {
            ret = E_SPI_FD_MODE_NO_SUPPORT;
            goto out;
        }
    }

out:
    return ret;
}

SECTION_SPI_TEXT static void
spi_dma_config_setup(struct spi_ctlr *ctlr,
              struct spi_master_init *setting)
{
    /* TODO: figure out if there is a way to validate these params */
    ctlr->dma_id = setting->dma_id;
    ctlr->dma_chans.tx = setting->dma_chans.tx;
    ctlr->dma_chans.rx = setting->dma_chans.rx;
    ctlr->dma_slave_req = setting->dma_slave_req;
}

SECTION_SPI_TEXT error_t
spi_transfer(struct spi_ctlr *ctlr,
             struct spi_xfer *xfer)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL || xfer == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    if (!ctlr->inited) {
        ret = E_SPI_NO_INIT;
        goto out;
    }

    if (xfer->len == 0) {
        ret = E_SPI_INVALID_PARAM;
        goto out;
    }

    if (ctlr->busy) {
        ret = E_SPI_CTLR_BUSY;
        goto out;
    }

    ctlr->busy = true;

    if (xfer->flags & BIT(TEGRA_SPI_XFER_FIRST_MSG)) {
        ret = spi_start_transfer_one(ctlr, xfer);
    } else {
        ret = spi_transfer_remain_message(ctlr, xfer);
    }

    if (ret != E_SUCCESS) {
        goto out;
    }

    ret = spi_wait_on_message_xfer(ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    ret = spi_handle_message(ctlr, xfer);
    if (ret != E_SUCCESS) {
        goto out;
    }

    if (ctlr->cur_pos == xfer->len) {
        goto out;
    }

    while (1) {
        ret = spi_transfer_remain_message(ctlr, xfer);
        if (ret != E_SUCCESS) {
            goto out;
        }

        ret = spi_handle_message(ctlr, xfer);
        if (ret != E_SUCCESS) {
            goto out;
        }

        if (ctlr->cur_pos == xfer->len) {
            break;
        }
    }

out:
    /* de-assert CS to end the transaction */
    if (ret == E_SUCCESS &&
               ((xfer->flags & BIT(TEGRA_SPI_XFER_LAST_MSG)) != 0UL)) {
        spi_writel(ctlr, ctlr->def_cmd1_reg_val, SPI_COMMAND_0);
    }

    if (ctlr != NULL) {
        ctlr->busy = false;
    }

    return ret;
}

SECTION_SPI_INIT_TEXT error_t
spi_client_setup(struct spi_ctlr *ctlr,
                 struct spi_client_setting *setting)
{
    error_t                   ret = E_SUCCESS;
    struct spi_client_setting *cdata;
    uint32_t                  command2_reg;

    if (ctlr == NULL || setting == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    if (!ctlr->inited) {
        ret = E_SPI_NO_INIT;
        goto out;
    }

    if (ctlr->busy) {
        ret = E_SPI_CTLR_BUSY;
        goto out;
    }

    if (setting->spi_max_clk_rate == 0UL) {
        ret = E_SPI_INVALID_CLK_RATE;
        goto out;
    }

    cdata = &ctlr->cdata[setting->chip_select];
    cdata->chip_select = setting->chip_select;
    cdata->cs_setup_clk_count = setting->cs_setup_clk_count;
    cdata->cs_hold_clk_count = setting->cs_hold_clk_count;
    cdata->cs_inactive_cycles = setting->cs_inactive_cycles;
    cdata->set_rx_tap_delay = setting->set_rx_tap_delay;
    cdata->spi_max_clk_rate = setting->spi_max_clk_rate;
    cdata->spi_no_dma = setting->spi_no_dma;

    ret = spi_port_clk_set_rate(ctlr, setting->spi_max_clk_rate);
    if (ret != E_SUCCESS) {
        ret = E_SPI_CLK_SET_RATE;
        goto out;
    }
    ctlr->spi_clk_rate = setting->spi_max_clk_rate;

    ctlr->def_cmd2_reg_val = spi_readl(ctlr, SPI_COMMAND2_0);
    command2_reg = ctlr->def_cmd2_reg_val;

    if (cdata->set_rx_tap_delay) {
        if (ctlr->spi_clk_rate > SPI_SPEED_TAP_DELAY_MARGIN) {
            command2_reg = NV_FLD_SET_DRF_NUM(SPI, COMMAND2,
                                              Rx_Clk_TAP_DELAY,
                                              SPI_DEFAULT_RX_TAP_DELAY,
                                              command2_reg);
        }
    }
    spi_writel(ctlr, command2_reg, SPI_COMMAND2_0);

out:
    return ret;
}

SECTION_SPI_INIT_TEXT error_t
spi_init(struct spi_ctlr *ctlr,
         struct spi_master_init *setting)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL || setting == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    if (ctlr->inited) {
        ret = E_SPI_REINIT;
        goto out;
    }

    ctlr->spi_clk_rate = setting->spi_max_clk_rate;
    ctlr->busy = false;

    spi_dma_config_setup(ctlr, setting);

    /* Set SPI controller clock rate */
    ret = spi_port_clk_set_rate(ctlr, ctlr->spi_clk_rate);
    if (ret!= E_SUCCESS) {
        goto out;
    }

    /* enable the clock for SPI controller */
    ret = spi_port_clk_enable(ctlr);
    if (ret!= E_SUCCESS) {
        goto out;
    }

    /* assert followed by deassert the reset for SPI controller */
    ret = spi_port_clk_reset_pulse(ctlr, 2);
    if (ret!= E_SUCCESS) {
        goto out;
    }

    /* Initialize the SPI DMA */
    ret = spi_dma_init(ctlr);
    if (ret!= E_SUCCESS) {
        spi_port_clk_disable(ctlr);
        goto out;
    }

    if (ctlr->conf.irq != UINT32_MAX) {
        /* register irq handler */
        ret = irq_safe_set_handler(ctlr->conf.irq, spi_master_isr, ctlr);
        if (ret != E_SUCCESS) {
            spi_port_clk_disable(ctlr);
            goto out;
        }
    }

    /* port specific SPI controller setup callout */
    ret = spi_port_init(ctlr);
    if (ret != E_SUCCESS) {
        spi_port_clk_disable(ctlr);
        goto out;
    }

    ctlr->def_cmd1_reg_val = NV_DRF_DEF(SPI, COMMAND, M_S, MASTER) |
                             NV_DRF_DEF(SPI, COMMAND, CS_SW_VAL, HIGH) |
                             NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE3, HIGH) |
                             NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE2, HIGH) |
                             NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE1, HIGH) |
                             NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE0, HIGH);
    spi_writel(ctlr, ctlr->def_cmd1_reg_val, SPI_COMMAND_0);

    ctlr->inited = true;

out:
    return ret;
}
