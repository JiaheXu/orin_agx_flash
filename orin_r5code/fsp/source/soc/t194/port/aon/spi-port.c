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
#include <misc/ct-assert.h>                // for CT_ASSERT

/* Hardware headers */
#include <arspi.h>

/* Late FSP headers */
#include <osa/rtos-task.h>                 // for rtosTaskYieldFromISR
#include <osa/rtos-semaphore.h>            // for rtosSemaphore...
#include <osa/rtos-values.h>               // for rtosBool
#include <error/common-errors.h>           // for E_SUCCESS, error_t, FSP_...
#include <clk/clk-tegra.h>                 // for tegra_clk_enable, ...
#include <cpu/type-conversion.h>           // for tegra_clk_enable, ...
#include <cpu/cache.h>                     // for cache_clean, ...

/* Module-specific FSP headers */
#include <spi/sections-spi.h>              // for SECTION_SPI_ ...
#include <spi/spi-errors.h>                // for E_SPI_ERR_CONFIG, ...
#include <spi/spi-priv.h>                  // for spi_ctlr, ...
#include <spi/spi-port.h>                  // for spi_port_...
#include <port/spi-port-priv.h>            // Immune from CT_ASSERT
#include <gpcdma/gpcdma.h>
#include <gpcdma/gpcdma-errors.h>

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
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

static rtosSemaphoreHandle  spi_sem;
static uint8_t              spi_sem_buf[rtosSemaphoreSize()];

void spi_port_isr_exit(const struct spi_ctlr *ctlr,
                       bool isr_yield_flag)
{
    (void) ctlr;

    rtosTaskYieldFromISR(isr_yield_flag);
}

void spi_port_cache_invalidate(void * base,
                               size_t length)
{
    cache_invalidate(base, length);
}

void spi_port_cache_clean(void * base,
                          size_t length)
{
    cache_clean(base, length);
}

error_t spi_port_dma_init(struct spi_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL || ctlr->dma_id == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = gpcdma_init(ctlr->dma_id);

out:
    return ret;
}

error_t spi_port_init(struct spi_ctlr *ctlr)
{
    error_t ret;
    rtosUCount depth;

    if (ctlr == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = rtosSemaphoreCreateBinary(spi_sem_buf,
                                    &spi_sem);
    if (ret != rtosPASS) {
        ret = E_SPI_PORT_INIT_FAIL;
        goto out;
    }

    ret = rtosSemaphoreGetCountDepth(spi_sem, &depth);
    if (ret != rtosPASS) {
        ret = E_SPI_PORT_INIT_FAIL;
        goto out;
    }

    if (depth == 0U) {
        ret = E_SUCCESS;
        goto out;
    }

    /*
     * Make sure the semaphore state is empty initially so that it
     * blocks on a notification from ISR.
     */
    ret = rtosSemaphoreAcquire(spi_sem, (rtosTick)0);
    if (ret != rtosPASS) {
        ret = E_SPI_PORT_INIT_FAIL;
        goto out;
    }
    ret = E_SUCCESS;

out:
    return ret;
}

error_t spi_port_xfer_sync(struct spi_ctlr *ctlr,
                           uint32_t timeout)
{
    error_t ret;

    if (ctlr == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = rtosSemaphoreAcquire(spi_sem, (rtosTick)timeout);
    if (ret != rtosPASS) {
        ret = E_SPI_PORT_SYNC_TIMEOUT;
        goto out;
    }
    ret = E_SUCCESS;

out:
    return ret;
}

error_t spi_port_xfer_sync_end(struct spi_ctlr *ctlr)
{
    error_t ret;
    rtosBool higher_prio_task_woken = rtosFALSE;

    if (ctlr == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = rtosSemaphoreReleaseFromISR(spi_sem,
                                      &higher_prio_task_woken);
    if (ret == rtosPASS) {
        rtosTaskYieldFromISR(higher_prio_task_woken);
        ret = E_SUCCESS;
    }

out:
    return ret;
}

void spi_port_clk_disable(struct spi_ctlr *ctlr)
{
    if (ctlr != NULL) {
        tegra_clk_disable(ctlr->conf.hw_handle->clk);
    }
}

error_t spi_port_clk_enable(struct spi_ctlr *ctlr)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = tegra_clk_enable(ctlr->conf.hw_handle->clk);
    if (ret != 0) {
        ret = E_SPI_CLK_EN;
    }

out:
    return ret;
}

error_t spi_port_tx_dma_xfer_abort(struct spi_ctlr *ctlr)
{
    return gpcdma_abort(ctlr->dma_id, ctlr->dma_chans.tx);
}

error_t spi_port_rx_dma_xfer_abort(struct spi_ctlr *ctlr)
{
    return gpcdma_abort(ctlr->dma_id, ctlr->dma_chans.rx);
}

error_t spi_port_start_rx_dma(struct spi_ctlr *ctlr,
                              struct spi_xfer *xfer,
                              uint32_t len,
                              uint32_t burst_size)
{
    uint8_t *rx_buf = (uint8_t *)xfer->rx_buf + ctlr->cur_rx_pos;

    struct gpcdma_xfer dma_xfer = {
        .direction = GPCDMA_XFER_DIR_IO_TO_MEM,
        .bus_width = GPCDMA_IO_BUS_WIDTH_32,
        .burst_size = burst_size,
        .src_addr = (uint64_t)(ctlr->conf.base_addr + SPI_RX_FIFO_0),
        .src_addr_wrap = 1,
        .dst_addr = (uint64_t)(fsp_c_u8_ptr_to_u32(rx_buf)),
        .dst_addr_wrap = 0,
        .xfer_count = len,
        .en_flow_ctrl = true,
        .slave_req = ctlr->dma_slave_req,
        .synchronous = true,
        .timeout = 10,
        .callback = NULL,
        .callback_param = NULL,
    };

    return gpcdma_transfer(ctlr->dma_id, ctlr->dma_chans.rx, &dma_xfer);
}

error_t spi_port_start_tx_dma(struct spi_ctlr *ctlr,
                              struct spi_xfer *xfer,
                              uint32_t len,
                              uint32_t burst_size)
{
    uint8_t *tx_buf = (uint8_t *)xfer->tx_buf + ctlr->cur_tx_pos;

    struct gpcdma_xfer dma_xfer = {
        .direction = GPCDMA_XFER_DIR_MEM_TO_IO,
        .bus_width = GPCDMA_IO_BUS_WIDTH_32,
        .burst_size = burst_size,
        .src_addr = (uint64_t)(fsp_c_u8_ptr_to_u32(tx_buf)),
        .src_addr_wrap = 0,
        .dst_addr = (uint64_t)(ctlr->conf.base_addr + SPI_TX_FIFO_0),
        .dst_addr_wrap = 1,
        .xfer_count = len,
        .en_flow_ctrl = true,
        .slave_req = ctlr->dma_slave_req,
        .synchronous = true,
        .timeout = 10,
        .callback = NULL,
        .callback_param = NULL,
    };

    return gpcdma_transfer(ctlr->dma_id, ctlr->dma_chans.tx, &dma_xfer);
}

error_t spi_port_clk_reset_pulse(struct spi_ctlr *ctlr,
                                 uint32_t delay)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = tegra_clk_reset_pulse(ctlr->conf.hw_handle->rst, delay);
    if (ret != 0) {
        ret = E_SPI_CLK_RST;
    }

out:
    return ret;
}

error_t spi_port_clk_set_rate(struct spi_ctlr *ctlr,
                              uint32_t rate)
{
    error_t ret = E_SUCCESS;

    if (ctlr == NULL) {
        ret = E_SPI_NULL_PTR;
        goto out;
    }

    ret = tegra_clk_set_rate(ctlr->conf.hw_handle->clk, rate);
    if (ret != 0) {
        ret = E_SPI_CLK_SET_RATE;
    }

out:
    return ret;
}
