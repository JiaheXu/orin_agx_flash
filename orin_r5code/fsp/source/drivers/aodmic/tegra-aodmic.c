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

/* Compiler headers */
#include <stdbool.h>                    // for true, false
#include <stddef.h>                     // for NULL
#include <stdint.h>                     // for uint32_t, uint8_t, int32_t
                                        // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <misc/nvrm_drf.h>              // for NV_FLD_SET_DRF_DEF, NV_DRF_DEF

/* Hardware headers */
#include <araodmic.h>

/* Late FSP headers */
#include <error/common-errors.h>        // for MODULE_ID_AODMIC, E_SUCCESS
#include <osa/rtos-values.h>            // for rtosPASS, rtosFALSE
#include <osa/rtos-queue.h>             // for rtosQueueReceive, rtosQueueCr...
#include <osa/rtos-semaphore.h>         // for rtosSemaphoreRelease, rtosSem...
#include <osa/rtos-task.h>              // for rtosTaskYieldFromISR
#include <cpu/type-conversion.h>        // for fsp_c_u32_ptr_to_u64
#include <gpcdma/gpcdma.h>              // for gpcdma_xfer, gpcdma_abort
#include <misc/macros.h>                // for END_RFD_BLOCK, START_RFD_BLOCK
#include <reg-access/reg-access.h>      // for readl, writel

/* Module-specific FSP headers */
#include <aodmic/aodmic-errors.h>       // for E_AODMIC_NULL_POINTER, E_AODM...
#include <aodmic/sections-aodmic.h>     // for SECTION_AODMIC_...
#include <aodmic/tegra-aodmic-priv.h>   // for tegra_aodmic_ctlr, tegra_aodm...
#include <aodmic/tegra-aodmic.h>        // for tegra_aodmic_config, TEGRA_AO...
#include <port/aodmic-port.h>           // for aodmic_port_clock_disable
#include <processor/tegra-aodmic-hw.h>  // for AODMIC_GPCDMA_REQ_SEL

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_QUEUE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_SEMAPHORE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__AODMIC__AODMIC_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__AODMIC__TEGRA_AODMIC_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__AODMIC__TEGRA_AODMIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PORT__AODMIC_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__TEGRA_AODMIC_HW_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")

SECTION_AODMIC_TEXT static inline uint32_t
aodmic_readl(struct tegra_aodmic_ctlr *ctlr,
             uint32_t offset)
{
    return readl(ctlr->id.base_addr + offset);
}

SECTION_AODMIC_TEXT static inline void
aodmic_writel(struct tegra_aodmic_ctlr *ctlr,
              uint32_t value,
              uint32_t offset)
{
    writel(value, ctlr->id.base_addr + offset);
}

START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")

/* Callback from GPCDMA upon completion of transfer */
SECTION_AODMIC_TEXT static void
tegra_aodmic_gpcdma_callback(void *callback_param,
                             dma_status status)
{
    struct tegra_aodmic_ctlr *ctlr;
    rtosError free_buf_avail;
    rtosBool higher_prio_task_woken = rtosFALSE;
    uint64_t filled_buf;

    ctlr = (struct tegra_aodmic_ctlr *)callback_param;

    /* Address of filled buffer */
    filled_buf = ctlr->dma_xfer.dst_addr;

    /* Next write position */
    ctlr->free_pos = (ctlr->free_pos + 1u) % ctlr->num_periods;
    ctlr->dma_xfer.dst_addr = fsp_c_u32_ptr_to_u64((uint32_t *)(ctlr->data +
                                (ctlr->free_pos * ctlr->period_bytes)));

    /* If reset is already flagged, then wait for it to clear */
    if (ctlr->reset) {
        ctlr->xrun_cnt++;
        goto out;
    }

    /* Acquire next free buffer */
    free_buf_avail = rtosSemaphoreAcquireFromISR(ctlr->rtosSemaphore_free_buf,
                        &higher_prio_task_woken);

    /* No free buffer indicates overflow */
    if (free_buf_avail != rtosPASS) {
        ctlr->reset = true; /* flag for buffer reset */
        ctlr->overflow_count++;  /* log overflow */
    }

    /* Post available buffer for consumption */
    rtosQueueSendToBackFromISR(ctlr->rtosQueue_filled_buf, &filled_buf,
                                &higher_prio_task_woken);

    rtosTaskYieldFromISR(higher_prio_task_woken);

out:
    return;
}

/**
 * @brief open and initialize AODMIC instance
 *
 * @pre gpcdma_init() has been called
 *
 * @param[in] ctlr:        AODMIC controller instance
 * @param[in] config:      Configuration structure
 *
 * @retval E_SUCCESS                     indicates success
 * @retval E_AODMIC_NULL_POINTER         indicates NULL pointer was passed
 * @retval E_AODMIC_INVALID_SAMPLE_RATE  invalid sample rate
 * @retval E_AODMIC_INVALID_CHANNELS     invalid channel configuration
 * @retval E_AODMIC_INVALID_SAMPLE_WIDTH invalid sample width
 * @retval E_AODMIC_INVALID_PERIOD       invalid period
 * @retval E_AODMIC_MEM_ALLOC_FAILURE    gpcdma_buf_size too small
 * @retval E_AODMIC_SEM_CREATE_FAILURE   failed rtos object creation
 */
SECTION_AODMIC_INIT_TEXT error_t
tegra_aodmic_open(struct tegra_aodmic_ctlr *ctlr,
                  const struct tegra_aodmic_config *config)
{
    uint32_t osr, num_channels, period_bytes;
    uint32_t ctrl_reg, fifo_ctrl_reg, dbg_ctrl_reg, enable_reg;
    error_t ret_val = E_SUCCESS;

    if ((config == NULL) || (ctlr == NULL)) {
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    /* Initialize memory pointers in ctlr to NULL */
    ctlr->data                   = NULL;
    ctlr->rtosSemaphore_free_buf = NULL;
    ctlr->rtosQueue_filled_buf   = NULL;

    if (config->gpcdma_buf == NULL) {
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    if (config->num_periods < 2UL) {
        ret_val = E_AODMIC_CONFIGURATION_NA;
        goto out;
    }

    /* Setup clock as per sample rate configuration */
    /* Clock = OSR * sample_rate                    */
    switch (config->sample_rate) {
    case TEGRA_AODMIC_RATE_8KHZ:
    case TEGRA_AODMIC_RATE_16KHZ:
    case TEGRA_AODMIC_RATE_44KHZ:
    case TEGRA_AODMIC_RATE_48KHZ:
        break;
    default:
        ret_val = E_AODMIC_INVALID_SAMPLE_RATE;
        goto out;
    }
    ctlr->sample_rate = (uint32_t)config->sample_rate;
    /* OSR is tied to OSR64 for AODMIC */
    osr = 1u << (AODMIC_CTRL_0_OSR_OSR64 + 6u);
    ret_val = aodmic_port_clock_set_rate(ctlr->id.aodmic_hw_handle,
                                        (osr * ctlr->sample_rate));
    if (ret_val != E_SUCCESS) {
        goto out;
    }

    ret_val = aodmic_port_clock_reset_config(ctlr->id.aodmic_hw_handle);
    if (ret_val != E_SUCCESS) {
        goto out;
    }

    /* Read control register */
    ctrl_reg = aodmic_readl(ctlr, AODMIC_CTRL_0);

    /* Setup channel configuration in control register */
    switch (config->channel_config) {
    case TEGRA_AODMIC_CHANNEL_MONO_LEFT:
        num_channels = 1;
        ctrl_reg =
            NV_FLD_SET_DRF_DEF(AODMIC, CTRL,
                    CHANNEL_SELECT, LEFT, ctrl_reg);
        break;
    case TEGRA_AODMIC_CHANNEL_MONO_RIGHT:
        num_channels = 1;
        ctrl_reg =
            NV_FLD_SET_DRF_DEF(AODMIC, CTRL,
                    CHANNEL_SELECT, RIGHT, ctrl_reg);
        break;
    case TEGRA_AODMIC_CHANNEL_STEREO:
        num_channels = 2;
        ctrl_reg =
            NV_FLD_SET_DRF_DEF(AODMIC, CTRL,
                    CHANNEL_SELECT, STEREO, ctrl_reg);
        break;

    default:
        ret_val = E_AODMIC_INVALID_CHANNELS;
        goto out;
    }
    ctlr->num_channels = num_channels;
    aodmic_writel(ctlr, ctrl_reg, AODMIC_CTRL_0);

    /* Read sample width configuration */
    if ((config->sample_width != TEGRA_AODMIC_BITS_PER_SAMPLE_16) &&
        (config->sample_width != TEGRA_AODMIC_BITS_PER_SAMPLE_32)) {
            ret_val = E_AODMIC_INVALID_SAMPLE_WIDTH;
            goto out;
    }
    ctlr->samp_bytes = (uint32_t)config->sample_width;

    /* Setup interrupt threshold in FIFO control register */
    fifo_ctrl_reg = NV_DRF_NUM(AODMIC, APB_FIFO_CTRL, THRESHOLD,
                    TEGRA_AODMIC_FIFO_THRESHOLD);
    aodmic_writel(ctlr, fifo_ctrl_reg, AODMIC_APB_FIFO_CTRL_0);

    /* SC filter is enabled by default, enable DCR filter here */
    dbg_ctrl_reg = aodmic_readl(ctlr, AODMIC_DBG_CTRL_0);
    dbg_ctrl_reg = NV_FLD_SET_DRF_DEF(AODMIC, DBG_CTRL,
                        DCR_ENABLE, ENABLE, dbg_ctrl_reg);
    aodmic_writel(ctlr, dbg_ctrl_reg, AODMIC_DBG_CTRL_0);

    /* Allocate driver buffer */
    if (((config->period_size % TEGRA_AODMIC_FIFO_THRESHOLD) != 0u) ||
         (config->num_periods < TEGRA_AODMIC_MIN_ALLOWED_PERIODS)   ||
         (config->num_periods > AODMIC_MAX_NUM_PERIODS)) {
            ret_val = E_AODMIC_INVALID_PERIOD;
            goto out;
    }
    /* Driver buffer may be viewed as a queue of    */
    /* num_periods buffers each of size period_size */
    period_bytes = config->period_size *
                    num_channels * TEGRA_AODMIC_WORD_SIZE;
    if (config->gpcdma_buf_size <  (period_bytes * config->num_periods)) {
        ret_val = E_AODMIC_MEM_ALLOC_FAILURE;
        goto out;
    }
    ctlr->data = config->gpcdma_buf;
    ctlr->period_bytes = period_bytes;
    ctlr->num_periods  = config->num_periods;

    /* Create semaphore for tracking free buffers */
     rtosSemaphoreCreateCounting(&ctlr->rtosSemaphore_object,
                ctlr->num_periods /* uxMaxCount */,
                0             /* uxInitialCount */,
                (void **)&ctlr->rtosSemaphore_free_buf);
    if (ctlr->rtosSemaphore_free_buf == NULL) {
        ret_val = E_AODMIC_SEM_CREATE_FAILURE;
        goto out;
    }

    /* Create queue for tracking filled buffers */
    rtosQueueCreate(ctlr->rtosQueue_object_mem,
                    rtosQueueSize(AODMIC_MAX_NUM_PERIODS, sizeof(uint64_t)),
                    ctlr->num_periods /* uxQueueLength */,
                    sizeof(uint64_t)    /* uxItemSize */,
                    (void **)&ctlr->rtosQueue_filled_buf);
    if (ctlr->rtosQueue_filled_buf == NULL) {
        ret_val = E_AODMIC_SEM_CREATE_FAILURE;
        goto out;
    }

    /* Set timeout at queue receive to total buffer duration */
    ctlr->max_read_timeout =
        ((config->period_size * ctlr->num_periods * 1000u) / ctlr->sample_rate) /
        rtosTICK_RATE_MS;

    /* GPCDMA contoller ID and channel for transfer from AODMIC FIFO */
    ctlr->dma_id       = config->dma_id;
    ctlr->dma_chan_num = config->dma_chan_num;
    /*$$  IT IS ASSUMED THAT THE GPCDMA CONTROLLER AND THE SELECTED  $$*/
    /*$$  CHANNEL ARE INITIALIZED PRIOR TO CALLING THIS FUNCTION     $$*/

    /* Setup DMA transfer parameters */
    ctlr->dma_xfer.direction      = GPCDMA_XFER_DIR_IO_TO_MEM;
    ctlr->dma_xfer.bus_width      = GPCDMA_IO_BUS_WIDTH_32;
    ctlr->dma_xfer.burst_size     = TEGRA_AODMIC_FIFO_THRESHOLD;
    ctlr->dma_xfer.src_addr       = (uint64_t)ctlr->id.base_addr +
                                    AODMIC_APB_FIFO_CTRL_RD_DATA_0;
    ctlr->dma_xfer.src_addr_wrap  = 1;
    ctlr->dma_xfer.dst_addr       = 0; /* To be filled later */
    ctlr->dma_xfer.dst_addr_wrap  = 0;
    ctlr->dma_xfer.continuous     = true;
    ctlr->dma_xfer.period_len     = period_bytes;
    ctlr->dma_xfer.xfer_count     = (period_bytes * config->num_periods);
    ctlr->dma_xfer.en_flow_ctrl   = true;
    ctlr->dma_xfer.slave_req      = AODMIC_GPCDMA_REQ_SEL;
    ctlr->dma_xfer.synchronous    = false;
    ctlr->dma_xfer.timeout        = 0;
    ctlr->dma_xfer.callback       = tegra_aodmic_gpcdma_callback;
    ctlr->dma_xfer.callback_param = ctlr;

    /* Enable AODMIC */
    enable_reg = NV_DRF_DEF(AODMIC, ENABLE, ENABLE, TRUE);
    aodmic_writel(ctlr, enable_reg, AODMIC_ENABLE_0);

    /* Return success */
    ctlr->dma_running    = false;
    ctlr->init_done      = true;
    ctlr->reset          = true;
    ctlr->xrun_cnt       = 0;
    ctlr->overflow_count = 0;

out:
    if(ret_val != E_SUCCESS) {
        /* Release all resources and return error */
        tegra_aodmic_close(ctlr);
    }
    return ret_val;
}

/**
 * @brief Close AODMIC instance
 *
 * @pre none
 *
 * @param[in] ctlr:                 AODMIC controller instance
 *
 * @retval E_SUCCESS                indicates success
 * @retval E_AODMIC_NULL_POINTER    detected a null pointer
 */
SECTION_AODMIC_TEXT error_t
tegra_aodmic_close(struct tegra_aodmic_ctlr *ctlr)
{
    uint32_t reg_val;
    error_t ret_val = E_SUCCESS;

    if (ctlr == NULL) {
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    if (ctlr->init_done) {
        ctlr->init_done = false;

        /* Abort any DMA transfer in progress */
        gpcdma_abort((const struct gpcdma_id *)ctlr->dma_id, ctlr->dma_chan_num);

        /* Reset AODMIC */
        reg_val = NV_DRF_DEF(AODMIC, SOFT_RESET, SOFT_RESET, TRUE);
        aodmic_writel(ctlr, reg_val, AODMIC_SOFT_RESET_0);

        /* Disable AODMIC */
        reg_val = NV_DRF_DEF(AODMIC, ENABLE, ENABLE, FALSE);
        aodmic_writel(ctlr, reg_val, AODMIC_ENABLE_0);

        /* Disable AODMIC clock */
        ret_val = aodmic_port_clock_disable(ctlr->id.aodmic_hw_handle);
        if (ret_val != E_SUCCESS) {
            goto out;
        }
    }

    /* Release resources that may have been acquired */
    if (ctlr->rtosQueue_filled_buf != NULL) {
        rtosQueueDelete(ctlr->rtosQueue_filled_buf);
        ctlr->rtosQueue_filled_buf = NULL;
    }
    if (ctlr->rtosSemaphore_free_buf != NULL) {
        rtosSemaphoreDelete(ctlr->rtosSemaphore_free_buf);
        ctlr->rtosSemaphore_free_buf = NULL;
    }

out:
    return ret_val;
}

/**
 * @brief Read captured samples
 *
 * @pre the function tegra_aodmic_open() has been called
 *
 * @param[in] ctlr:        AODMIC controller instance
 * @param[in] count:       number of bytes to capture
 *
 * @retval E_SUCCESS                  success
 * @retval E_AODMIC_NULL_POINTER      detected a null pointer
 * @retval E_AODMIC_CONFIGURATION_NA  count of 0 not allowed
 * @retval E_AODMIC_NOT_INITIALIZED   tegra_aodmic_open() not called prior
 * @retval E_AODMIC_BUF_INVALID       null data pointer
 * @retval E_AODMIC_READ_TIMEOUT      timeout waiting for queue
 */
SECTION_AODMIC_INIT_TEXT error_t
tegra_aodmic_read(struct tegra_aodmic_ctlr *ctlr,
                  void *data,
                  uint32_t count)
{
    rtosError status;
    uint32_t i;
    uint8_t *write_buf;
    int32_t pcm_samp;
    error_t ret_val = E_SUCCESS;

    if (ctlr == NULL){
        ret_val = E_AODMIC_NULL_POINTER;
        goto out;
    }

    if (count == 0u) {
        ret_val = E_AODMIC_CONFIGURATION_NA;
        goto out;
    }

    if (!(ctlr->init_done)) {
        ret_val = E_AODMIC_NOT_INITIALIZED;
        goto out;
    }

    if (data == NULL) {
        ret_val = E_AODMIC_BUF_INVALID;
        goto out;
    }
    write_buf = data;

    /* Reset buffer queue, which may be at */
    /* init or due to buffer overflow      */
    if (ctlr->reset) {
        /* Drain out any buffers that may already be filled */
        do {
            status = rtosQueueReceive(ctlr->rtosQueue_filled_buf,
                        &ctlr->read_ptr,
                        0);
        } while (status == rtosPASS);
        ctlr->read_ptr      = NULL;
        ctlr->bytes_to_read = 0;

        /* Setup all buffers as free (for DMA to write) */
        do {
            status = rtosSemaphoreRelease(ctlr->rtosSemaphore_free_buf);
        } while (status == rtosPASS);

        ctlr->xrun_cnt = 0;
        ctlr->reset = false;
    }

    if (!(ctlr->dma_running)) {
        /* Trigger first DMA transfer */
        ctlr->dma_running = true;
        ctlr->free_pos = 0;
        rtosSemaphoreAcquire(ctlr->rtosSemaphore_free_buf, 0);
        ctlr->dma_xfer.dst_addr = fsp_c_u32_ptr_to_u64(
                                    (uint32_t *)ctlr->data);
        ret_val = gpcdma_transfer(
                    (const struct gpcdma_id *)ctlr->dma_id,
                    ctlr->dma_chan_num,
                    &ctlr->dma_xfer);
        if (ret_val != E_SUCCESS) {
            ctlr->dma_running = false;
            goto out;
        }
    }

    /* Read PCM data from filled buffers */
    for (i = 0; i < count; i += ctlr->samp_bytes) {
        /* If current buffer is exhausted, proceed to next buffer */
        if (ctlr->bytes_to_read == 0u) {
            /* Release consumed buffer for writing */
            if (ctlr->read_ptr != NULL) {
                rtosSemaphoreRelease(ctlr->rtosSemaphore_free_buf);
            }

            /* Get next buffer from filled queue */
            ctlr->read_ptr = NULL;
            status = rtosQueueReceive(ctlr->rtosQueue_filled_buf,
                        &ctlr->read_ptr,
                        ctlr->max_read_timeout);
            if (status != rtosPASS) {
                ret_val = E_AODMIC_READ_TIMEOUT;
                goto out;
            }
            ctlr->bytes_to_read = ctlr->period_bytes;
        }

        /* AODMIC provides 24-bit PCM packed in 32 bits,  */
        /* Little Endian, aligned to LSB (i.e. bits 23:0) */
        pcm_samp             = (*((int32_t *)(ctlr->read_ptr))) << 8;
        ctlr->read_ptr      += TEGRA_AODMIC_WORD_SIZE;
        ctlr->bytes_to_read -= TEGRA_AODMIC_WORD_SIZE;

        /* Format PCM sample to requested width */
        switch (ctlr->samp_bytes) {
        case 2:
            *((int16_t *)(write_buf + i)) = (int16_t)
                            (pcm_samp >> 16);
            break;
        case 4:
            *((int32_t *)(write_buf + i)) = pcm_samp;
            break;
        default:
            ret_val = E_AODMIC_INVALID_SAMPLE_RATE;
            break;
        }
    }

out:
    return ret_val;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx")
