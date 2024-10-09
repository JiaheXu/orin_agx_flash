/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>                   // for uint32_t, uint8_t, UINT32_...
#include <stddef.h>                   // for NULL
#include <stdbool.h>                  // for bool
                                      // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>           // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>            // for rtosTaskYieldFromISR
#include <osa/rtos-semaphore.h>       // for rtosSemaphore...
#include <osa/rtos-queue.h>	          // for rtosQueue
#include <osa/rtos-values.h>          // for rtosBool
#include <misc/attributes.h>          // for ALIGN
#include <misc/macros.h>              // for ARRAY_SIZE
#include <error/common-errors.h>      // for error_t, E_SUCCESS
#include <clk/clk-tegra.h>            // for tegra_clk_reset_pulse

/* Module specific headers */
#include <gpcdma/gpcdma-errors.h>     // for E_GPCDMA_NULL_PTR, E_GPCDMA_...
#include <gpcdma/gpcdma-priv.h>       // for struct gpcdma_id, gpcdma_channel...
#include <gpcdma/gpcdma-port.h>       // for gpcdma_port_...
#include <port/gpcdma-port-priv.h>    // Immune from CT_ASSERT protection

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/macros.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_SEMAPHORE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_QUEUE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_PORT_H, "Header file missing or invalid.")

static rtosSemaphoreHandle  chan_sems[TEGRA_GPCDMA_NUM_CHANNELS];
static uint8_t              chan_sem_bufs[TEGRA_GPCDMA_NUM_CHANNELS][rtosSemaphoreSize()];

/* Queue is only used during GPCDMA cyclic mode */
#define GPCDMA_NUM_EL		     64UL /* Means 64 buf descriptors */
#define GPCDMA_QUEUE_SIZE        rtosQueueSize(GPCDMA_NUM_EL, \
                                               sizeof(struct gpcdma_buf_desc))
static rtosQueueHandle chan_que[TEGRA_GPCDMA_NUM_CHANNELS];
static uint8_t         chan_que_bufs[TEGRA_GPCDMA_NUM_CHANNELS][GPCDMA_QUEUE_SIZE];
/* Adjust GPCDMA_NUM_EL based on application requirement */

static inline bool is_dma_chan_valid(uint32_t chan_num)
{
    return chan_num < (uint32_t)TEGRA_GPCDMA_NUM_CHANNELS;
}

/* Queue implementation for the continuous or cyclic mode of tranfer */

error_t gpcdma_port_init_queue(uint32_t chan_num, uint32_t nelems,
                               uint32_t el_sz)
{
    rtosError status;

    if (!is_dma_chan_valid(chan_num)) {
        return E_GPCDMA_INVALID_PARAM;
    }

    if (nelems > GPCDMA_NUM_EL) {
        return E_GPCDMA_INVALID_PARAM;
    }

    status = rtosQueueCreate(chan_que_bufs[chan_num],
                     ARRAY_SIZE(chan_que_bufs[chan_num]), nelems, el_sz,
                     &chan_que[chan_num]);
    if (status != rtosPASS) {
        return E_GPCDMA_QUEUE_OP_FAIL;
    }

    return E_SUCCESS;
}

void gpcdma_port_delete_queue(uint32_t chan_num)
{
    if (is_dma_chan_valid(chan_num)) {
        rtosQueueDelete(chan_que[chan_num]);
    }
}

error_t gpcdma_port_get_desc(uint32_t chan_num, void *ptr, uint32_t timeout,
                             bool from_isr)
{
    rtosBool higher_prio_task_woken = rtosFALSE;
    error_t ret = E_SUCCESS;

    if (ptr == NULL) {
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }
    if (!is_dma_chan_valid(chan_num)) {
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    if (from_isr) {
        if (rtosQueueReceiveFromISR(chan_que[chan_num], ptr,
            &higher_prio_task_woken) != rtosPASS) {
            ret = E_GPCDMA_QUEUE_OP_FAIL;
        } else {
            rtosTaskYieldFromISR(higher_prio_task_woken);
        }
    } else {
        if (rtosQueueReceive(chan_que[chan_num], ptr,
            (rtosTick)timeout) != rtosPASS) {
            ret = E_GPCDMA_QUEUE_OP_FAIL;
        }
    }

out:
    return ret;
}

error_t gpcdma_port_send_desc_to_back(uint32_t chan_num, void *ptr,
                                      uint32_t timeout, bool from_isr)
{
    rtosBool higher_prio_task_woken = rtosFALSE;
    error_t ret = E_SUCCESS;

    if (ptr == NULL) {
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }
    if (!is_dma_chan_valid(chan_num)) {
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    if (from_isr) {
        if (rtosQueueSendToBackFromISR(chan_que[chan_num], ptr,
            &higher_prio_task_woken) != rtosPASS) {
            ret = E_GPCDMA_QUEUE_OP_FAIL;
        } else {
            rtosTaskYieldFromISR(higher_prio_task_woken);
        }
    } else {
        if (rtosQueueSendToBack(chan_que[chan_num], ptr,
            (rtosTick)timeout) != rtosPASS) {
            ret = E_GPCDMA_QUEUE_OP_FAIL;
        }
    }

out:
    return ret;
}
/* End of queue implementation */

error_t gpcdma_port_chan_sync(uint32_t chan_num,
                              uint32_t timeout)
{
    error_t ret;

    if (!is_dma_chan_valid(chan_num)) {
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    ret = rtosSemaphoreAcquire(chan_sems[chan_num], (rtosTick)timeout);
    if (ret != rtosPASS) {
        ret = E_GPCDMA_PORT_SYNC_TIMEOUT;
        goto out;
    }
    ret = E_SUCCESS;

out:
    return ret;
}

error_t gpcdma_port_chan_sync_end(uint32_t chan_num)
{
    error_t ret;
    rtosBool higher_prio_task_woken = rtosFALSE;

    if (!is_dma_chan_valid(chan_num)) {
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    ret = rtosSemaphoreReleaseFromISR(chan_sems[chan_num],
                                      &higher_prio_task_woken);
    if (ret == rtosPASS) {
        rtosTaskYieldFromISR(higher_prio_task_woken);
        ret = E_SUCCESS;
    }

out:
    return ret;
}

error_t gpcdma_port_chan_setup(uint32_t chan_num)
{
    error_t ret;
    rtosUCount depth;

    if (!is_dma_chan_valid(chan_num)) {
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    ret = rtosSemaphoreCreateBinary(chan_sem_bufs[chan_num],
                                    &chan_sems[chan_num]);
    if (ret != rtosPASS) {
        ret = E_GPCDMA_PORT_CHAN_SETUP_FAIL;
        goto out;
    }

    ret = rtosSemaphoreGetCountDepth(chan_sems[chan_num], &depth);
    if (ret != rtosPASS) {
        ret = E_GPCDMA_PORT_CHAN_SETUP_FAIL;
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
    ret = rtosSemaphoreAcquire(chan_sems[chan_num], (rtosTick)0);
    if (ret != rtosPASS) {
        ret = E_GPCDMA_PORT_CHAN_SETUP_FAIL;
        goto out;
    }
    ret = E_SUCCESS;

out:
    return ret;
}

error_t gpcdma_port_init(struct gpcdma_id *id)
{
    error_t ret = E_SUCCESS;

    if (id == NULL) {
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }

    ret = tegra_clk_reset_pulse(id->conf.hw_handle->rst, 2);
    if (ret != 0) {
        ret = E_GPCDMA_PORT_INIT_FAIL;
    }

out:
    return ret;
}
