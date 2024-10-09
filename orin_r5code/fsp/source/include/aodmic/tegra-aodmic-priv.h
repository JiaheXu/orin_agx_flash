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

#ifndef AODMIC__TEGRA_AODMIC_PRIV_H_
#define AODMIC__TEGRA_AODMIC_PRIV_H_
#define FSP__AODMIC__TEGRA_AODMIC_PRIV_H                1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-queue.h>
#include <gpcdma/gpcdma.h>
#include <misc/macros.h>                // for END_RFD_BLOCK, START_RFD_BLOCK

/* Module-specific FSP headers */
#include <aodmic/tegra-aodmic.h>        // for AODMIC_MAX_NUM_PERIODS

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSA__RTOS_QUEUE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/* Internal hardcodings of AODMIC */
#define TEGRA_AODMIC_WORD_SIZE        4u   /* 4 bytes  */
#define TEGRA_AODMIC_FIFO_SIZE        32u  /* 32 words */

/* AODMIC interrupt threshold */
#define TEGRA_AODMIC_FIFO_THRESHOLD   (TEGRA_AODMIC_FIFO_SIZE / 2u)

#define TEGRA_AODMIC_MIN_ALLOWED_PERIODS (2UL)

struct tegra_aodmic_id {
    uint32_t base_addr;
    const struct aodmic_port_clk_rst *aodmic_hw_handle;
    uint32_t irq;
};

struct tegra_aodmic_ctlr {
    const struct tegra_aodmic_id id;

    /* PCM configuration */
    uint32_t sample_rate;  /* In Hz */
    uint32_t num_channels; /* Channel count */
    uint32_t samp_bytes;   /* Bytes per sample */

    /* GPCDMA configuration */
    struct gpcdma_id *dma_id;   /* DMA instance ID */
    uint32_t dma_chan_num; /* DMA channel no. */
    struct gpcdma_xfer dma_xfer;     /* DMA transfer parameters */

    /* Driver data buffer */
    uint8_t *data;         /* Data pointer (of num_periods length) */
    uint32_t period_bytes;  /* Size of each period in data buffer */
    uint32_t num_periods;   /* No. of periods */

    /* Buffer control */
    rtosSemaphoreBuffer rtosSemaphore_object; /* statically allocated object */
    rtosSemaphoreHandle rtosSemaphore_free_buf; /* Count of free periods */
    uint8_t rtosQueue_object_mem[            /* queue object and queue data */
                rtosQueueSize(AODMIC_MAX_NUM_PERIODS, sizeof(uint64_t))];
    rtosQueueHandle rtosQueue_filled_buf;   /* Queue of filled periods */
    rtosTick max_read_timeout;    /* Timeout at queue receive */
    uint32_t free_pos;      /* Next available free period (for writing) */
    uint8_t *read_ptr;     /* Location of period to read */
    uint32_t bytes_to_read; /* Bytes available to read */

    /* Control information */
    bool     init_done;
    bool     dma_running;
    bool     reset;
    uint32_t xrun_cnt;

    /* Error Logging */
    uint32_t overflow_count;
};

#endif
