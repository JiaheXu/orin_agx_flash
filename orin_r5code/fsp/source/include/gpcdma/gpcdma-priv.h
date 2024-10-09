/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef GPCDMA__GPCDMA_PRIV_H
#define GPCDMA__GPCDMA_PRIV_H
#define FSP__GPCDMA__GPCDMA_PRIV_H                      1

/**
 * @file gpcdma/gpcdma-tegra-priv.h
 * @brief Structures that are internal to the GPCDMA driver
 */

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <gpcdma/gpcdma.h>
#include <lock/smplock.h>
#include <lock/smplock-up.h>        // for smplock_t

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
CT_ASSERT(FSP__GPCDMA__GPCDMA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOCK__SMPLOCK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOCK__SMPLOCK_UP_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * Most of the AUXP GPCDMA controllers support 8 channels. Hence,
 * setting number of DMA channels as 8.
 */
#define TEGRA_GPCDMA_NUM_CHANNELS 8

struct gpcdma_hw_handle;

/**
 * @brief GPCDMA channel context
 *
 * Structure that describes the configuration of a GPCDMA controller.
 *
 * @chan_base       base address of the channel
 * @id              channel identifier ranging from 0 to 7
 * @irq             channel irq
 * @total_bytes	    total bytes transferred so far in this channel
 * @words_to_xfer   words to transfer
 * @lock            per channel smplock for SMP systems
 * @callback        callback to be executed for asynchronous transfers
 * @cdata           opaque data pointer for the callback function
 * @synchronous     indicates if the channel xfers are synchronous
 * @continuous	    indicates if channel xfers are continuous
 * @busy            indicates if the channel is busy
 */
struct gpcdma_channel {
    uint32_t        chan_base;
    uint32_t        id;
    uint32_t        irq;
    uint64_t        total_bytes;
    uint32_t        words_to_xfer;
    smplock_t       *lock;
    dma_callback    *callback;
    void            *cdata;
    bool            synchronous;
    bool            continuous;
    bool            busy;
};

/**
 * @brief GPCDMA controller configuration structure
 *
 * Structure that describes the configuration of a GPCDMA controller.
 *
 * @base_addr   base address of the controller
 * @irqs        array of per channel irqs
 * @hw_handle   port specific hw handle
 */
struct gpcdma_conf {
    uint32_t                        base_addr;
    const uint32_t                  irqs[TEGRA_GPCDMA_NUM_CHANNELS];
    const struct gpcdma_hw_handle   *hw_handle;
};

/**
 * @brief GPCDMA controller context structure
 *
 * Structure that is used by the GPCDMA driver to manage the GPCDMA controller
 * during run time.
 *
 * @conf        GPCDMA controller instance configuration
 * @channels    array of pointers to GPCDMA controller per-channel context
 * @inited      GPCDMA controller initialization state
 */
struct gpcdma_id {
    const struct gpcdma_conf    conf;
    struct gpcdma_channel       *channels[TEGRA_GPCDMA_NUM_CHANNELS];
    bool                        inited;
};

/**
 * @brief GPCDMA source and destination buffers used in cyclic mode
 *
 * Structure that is used by the driver during cyclic mode. It will store
 * buffer descriptors equal to period length specified during transfers.
 *
 * @src_addr    stores the source address
 * @dst_addr    stores the destination address
 */
struct gpcdma_buf_desc {
    uint32_t src_addr;
    uint32_t dst_addr;
    uint32_t hi_src_addr;
    uint32_t hi_dst_addr;
};

#endif
