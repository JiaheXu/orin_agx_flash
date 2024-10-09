/*
 * Copyright (c) 2015-2021 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef AODMIC__TEGRA_AODMIC_H
#define AODMIC__TEGRA_AODMIC_H
#define FSP__AODMIC__TEGRA_AODMIC_H                     1

/* Compiler headers */
#include <stdint.h>
                                        // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for MODULE_ID_AODMIC, E_SUCCESS
#include <misc/macros.h>                // for END_RFD_BLOCK, START_RFD_BLOCK

/* Module-specific FSP headers */

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
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")


// IWYU pragma: no_forward_declare tegra_aodmic_ctlr
struct tegra_aodmic_ctlr;

/* Supported sample rates */
enum tegra_aodmic_sample_rate {
    TEGRA_AODMIC_RATE_8KHZ  =  8000,
    TEGRA_AODMIC_RATE_16KHZ = 16000,
    TEGRA_AODMIC_RATE_44KHZ = 44100,
    TEGRA_AODMIC_RATE_48KHZ = 48000,
    TEGRA_AODMIC_NUM_RATES
};

/* Supported channel configurations */
enum tegra_aodmic_channel_config {
    TEGRA_AODMIC_CHANNEL_MONO_LEFT,
    TEGRA_AODMIC_CHANNEL_MONO_RIGHT,
    TEGRA_AODMIC_CHANNEL_STEREO,
    TEGRA_AODMIC_NUM_CHANNEL_CONFIGS
};

/*
 * Supported sample widths
 *  AODMIC provides samples at 24 bit sample width,
 *  which will be scaled as per below configuration
 */
enum tegra_aodmic_sample_width {
    TEGRA_AODMIC_BITS_PER_SAMPLE_16 = 2,
    TEGRA_AODMIC_BITS_PER_SAMPLE_32 = 4,
};

/*
 * This relates to the Queue size in the driver. It is statically
 * allocated for the largest number supported, so this number
 * should only be raised in rare cases where quadruple data buffering
 * is deemed insufficient.  Double buffering is the typical usage.
 */
#define AODMIC_MAX_NUM_PERIODS (4)

/*
 * @brief AODMIC config structure
 * PCM configuration:
 * @sample_rate         As per tegra_aodmic_sample_rate
 * @channel_config      As per tegra_aodmic_channel_config
 * @sample_width        As per tegra_aodmic_sample_width
 * DMA configuration:
 * @period_size         DMA interrupt interval (in sample frames),
 *                      must be multiple of 16
 * @num_periods         2 <= num_periods <= AODMIC_MAX_NUM_PERIODS
 * DMA instance:
 * @gpcdma_id           GPCDMA instance handle
 * @dma_chan_num        GPCDMA channel no. to use (0 - 7)
 *
 * GPCDMA Data Buffers
 * @gpcdma_buf          Pointer to data buffer that will be
 *                      used exclusively by the driver, i.e. application should
 *                      create but never directly access this buffer.
 * @gpcdma_buf_size     This is the size of the buffer created. It must be at
 *                      least period_size*num_periods*4*num_channels where
 *                      num_channels=1 for mono channel configurations and
 *                      num_channels=2 for stereo channel configutations. If
 *                      this condition is not met the driver will return an
 *                      error code of E_AODMIC_MEM_ALLOC_FAILURE.
 */
struct tegra_aodmic_config {
    /* PCM configuration */
    enum tegra_aodmic_sample_rate    sample_rate;
    enum tegra_aodmic_channel_config channel_config;
    enum tegra_aodmic_sample_width   sample_width;

    /* DMA configuration */
    uint32_t                         period_size;
    uint32_t                         num_periods;

    /* DMA instance */
    struct gpcdma_id                *dma_id;
    uint32_t                         dma_chan_num;

    /* GPCDMA Data Buffers */
    uint8_t                         *gpcdma_buf;
    uint32_t                         gpcdma_buf_size;
};

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
error_t
tegra_aodmic_open(struct tegra_aodmic_ctlr *ctlr,
                  const struct tegra_aodmic_config *config);

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
error_t
tegra_aodmic_read(struct tegra_aodmic_ctlr *ctlr,
                  void *data,
                  uint32_t count);

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
error_t
tegra_aodmic_close(struct tegra_aodmic_ctlr *ctlr);

#endif
