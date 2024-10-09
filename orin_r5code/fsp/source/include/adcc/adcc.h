/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef ADCC__ADCC_H
#define ADCC__ADCC_H
#define FSP__ADCC__ADCC_H       1

#define ADCC_MODE_SINGLE_SHOT   1U
#define ADCC_MODE_CONT_MODE     0U

#define ADCC_CLK_SRC_OSC_UNDIV  0U
#define ADCC_CLK_SRC_PLLP       1U

/**
 * @file adcc/adcc-tegra-priv.h
 * @brief Structures that are internal to the ADCC driver
 */

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>

/**
 * ADCC controllers support 6 channels. Hence,
 * setting number of ADCC channels as 6.
 */
#define ADCC_NCHANS 6U

/*
 * Declaration for adcc_id that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
struct adcc_id;

/**
 * @brief ADCC channels samples
 *
 * Structure that is used by the ADCC driver to fetch the samples from
 * all the ADCC channels.
 *
 * @ch_data          array containing all the channels samples
 * @enabled_chans    bit mask of the enabled channels
 */
struct adcc_data {
    uint32_t  ch_data[ADCC_NCHANS];
    uint32_t  enabled_chans;
};

/**
 * @brief ADCC configuration
 *
 * Structure that stores the configuration details that is used by the
 * ADCC driver.
 *
 * @enabled_chans    bit mask of the enabled channels
 * @sampling_dur     sampling duration
 * @avg_window       averaging window duration
 * @mode             single shot or continuous mode
 * @clk_src          adcc clock source
 * @chans_lb_data    adcc channels loop back data
 */
struct adcc_conf {
    uint32_t  enabled_chans;
    uint32_t  sampling_dur;
    uint32_t  avg_window;
    uint32_t  mode;
    uint32_t  clk_src;
    uint64_t  chans_lb_data;
};

/**
 * @brief Read all ADCC channels samples
 *
 * @jama_func_req_id xxxxxxx
 *
 * This function fetches the samples read on all ADCC channel.
 *
 * @pre the function adcc_init() has been called
 *
 * @param[in] id          pointer to the ADCC descriptor instance.
 * @param[in] data        pointer to store the channels samples read.
 *
 * @retval E_SUCCESS        indicates success
 * @retval E_ADCC_NULL_PTR  invalid id paramter passed
 * @retval E_ADCC_NO_INIT   adcc is not initialized
 */
error_t adcc_read_data(const struct adcc_id *id,
                       struct adcc_data *data);

/**
 * @brief Read an ADCC channel's sample
 *
 * @jama_func_req_id xxxxxxx
 *
 * This function fetches the sample read on an ADCC channel specified.
 *
 * @pre the function adcc_init() has been called
 *
 * @param[in] id          pointer to the ADCC descriptor instance.
 * @param[in] chan        channel index in [0, ADCC_NCHANS - 1]
 * @param[in] data        pointer to store the read channel sample
 *
 * @retval E_SUCCESS                indicates success
 * @retval E_ADCC_NULL_PTR          invalid id paramter passed
 * @retval E_ADCC_NO_INIT           adcc is not initialized
 * @retval E_ADCC_INVALID_CHAN      invalid chan id passed
 * @retval E_ADCC_CHAN_NOT_ENABLED  invalid chan id passed
 */
error_t adcc_read_chan_data(const struct adcc_id *id,
                            uint32_t chan,
                            uint32_t *data);

/**
 * @brief Stop the ADCC channel scan
 *
 * @jama_func_req_id xxxxxxx
 *
 * This function disables the ADCC channel scanning.
 *
 * @pre the function adcc_init() has been called
 *
 * @param[in] id          pointer to the ADCC descriptor instance.
 *
 * @retval E_SUCCESS        indicates success
 * @retval E_ADCC_NULL_PTR  invalid id paramter passed
 * @retval E_ADCC_NO_INIT   adcc is not initialized
 */
error_t adcc_stop_scan(const struct adcc_id *id);

/**
 * @brief Start the ADCC channel scan
 *
 * @jama_func_req_id xxxxxxx
 *
 * This function enables the ADCC channel scanning.
 *
 * @pre the function adcc_init() has been called
 *
 * @param[in] id          pointer to the ADCC descriptor instance.
 *
 * @retval E_SUCCESS        indicates success
 * @retval E_ADCC_NULL_PTR  invalid id paramter passed
 * @retval E_ADCC_NO_INIT   adcc is not initialized
 */
error_t adcc_start_scan(const struct adcc_id *id);

/**
 * @brief Initialize the ADCC context
 *
 * @jama_func_req_id xxxxxxx
 *
 * This function initializes the ADCC context for an instance.
 *
 * @param[in] id          pointer to the ADCC descriptor instance.
 * @param[conf]           adcc configuration.
 *
 * @retval E_SUCCESS        indicates success
 * @retval E_ADCC_NULL_PTR  invalid paramter passed
 */
error_t adcc_init(const struct adcc_id *id,
                  const struct adcc_conf *conf);

#endif
