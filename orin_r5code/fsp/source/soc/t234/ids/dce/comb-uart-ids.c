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
#include <stdint.h>                       // for UINT32_MAX, UINT8_MAX

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT
#include <soc-common/hw-const.h>          /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>              // for NV_ADDRESS_MAP_AON_HSP_SM_BASE

/* Late FSP headers */
#include <comb-uart/comb-uart-priv.h>     // for comb_uart_id, FSP__HSP__HSP...
#include <comb-uart/sections-comb-uart.h> // Immune from CT_ASSERT protection

SECTION_COMB_UART_DATA
const struct comb_uart_id comb_uart_id_dce = {
    .tx_sm_reg   = HSP_SM_BASE(NV_ADDRESS_MAP_AON_HSP_SM_BASE, 3),
    .rx_sm_reg   = 0UL,
    .supports_rx = false,
};
