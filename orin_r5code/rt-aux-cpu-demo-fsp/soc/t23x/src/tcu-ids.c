/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>

#include <osa/osa-task.h>
#include <osa/osa-errors.h>

#include <misc/attributes.h>
#include <processor/hsp-tegra-hw.h>

#include <mbox-aon.h>
#include <tcu.h>
#include <tcu-priv.h>

extern void tcu_tx_task(void *param);
extern void tcu_rx_task(void *param);

static tcu_client_t ext_clients[] = {
    [TCU_ID_CCPLEX] = {
        .id            = TCU_ID_CCPLEX,
        .tag           = TCU_TAG_CCPLEX,
        .hsp_id        = &tegra_hsp_id_top0,
        .recv_mbox     = MBOX_AON_CCPLEX_UART,
        .send_mbox     = 0,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
    [TCU_ID_BPMP] = {
        .id            = TCU_ID_BPMP,
        .tag           = TCU_TAG_BPMP,
        .hsp_id        = &tegra_hsp_id_bpmp,
        .recv_mbox     = MBOX_AON_BPMP_UART,
        .send_mbox     = MBOX_BPMP_UART,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
    [TCU_ID_SCE]  = {
        .id            = TCU_ID_SCE,
        .tag           = TCU_TAG_SCE,
        .recv_mbox     = MBOX_AON_SCE_UART,
        .send_mbox     = 0,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
    [TCU_ID_TZ]  = {
        .id            = TCU_ID_TZ,
        .tag           = TCU_TAG_TZ,
        .recv_mbox     = MBOX_AON_TZ_UART,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
    [TCU_ID_RCE]  = {
        .id            = TCU_ID_RCE,
        .tag           = TCU_TAG_RCE,
        .recv_mbox     = MBOX_AON_RCE_UART,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
    [TCU_ID_FSI]  = {
        .id            = TCU_ID_FSI,
        .tag           = TCU_TAG_FSI,
        .hsp_tx_id     = &tegra_hsp_id_top1,
        .hsp_id        = &tegra_hsp_id_fsi,
        .recv_mbox     = MBOX_TX_FSI_UART,
        .send_mbox     = MBOX_FSI_UART,
        .enabled       = true,
        .aon_sm_for_tx = false,
    },
    [TCU_ID_PSC]  = {
        .id            = TCU_ID_PSC,
        .tag           = TCU_TAG_PSC,
        .recv_mbox     = MBOX_AON_PSC_UART,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
    [TCU_ID_DCE]  = {
        .id            = TCU_ID_DCE,
        .tag           = TCU_TAG_DCE,
        .recv_mbox     = MBOX_AON_DCE_UART,
        .enabled       = true,
        .aon_sm_for_tx = true,
    },
};

static tcu_client_t *tag_to_ext_client[] = {
    [TCU_TAG_CCPLEX - TCU_TAG_MIN] = &ext_clients[TCU_ID_CCPLEX],
    [TCU_TAG_BPMP - TCU_TAG_MIN] = &ext_clients[TCU_ID_BPMP],
    [TCU_TAG_SCE - TCU_TAG_MIN] = &ext_clients[TCU_ID_SCE],
    [TCU_TAG_TZ - TCU_TAG_MIN] = &ext_clients[TCU_ID_TZ],
    [TCU_TAG_RCE - TCU_TAG_MIN] = &ext_clients[TCU_ID_RCE],
    [TCU_TAG_FSI - TCU_TAG_MIN] = &ext_clients[TCU_ID_FSI],
    [TCU_TAG_PSC - TCU_TAG_MIN] = &ext_clients[TCU_ID_PSC],
    [TCU_TAG_DCE - TCU_TAG_MIN] = &ext_clients[TCU_ID_DCE],
};

rtos_task_define_s(tcu_tx_task_ccplex,
                   tcu_tx_task,
                   (char *)"tcu_tx_ccplex",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_CCPLEX],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_rx_task_ccplex,
                   tcu_rx_task,
                   (char *)"tcu_rx_ccplex",
                   configMAX_PRIORITIES - 6,
                   &ext_clients[TCU_ID_CCPLEX],
                   configMINIMAL_STACK_SIZE);


rtos_task_define_s(tcu_tx_task_bpmp,
                   tcu_tx_task,
                   (char *)"tcu_tx_bpmp",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_BPMP],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_rx_task_bpmp,
                   tcu_rx_task,
                   (char *)"tcu_rx_bpmp",
                   configMAX_PRIORITIES - 6,
                   &ext_clients[TCU_ID_BPMP],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_tx_task_sce,
                   tcu_tx_task,
                   (char *)"tcu_tx_sce",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_SCE],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_tx_task_tz,
                   tcu_tx_task,
                   (char *)"tcu_tx_tz",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_TZ],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_tx_task_rce,
                   tcu_tx_task,
                   (char *)"tcu_tx_rce",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_RCE],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_tx_task_fsi,
                   tcu_tx_task,
                   (char *)"tcu_tx_fsi",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_FSI],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_rx_task_fsi,
                   tcu_rx_task,
                   (char *)"tcu_rx_fsi",
                   configMAX_PRIORITIES - 6,
                   &ext_clients[TCU_ID_FSI],
                   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_tx_task_psc,
				   tcu_tx_task,
				   (char *)"tcu_tx_fsi",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_PSC],
				   configMINIMAL_STACK_SIZE);

rtos_task_define_s(tcu_tx_task_dce,
                   tcu_tx_task,
                   (char *)"tcu_tx_dce",
                   configMAX_PRIORITIES - 5,
                   &ext_clients[TCU_ID_DCE],
                   configMINIMAL_STACK_SIZE);

static const rtosTaskParameters* tcu_clients_tx_task_params[] = {
    [TCU_ID_CCPLEX] = &tcu_tx_task_ccplex_params,
    [TCU_ID_BPMP]   = &tcu_tx_task_bpmp_params,
    [TCU_ID_SCE]    = &tcu_tx_task_sce_params,
    [TCU_ID_TZ]     = &tcu_tx_task_tz_params,
    [TCU_ID_RCE]    = &tcu_tx_task_rce_params,
    [TCU_ID_FSI]    = &tcu_tx_task_fsi_params,
    [TCU_ID_PSC]    = &tcu_tx_task_psc_params,
    [TCU_ID_DCE]    = &tcu_tx_task_dce_params,
};

static const rtosTaskParameters* tcu_clients_rx_task_params[] = {
    [TCU_ID_CCPLEX] = &tcu_rx_task_ccplex_params,
    [TCU_ID_BPMP]   = &tcu_rx_task_bpmp_params,
    [TCU_ID_SCE]    = NULL,
    [TCU_ID_TZ]     = NULL,
    [TCU_ID_RCE]    = NULL,
    [TCU_ID_FSI]    = &tcu_rx_task_fsi_params,
    [TCU_ID_PSC]    = NULL,
    [TCU_ID_DCE]    = NULL,
};

uint32_t get_tcu_client_count(void)
{
    return ARRAY_SIZE(ext_clients);
}

tcu_client_t* get_tcu_clients_handle(void)
{
    return ext_clients;
}

tcu_client_t* get_tcu_client_id(char tag)
{
    tcu_client_t *dest_id = &ext_clients[TCU_ID_CCPLEX];
    uint8_t      t = (uint8_t)tag;

    if (t >= TCU_TAG_MIN && t <= TCU_TAG_MAX) {
        dest_id = tag_to_ext_client[t - TCU_TAG_MIN];
    }

    return dest_id;
}

const rtosTaskParameters* get_tcu_client_tx_task_params(uint8_t client_id)
{
    return (client_id > ARRAY_SIZE(ext_clients)) ?
           NULL : tcu_clients_tx_task_params[client_id];
}

const rtosTaskParameters* get_tcu_client_rx_task_params(uint8_t client_id)
{
    return (client_id > ARRAY_SIZE(ext_clients)) ?
           NULL : tcu_clients_rx_task_params[client_id];
}
