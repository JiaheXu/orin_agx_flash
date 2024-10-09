/*
 * Copyright (c) 2020-2022, NVIDIA CORPORATION. All rights reserved.
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

#ifndef SPE_ERRORS_H
#define SPE_ERRORS_H

#include <error/error.h>

#define SPE_ERROR(_x_)                  (MODULE_ERROR(APPLICATION, (_x_)))

#define E_SPE_ERR_NOT_IMPLEMENTED       SPE_ERROR(1U)
#define E_SPE_ERR_NULL_PTR              SPE_ERROR(2U)
#define E_SPE_ERR_TIMEOUT               SPE_ERROR(3U)
#define E_BPMP_IPC_SEND_FROM_ISR_FAIL   SPE_ERROR(4U)
#define E_BPMP_IPC_SEND_FAIL            SPE_ERROR(5U)
#define E_BPMP_IPC_INIT_FAIL            SPE_ERROR(6U)
#define E_TCU_PORT_INVALID              SPE_ERROR(7U)
#define E_TCU_PREINIT_FAIL              SPE_ERROR(8U)
#define E_TCU_INIT_FAIL                 SPE_ERROR(9U)
#define E_TCU_CLIENT_NO_TX_QUEUE        SPE_ERROR(10U)
#define E_TCU_CLIENT_NO_TX_TASK         SPE_ERROR(11U)
#define E_TCU_CLIENT_TX_SETUP_FAIL      SPE_ERROR(12U)
#define E_TCU_CLIENT_NO_RX_QUEUE        SPE_ERROR(13U)
#define E_TCU_CLIENT_NO_RX_TASK         SPE_ERROR(14U)
#define E_TCU_CLIENT_RX_SETUP_FAIL      SPE_ERROR(15U)
#define E_TCU_EXT_CLIENTS_INIT_FAIL     SPE_ERROR(16U)
#define E_AST_REGION_SIZE_INVALID       SPE_ERROR(17U)
#define E_CCPLEX_IPC_INIT_FAIL          SPE_ERROR(18U)
#define E_CCPLEX_IPC_NO_SUPPORT         SPE_ERROR(19U)
#define E_CCPLEX_IPC_NOT_INITED         SPE_ERROR(20U)
#define E_IVC_CHAN_INIT_FAIL            SPE_ERROR(21U)
#define E_IVC_TASK_CREATE_FAIL          SPE_ERROR(22U)
#define E_IVC_INVALID_CPU_ID            SPE_ERROR(23U)
#define E_IVC_CHAN_NOTIFY_FAIL          SPE_ERROR(24U)
#define E_IVC_INVALID_PARAM             SPE_ERROR(25U)
#define E_INVALID_CLK_FREQ              SPE_ERROR(26U)
#define E_INVALID_CLK_ID                SPE_ERROR(27U)
#define E_INVALID_CLK_SRC               SPE_ERROR(28U)
#define E_INVALID_PARENT_FREQ           SPE_ERROR(29U)
#define E_TCU_NO_INIT                   SPE_ERROR(30U)
#define E_TCU_NO_SUSPEND                SPE_ERROR(31U)

#endif
