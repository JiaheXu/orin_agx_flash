/*
 * Copyright (c) 2017-2022, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TCU_H
#define TCU_H

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>
#include <hsp/hsp-tegra.h>

/* local headers */
#include <osa/osa-task.h>
#include <osa/osa-errors.h>

/**
 * @brief TCU controller identification structure
 *
 * Structure that describes the identification of a TCU client.
 *
 * @hsp_id        hsp context for the client
 * @tx_id         tx hsp context for the client
 * @tx_task       task handle for the tx task
 * @tx_queue      queue for buffering tx data
 * @rx_queue      queue for buffering rx data
 * @recv_mbox     shared mailbox on which data is received
 * @send_mbox     shared mailbox on which data is sent
 * @id            client CPU id
 * @tag           client identifier for tcu_muxer
 * @aon_sm_for_tx indicates if the client uses AON mailbox for tx
 * @enabled       indicates if the client is enabled
 * @tx_fifo_full  indicates if the tx fifo is full
 * @rx_enabled    indicates if the client has rx support
 * @rx_timed_out  indicates if rx timeout occured
 */
typedef struct tcu_client {
    struct tegra_hsp_id       *hsp_id;
    struct tegra_hsp_id       *hsp_tx_id;
    rtosTaskHandle            tx_task;
    rtosQueueHandle           tx_queue;
    rtosQueueHandle           rx_queue;
    uint32_t                  recv_mbox;
    uint32_t                  send_mbox;
    uint8_t                   id;
    uint8_t                   tag;
    bool                      aon_sm_for_tx;
    bool                      enabled;
    bool                      tx_fifo_full;
    bool                      rx_enabled;
    bool                      rx_timedout;
} tcu_client_t;

/**
 * @brief set the global enable state for the Tegra Combined UART server
 *
 * This function sets the enable state of TCU.
 *
 * @return None
 */
void set_tcu_status(bool enable);

/**
 * @brief get the global enable state for the Tegra Combined UART server
 *
 * This function gets the enable state of TCU.
 *
 * @return true/false
 */
bool is_tcu_enabled(void);

/**
 * @brief pre-init setup for the Tegra Combined UART server
 *
 * This function initializes the pre-init TCU context for the server.
 *
 * @retval E_SUCCESS           indicates success
 * @retval E_TCU_PREINIT_FAIL  pre init failed
 */
error_t tcu_preinit(void);

/**
 * @brief Initialize the Tegra Combined UART server
 *
 * This function initializes the TCU context for the server.
 *
 * @param[in] uart_id uart port used by the TCU server.
 *
 * @retval E_SUCCESS         indicates success
 * @retval E_TCU_INVALID_ID  invalid uart port passed
 */
error_t tcu_init(uint32_t uart_id);

/**
 * @brief Initialize the Tegra Combined UART tasks
 *
 * This function creates and initializes the TCU tasks for clients.
 *
 * @retval E_SUCCESS         indicates success
 * @retval E_TCU_INIT_FAIL   tasks init failed
 */
error_t tcu_tasks_init(void);

/**
 * @brief save the TCU context across SC7 entry/power gating the hardware
 *
 * This function preserves the context of TCU  across power gating the
 * hardware(SC7).
 *
 * @pre the function tcu_init() has been called
 *
 * @retval E_SUCCESS     indicates success
 * @retval E_TCU_NO_INIT indicates TCU has not been initialized
 */
error_t tcu_suspend(void);

/**
 * @brief resume the TCU context across SC7 exit/unpower gating the hardware
 *
 * This function restores the TCU context across power gating the hardware(SC7).
 *
 * @pre the function tcu_init() has been called
 *
 * @retval E_SUCCESS     indicates success
 * @retval E_TCU_NO_INIT indicates TCU has not been initialized
 */
error_t tcu_resume(void);

/**
 * @brief fetch the external TCU clients count
 *
 * This function fetches the number of external TCU clients that are supported.
 *
 * @retval number of clients supported. 0 if none.
 */
uint32_t get_tcu_client_count(void);

/**
 * @brief fetch TCU clients array supported
 *
 * This is used to fetch the TCU client descriptors array supported
 * by the SPE TCU server.
 *
 * @return pointer to the client descriptors else NULL in the case
 *         of none
 */
tcu_client_t* get_tcu_clients_handle(void);

/**
 * @brief fetch TCU client descriptor from tag to id mapped array
 *
 * This is used to fetch the TCU client descriptor from tag to id
 * mapped array.
 *
 * @return pointer to the client descriptor else CCPLEX in the case
 *         of none
 */
tcu_client_t* get_tcu_client_id(char tag);

/**
 * @brief fetch TCU client tx task parameters
 *
 * This is used to fetch the TCU client tx task parameters to be used for
 * the task creation.
 *
 * @return pointer to the task parameters in case of success else NULL.
 */
const rtosTaskParameters* get_tcu_client_tx_task_params(uint8_t client_id);

/**
 * @brief fetch TCU client tx task parameters
 *
 * This is used to fetch the TCU client tx task parameters to be used for
 * the task creation.
 *
 * @return pointer to the task parameters in case of success else NULL.
 */
const rtosTaskParameters* get_tcu_client_rx_task_params(uint8_t client_id);

/**
 * @brief fetch TCU client tx queue buffer pointer
 *
 * This is used to fetch the TCU client tx queue buffer to be used for
 * queue creation.
 *
 * @return pointer to the queue buffer in memory in case of success else NULL.
 */
char* get_tcu_client_tx_queue_buf(uint8_t client_id);

/**
 * @brief fetch TCU client rx queue buffer pointer
 *
 * This is used to fetch the TCU client rx queue buffer to be used for
 * queue creation.
 *
 * @return pointer to the queue buffer in memory in case of success else NULL.
 */
char* get_tcu_client_rx_queue_buf(uint8_t client_id);

/**
 * @brief print a message on the tcu console.
 *
 * This function spews a message on to the TCU uart.
 *
 * @pre the function tcu_init() has been called
 *
 * @param[in] msg_buf  buffer holding the message to be spewed.
 * @param[in] len      length of the message to be spewed.
 * @param[in] from_isr flag to indicate spew from isr.
 *
 * @retval number of characters sent to the TCU successfully
 */
int tcu_print_msg(const char* msg_buf, int len, bool from_isr);

#endif
