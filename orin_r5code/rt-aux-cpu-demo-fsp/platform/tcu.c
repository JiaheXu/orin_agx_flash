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

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* local os headers */
#include <osa/osa-task.h>
#include <osa/osa-errors.h>
#include <osa/osa-semaphore.h>
#include <osa/osa-queue.h>
#include <osa/osa-mutex.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <chipid/chip-id.h>
#include <error/common-errors.h>
#include <hsp/hsp-tegra.h>
#include <processor/hsp-tegra-hw.h>
#include <debug/print.h>
#include <misc/macros.h>
#include <irq/irqs.h>
#include <tke/tke-tegra.h>
#include <misc/bitops.h>

/* local headers */
#include <spe-uart.h>
#include <mbox-aon.h>
#include <tcu.h>
#include <tcu-priv.h>
#include <spe-errors.h>
#include <err-hook.h>

#define NUM_BYTES_FIELD_BIT       24U
#define FLUSH_BIT                 26U
#define HW_FLUSH_BIT              27U
#define DMA_MODE_BIT              30U
#define INTR_TRIGGER_BIT          31U
#define NUM_BYTES(reg_val)        ((reg_val >> NUM_BYTES_FIELD_BIT) & 0x3U)

#define SPE_TX_QUEUE_SIZE         rtos_queue_size(SPE_NUM_MSGS, \
                                             sizeof(uint32_t))

#define TCU_ESCAPE_START          0xFF
#define TCU_ESCAPE_CHAR           0xFF
#define TAG_ESCAPE_CHAR           0xFF
#define TAG_ESCAPE_STRING         "\377"

#define TAG_RESET                 0xFD
#define TAG_BOOT_RESET_STRING     "\0\377\375"

#define RX_TIMEOUT_MS             100UL
#define RX_TIMEOUT_US             (RX_TIMEOUT_MS * 1000UL)

#define ESCAPE                    2UL
#define NORMAL                    1UL

static struct spe_uart *uart = NULL;
static bool tcu_enable       = false;
static bool tcu_inited       = false;
static bool tcu_preinited    = false;
static bool tcu_suspended    = false;

static rtosSemaphoreHandle tx_sem;

static char spe_tx_fifo[SPE_NUM_MSGS][SPE_MSG_SIZE];

static uint32_t spe_tx_wr = 0;
static uint32_t spe_tx_rd = 0;

static tcu_client_t spe_id = {
    .tag = 0xe0,
};
static const tcu_client_t *last_id = NULL;
static tcu_client_t *ext_clients;
static uint32_t     nclients;

/* spe tx task handle, used on notifications to the task */
static rtosTaskHandle spe_tx_handle;

/* spe rx task handle, used on notifications to the task */
static rtosTaskHandle spe_rx_handle;

static void spe_tx_task (void *param);
static void spe_rx_task (void *param);
static void
tcu_print(const tcu_client_t *id,
          const char *msg_buf,
          int32_t len);
int _write(int file, char *ptr, int len);

rtos_task_define_s(spe_tx_task,
                 spe_tx_task,
                 (char *)"spe_tx",
                 configMAX_PRIORITIES - 3,
                 &spe_id,
                 configMINIMAL_STACK_SIZE);

rtos_task_define_s(spe_rx_task,
                 spe_rx_task,
                 (char *)"spe_rx",
                 configMAX_PRIORITIES - 2,
                 &spe_id,
                 configMINIMAL_STACK_SIZE);

void tcu_tx_task(void *param);
void tcu_rx_task(void *param);

int _write(int file, char *ptr, int len)
{

	if (len <= 0)
		return len;

	/* stderr */
	if (file == 2) {

		len = tcu_print_msg(ptr, len, true);
		return len;
	}

	if (file != 1)
		return -1;

	/* stdout */
	tcu_print(&spe_id, ptr, len);
	return len;
}

void
set_tcu_status(bool enable)
{
    tcu_enable = enable;
}

bool
is_tcu_enabled(void)
{
    return tcu_enable;
}

static void
tcu_raw_print(const char *str,
              int32_t len)
{
    uint32_t written = 0UL;
    uint32_t wcount = 0UL;
    error_t  ret;

    if (!tcu_inited) {
        goto out;
    }

    if (in_interrupt() || in_critical() ||
                (rtosTaskIsSchedulerStarted() == rtosFALSE)) {
        uart->ops->write_now(uart->ctlr, str, len);
        goto out;
    }

    while (written < len) {
        ret = uart->ops->write(uart->ctlr,
                               str,
                               len - written,
                               &wcount,
                               rtosMAX_DELAY);
        if (ret != E_SUCCESS) {
            break;
        }
        written += wcount;
    }

out:
    return;
}

static void
tcu_dbg_putc(const char *str,
             int32_t len)
{
    const char *buf, *tag_escape;
    int32_t    remaining;
    int32_t    current_len;

    buf = str;
    remaining = len;
    while (remaining > 0) {
        tag_escape = memchr(buf, TAG_ESCAPE_CHAR, remaining);

        current_len = tag_escape ?
                      tag_escape - buf + 1 :
                      remaining;

        tcu_raw_print(buf, current_len);
        if (buf[current_len - 1] == TAG_ESCAPE_CHAR) {
            tcu_raw_print(TAG_ESCAPE_STRING, 1);
        }

        buf += current_len;
        remaining -= current_len;
    }
}

static void
tcu_print(const tcu_client_t *id,
          const char *msg_buf,
          int32_t len)
{
    char c;

    if (last_id != id) {
        tcu_raw_print(TAG_ESCAPE_STRING, 1);
        c = id->tag;
        tcu_raw_print(&c, 1);
        last_id = id;
    }

    if (len > 0) {
        tcu_dbg_putc(msg_buf, len);
    }
}

int
tcu_print_msg(const char* msg_buf,
              int len,
              bool from_isr)
{
    error_t status;

    if (len <= 0) {
        goto out;
    }

    if (from_isr || (rtosTaskIsSchedulerStarted() == rtosFALSE)) {
        tcu_print(&spe_id, msg_buf, len);
        goto out;
    }

    len = (len > SPE_MSG_SIZE) ? SPE_MSG_SIZE : len;

    enter_critical();
    memcpy(spe_tx_fifo[spe_tx_wr], msg_buf, len);
    spe_tx_wr = (spe_tx_wr + 1) & (SPE_NUM_MSGS - 1);
    exit_critical();

    status = rtosQueueSend(spe_id.tx_queue,
                             &len,
                             tcu_inited ? rtosMAX_DELAY : 0);
    if (status != rtosPASS) {
        len = 0;
        goto out;
    }

out:
    return len;
}

static inline bool
msg_needs_deferred_ack(uint32_t msg)
{
    return ((msg & (BIT(FLUSH_BIT) | BIT(HW_FLUSH_BIT))) != 0UL);
}

static void
tcu_irq_handler(void *priv,
                hsp_sm_data_t *value)
{
    tcu_client_t *client = (tcu_client_t *)priv;
    rtosBool     higher_prio_task_woken = false;
    uint32_t     data;
    error_t      rc;

    client->rx_enabled = true;
    data = value->sm.val;
    data &= ~MBOX_TAG_BIT;

    rc = rtosQueueSendToBackFromISR(client->tx_queue,
                                  &data,
                                  &higher_prio_task_woken);
    if (rc != rtosPASS) {
        error_hook("rtosQueueSendFromISR() failed\r\n");
        goto out;
    }

    if (msg_needs_deferred_ack(data)) {
        (void) tegra_hsp_sm_full_disable(client->hsp_tx_id, client->recv_mbox);
    } else if (rtosQueueIsQueueFullFromISR(client->tx_queue)) {
        client->tx_fifo_full = true;
        (void) tegra_hsp_sm_full_disable(client->hsp_tx_id, client->recv_mbox);
    } else {
        (void) tegra_hsp_sm_vacate(client->hsp_tx_id, client->recv_mbox);
    }

out:
    rtosTaskYieldFromISR(higher_prio_task_woken);
}

static void
tcu_process_tx_queue_msg(tcu_client_t *client,
                         uint32_t msg)
{
    uint32_t nbytes;
    uint32_t i;
    error_t  rc;
    char     buf[3];

    /* fetch number of bytes from the message packet */
    nbytes = NUM_BYTES(msg);

    /* extract chars from message packet */
    for (i = 0UL; i < nbytes; i += 1UL) {
        buf[i] = (msg >> (i * 8U)) & 0xFFU;
    }

    /* acquire tx semaphore */
    rc = rtosSemaphoreAcquire(tx_sem, rtosMAX_DELAY);
    if (rc != rtosPASS) {
        goto out;
    }

    tcu_print(client, buf, nbytes);

    /* handle flush if specified */
    if ((msg & BIT(HW_FLUSH_BIT)) != 0UL) {
        (void) uart->ops->flush(uart->ctlr);
        (void) uart->ops->flush_hw(uart->ctlr);
    }

    /* release tx semaphore */
    rtosSemaphoreRelease(tx_sem);

out:
    return;
}

void
tcu_tx_task(void *param)
{
    tcu_client_t *client = (tcu_client_t *)param;
    uint32_t     msg;
    error_t      status;
    error_t      ret;
    bool         tx_fifo_full;
    rtosError    queue_available;

    for (;;) {
        /* check for any pending messages on the tx fifo */
        status = rtosQueueReceive(client->tx_queue,
                                    &msg,
                                    rtosMAX_DELAY);
        enter_critical();
        if (status != rtosPASS) {
            exit_critical();
            continue;
        }
        tx_fifo_full = client->tx_fifo_full;
        queue_available = rtosQueueSpacesAvailable(client->tx_queue);
        exit_critical();

        tcu_process_tx_queue_msg(client, msg);

        if ((tx_fifo_full && queue_available != 0) ||
                                   msg_needs_deferred_ack(msg)) {
            /*
             * Entering this block imples MBOX irq is disabled. Therefore,
             * synchronization is not necessary.
             */
            client->tx_fifo_full = false;
            ret = tegra_hsp_sm_vacate(client->hsp_tx_id, client->recv_mbox);
            if (ret != E_SUCCESS) {
                continue;
            }
            ret = tegra_hsp_sm_full_enable(client->hsp_tx_id,
                                           client->recv_mbox,
                                           tcu_irq_handler,
                                           client);
        }
    }
}

static void
send_rx_chars(tcu_client_t *client,
              uint8_t *chs,
              uint32_t len)
{
    uint32_t mbox_val;
    uint32_t tstart;
    uint32_t i;

    mbox_val = (len << NUM_BYTES_FIELD_BIT) | BIT(INTR_TRIGGER_BIT);
    for (i = 0UL; i < len; i += 1UL) {
        mbox_val |= ((chs[i] & 0xFFU) << (i * 8U));
    }

    tstart = tegra_tke_get_usec();
    while (!tegra_hsp_sm_is_empty(client->hsp_id, client->send_mbox)) {
        if (client->rx_timedout) {
            goto out;
        }

        if (tegra_tke_get_usec() - tstart > RX_TIMEOUT_US) {
            client->rx_timedout = true;
            goto out;
        }
    }

    client->rx_timedout = false;
    (void)tegra_hsp_sm_produce(client->hsp_id, client->send_mbox, mbox_val);

out:
    return;
}

void
tcu_rx_task(void *param)
{
    tcu_client_t  *client = (tcu_client_t *)param;
    uint8_t       chs[3];
    uint8_t       ch;
    uint32_t      len;
    uint32_t      i;
    error_t       status;

    for (;;) {
        len = 0UL;
        for (i = 0UL; i < ARRAY_SIZE(chs); i += 1UL) {
            /* check for any pending messages on the tx fifo */
            status = rtosQueueReceive(client->rx_queue,
                                        &ch,
                                        (i == 0UL) ? rtosMAX_DELAY : 0);
            if (status != rtosPASS) {
                break;
            }
            chs[len++] = ch;
        }

        if ((len > 0UL) && client->rx_enabled) {
            send_rx_chars(client, chs, len);
        }
    }
}

static void
spe_rx_task (void *param)
{
    tcu_client_t *dest_id = &ext_clients[TCU_ID_CCPLEX];
    error_t      ret;
    char         c;
    uint32_t     rcount = 0;
    uint32_t     state = NORMAL;
    (void)       param;

    for (;;) {
        ret = uart->ops->read(uart->ctlr, &c, 1, &rcount, rtosMAX_DELAY);
        if (ret != E_SUCCESS) {
            continue;
        }

        switch (state) {
            case ESCAPE:
                /* Handle escape char */
                if (c == TCU_ESCAPE_CHAR) {
                    (void)rtosQueueSend(dest_id->rx_queue, &c, 0);
                } else if (c == TAG_RESET) {
                    rtosSemaphoreAcquire(tx_sem, rtosMAX_DELAY);
                    last_id = NULL;
                    rtosSemaphoreRelease(tx_sem);
                } else {
                    /* void the input */
                    if (c == spe_id.tag) {
                         dest_id = &spe_id;
                    } else {
                        dest_id = get_tcu_client_id(c);
                    }
                }
                state = NORMAL;
                break;
            default: /* case: NORMAL */
                if (c == TCU_ESCAPE_START) {
                    state = ESCAPE;
                } else {
                    (void)rtosQueueSend(dest_id->rx_queue, &c, 0);
                }
                break;
        }
    }
}

static void
spe_tx_task (void *param)
{
    tcu_client_t *id = (tcu_client_t *)param;
    error_t      status;
    uint32_t     len;


    /* flush the HW TX fifo */
    uart->ops->flush_hw(uart->ctlr);

    for (;;) {
        /* check for any pending messages on the tx fifo */
        status = rtosQueueReceive(id->tx_queue,
                                    &len,
                                    rtosMAX_DELAY);
        if (status != rtosPASS) {
            continue;
        }
        /* acquire tx semaphore */
        status = rtosSemaphoreAcquire(tx_sem,
                                  rtosMAX_DELAY);
        if (status != rtosPASS) {
            continue;
        }

        /* print message and update the read pointer in SPE tx fifo*/
        tcu_print(id, spe_tx_fifo[spe_tx_rd], len);
        spe_tx_rd = (spe_tx_rd + 1UL) & (SPE_NUM_MSGS - 1UL);

        /* release tx semaphore */
        rtosSemaphoreRelease(tx_sem);
    }
}

error_t
tcu_preinit(void)
{
    error_t ret = E_SUCCESS;

    rtosQueueCreate(NULL,
                    0,
                    SPE_NUM_MSGS,
                    sizeof(uint32_t),
                    &spe_id.tx_queue);

    if (!spe_id.tx_queue) {
        ret = E_TCU_PREINIT_FAIL;
        goto out;
    }

    tcu_preinited = true;

out:
    return ret;
}

error_t
tcu_init(uint32_t uart_id)
{
    error_t ret = E_SUCCESS;

    /* validate the uart controller id */
    uart = spe_uart_get(uart_id);
    if (uart == NULL || uart->ctlr == NULL) {
        ret = E_TCU_PORT_INVALID;
        goto out;
    }

    /* Initialize uart port */
    if (tcu_enable) {
        ret = uart->ops->init(uart->ctlr, uart->conf);
        if (ret != E_SUCCESS) {
            goto out;
        }
    } else {
        ret = uart->ops->init_3rdparty(uart->ctlr, uart->conf);
        goto out;
    }

    /* update the tcu uart controller init status */
    tcu_inited = true;

    tcu_raw_print(TAG_BOOT_RESET_STRING, 3);

    /* create SPE tx queue only if preinit was not called.
     * preinit is only needed for t19x and not for t23x as
     * MB2 loads SPE on t23x and UART can be accessed from
     * early boot.
     */
    if (!tcu_preinited) {
        rtosQueueCreate(NULL,
                        0,
                        SPE_NUM_MSGS,
                        sizeof(uint32_t),
                        &spe_id.tx_queue);
        if (!spe_id.tx_queue) {
            ret = E_TCU_INIT_FAIL;
            goto out;
        }
    }

    rtosMutexCreate(NULL, &tx_sem);
    if (!tx_sem) {
        ret = E_TCU_INIT_FAIL;
        goto out;
    }
    rtosSemaphoreRelease(tx_sem);

out:
    return ret;
}

static error_t
tcu_client_setup_tx(tcu_client_t *client)
{
    error_t                ret = E_SUCCESS;
    error_t                status;
    const rtosTaskParameters *task_params;

    /* create the client tx task queue */
    status = rtosQueueCreate(NULL,
                             0,
                             EXT_CLIENT_MSG_SIZE,
                             sizeof(uint32_t),
                             &client->tx_queue);
    if (!client->tx_queue) {
        error_hookf("rtosQueueCreate(tcu_tx id %d) failed!\r\n", client->id);
        ret = E_TCU_CLIENT_TX_SETUP_FAIL;
        goto out;
    }

    /* fetch the client tx task parameters */
    task_params = get_tcu_client_tx_task_params(client->id);

     /* create the client tx task */
    status = rtosTaskCreate(task_params, NULL);
    if (status != rtosPASS) {
        error_hookf("rtosTaskCreate(tcu_tx %d) failed!\r\n", client->id);
        ret = E_TCU_CLIENT_TX_SETUP_FAIL;
        goto out;
    }

    /* Initialize tx hsp if client does not use AON sm for tx */
    if (client->aon_sm_for_tx) {
        client->hsp_tx_id = &tegra_hsp_id_aon;
    } else {
        ret = tegra_hsp_init(client->hsp_tx_id);
        if (ret != E_SUCCESS) {
            error_hook("tcu_tx hsp init failed!\r\n");
            goto out;
        }
    }

    ret = tegra_hsp_sm_full_enable(client->hsp_tx_id,
                                   client->recv_mbox,
                                   tcu_irq_handler,
                                   client);
    if (ret != E_SUCCESS) {
        error_hookf("hsp_sm_full_enable(tcu_tx %d) failed!\r\n", client->id);
        ret = E_TCU_CLIENT_TX_SETUP_FAIL;
    }

out:
    return ret;
}

static error_t
tcu_client_setup_rx(tcu_client_t *client)
{
    error_t                ret = E_SUCCESS;
    error_t                status;
    const rtosTaskParameters *task_params;

    /* It is okay to not support RX */
    if (client->hsp_id == NULL) {
        goto out;
    }

    /* support RX and create the client rx task queue */
    status = rtosQueueCreate(NULL,
                             0,
                             EXT_CLIENT_RX_BUF_SIZE,
                             1,
                             &client->rx_queue);
    if (!client->rx_queue) {
        error_hookf("rtosQueueCreate(tcu_rx id %d) failed!\r\n", client->id);
        ret = E_TCU_CLIENT_RX_SETUP_FAIL;
        goto out;
    }

    /* fetch the client rx task parameters */
    task_params = get_tcu_client_rx_task_params(client->id);
    if (task_params == NULL) {
        error_hookf("get_tcu_rx_task_params(id: %d) failed!\r\n", client->id);
        ret = E_TCU_CLIENT_NO_RX_TASK;
        goto out;
    }

    /* initialize the rx HSP context */
    ret = tegra_hsp_init(client->hsp_id);
    if (ret != E_SUCCESS) {
        error_hook("tcu_rx hsp init failed!\r\n");
        goto out;
    }

    /* create the rx task */
    status = rtosTaskCreate(task_params, NULL);
    if (status != rtosPASS) {
        error_hookf("rtosTaskCreate(tcu_rx id %d) failed!\r\n", client->id);
        ret = E_TCU_CLIENT_RX_SETUP_FAIL;
    }

out:
    return ret;
}

static error_t
tcu_clients_init(void)
{
    error_t      ret = E_SUCCESS;
    uint32_t     i;
    tcu_client_t *client;

    nclients = get_tcu_client_count();

    if (nclients == 0UL) {
        goto out;
    }

    ext_clients = get_tcu_clients_handle();
    for (i = 0UL; i < nclients; i += 1UL) {
        client = &ext_clients[i];

        if (!client->enabled) {
            continue;
        }

        client->rx_enabled = false;
        client->rx_timedout = false;
        client->tx_fifo_full = false;

        ret = tcu_client_setup_rx(client);
        if (ret != E_SUCCESS) {
            error_hookf("tcu_client_setup_rx() failed!: %ld\r\n", ret);
            break;
        }

        ret = tcu_client_setup_tx(client);
        if (ret != E_SUCCESS) {
            error_hookf("tcu_client_setup_tx() failed!: %ld\r\n", ret);
            break;
        }
    }

out:
    return ret;
}

error_t
tcu_tasks_init(void)
{
    error_t ret = E_SUCCESS;
    error_t status;

    /*
     * create the SPE tx task which will buffer uart logging from
     * SPE.
     */
    status = rtosTaskCreate(&spe_tx_task_params, &spe_tx_handle);
    if (status != rtosPASS) {
        ret = E_TCU_INIT_FAIL;
        goto out;
    }

    status = rtosTaskCreate(&spe_rx_task_params, &spe_rx_handle);
    if (status != rtosPASS) {
        ret = E_TCU_INIT_FAIL;
        goto out;
    }

    /* create the external clients TX and RX tasks */
    ret = tcu_clients_init();
    if (ret != E_SUCCESS) {
        ret = E_TCU_EXT_CLIENTS_INIT_FAIL;
    }

out:
    return ret;
}


error_t
tcu_suspend(void)
{
    error_t         ret = E_SUCCESS;
    error_t         rc;
    tcu_client_t    *bpmp;
    tcu_client_t    *ccplex;

    if (!tcu_inited) {
        ret = E_TCU_NO_INIT;
        goto out;
    }

    bpmp = &ext_clients[TCU_ID_BPMP];
    ccplex = &ext_clients[TCU_ID_CCPLEX];

    rc = rtosSemaphoreAcquire(tx_sem, rtosMAX_DELAY);
    if (rc != rtosPASS) {
        ret = rc;
        error_hook("tcu tx sem acquire failed!\r\n");
        goto out;
    }

    tcu_print(bpmp, "", 0);
    tcu_inited = false;

    /* release tx semaphore */
    rtosSemaphoreRelease(tx_sem);

    ret = tegra_hsp_sm_full_disable(ccplex->hsp_tx_id, ccplex->recv_mbox);
    if (ret != E_SUCCESS) {
        error_hook("tcu ccplex tx irq disable failed!\r\n");
    }

    ret = uart->ops->flush(uart->ctlr);
    if (ret != E_SUCCESS) {
        error_hook("tcu flush failed!\r\n");
    }

    ret = uart->ops->flush_hw(uart->ctlr);
    if (ret != E_SUCCESS) {
        error_hook("tcu hw flush failed!\r\n");
    }

    ret = uart->ops->suspend(uart->ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    tcu_suspended = true;

out:
    return ret;
}

error_t
tcu_resume(void)
{
    error_t ret = E_SUCCESS;
    tcu_client_t *ccplex;

    if (!tcu_suspended) {
        ret = E_TCU_NO_SUSPEND;
        goto out;
    }

    ccplex = &ext_clients[TCU_ID_CCPLEX];

    ret = uart->ops->resume(uart->ctlr);
    if (ret != E_SUCCESS) {
        goto out;
    }

    tcu_inited = true;
    tcu_suspended = false;

    ret = tegra_hsp_sm_full_enable(ccplex->hsp_tx_id,
                                   ccplex->recv_mbox,
                                   tcu_irq_handler,
                                   ccplex);
    if (ret != E_SUCCESS) {
        error_hook("tcu ccplex tx resume failed!\r\n");
    }

out:
    return ret;
}
