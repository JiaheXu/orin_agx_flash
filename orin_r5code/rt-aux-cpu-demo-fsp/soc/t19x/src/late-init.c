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

#include <stdio.h>
#include <stdint.h>
#include <address_map_new.h>
#include <delay/delay.h>
#include <err-hook.h>
#include <irq/irqs.h>
#include <processor/irqs-hw.h>
#include <cpu/arm-vic.h>
#include <misc/bitops.h>

#include <reg-access/reg-access.h>

#include <osa/rtos-task.h>
#include <osa/rtos-semaphore.h>
#include <osa/rtos-timer.h>
#include <late-init.h>
#include <spe-pm.h>
#include "config.h"
#include "main.h"

#define WAIT_TIMEOUT_MS 3000
#define MB1_DB_IRQ (NV_AON_INTERRUPT_VIC1_BASE + NV_AON_INTERRUPT_TOP0_HSP_DB)

static rtosSemaphoreHandle late_init_sem;

static const rtosTaskParameters main_task_params = {
	.pvTaskCode = &main_task,
	.pcTaskName = (char *)"main_task",
	.uxPriority = configMAX_PRIORITIES - 1,
	.pvParameters = NULL,
	.uxStackDepthBytes = 256,
};

static void late_init_task(void *params)
{
	(void)params;
	if (rtosSemaphoreAcquire(late_init_sem,
				 WAIT_TIMEOUT_MS / portTICK_PERIOD_MS)
				 == pdFALSE)
		warning_hook("No doorbell received from MB1");

	if (rtosTaskCreate(&main_task_params, NULL) != rtosPASS)
		error_hook("Couldn't create main_task");

	rtosSemaphoreDelete(late_init_sem);
	late_init_sem = NULL;
	rtosTaskDelete(NULL);
}

static const rtosTaskParameters late_init_task_params = {
	.pvTaskCode = &late_init_task,
	.pcTaskName = (char *)"late_init",
	.uxPriority = rtosIDLE_PRIORITY,
	.pvParameters = NULL,
	.uxStackDepthBytes = 256,
};

static void late_init_db_irq(void *data)
{
	rtosPortBaseType yield;
	(void)data;

	if (rtosSemaphoreReleaseFromISR(late_init_sem, &yield) != pdTRUE)
		error_hook("Couldn't give semaphore.");

	irq_disable(MB1_DB_IRQ);
	irq_set_handler(MB1_DB_IRQ, NULL, NULL);
	rtosTaskYieldFromISR(yield);
}

int spe_late_init(void)
{
	/* select FIQ interrupt, spe-safertos does not init this */
	arm_vic_write_intselect(NV_ADDRESS_MAP_AON_VIC_0_BASE, BIT(NV_AON_INTERRUPT_WDTFIQ));

	spe_pm_init();

	/* Late init task initializing IVC with CCPLEX */
	rtosSemaphoreCreateBinary(NULL, &late_init_sem);
	if (!late_init_sem) {
		error_hook("Couldn't create semaphore!");
		return -1;
	}
	if (rtosTaskCreate(&late_init_task_params, NULL) != rtosPASS) {
		error_hook("Couldn't create late_init_task");
		return -1;
	} else {
		irq_set_handler(MB1_DB_IRQ, late_init_db_irq, NULL);
		irq_enable(MB1_DB_IRQ);
	}

	return 0;
}