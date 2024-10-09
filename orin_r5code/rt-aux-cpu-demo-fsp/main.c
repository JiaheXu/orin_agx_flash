/*
 * Copyright (c) 2014-2022, NVIDIA CORPORATION. All rights reserved.
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

#include <timeserver.h>

#include <stdint.h>
#include <address_map_new.h>
#include <cpu/arm-vic.h>
#include <arclk_rst.h>
#include <araopm.h>
#include <clk/clk-tegra.h>
#include <cpu/cache.h>
#include <delay/delay.h>
#include <err-hook.h>
#include <irq/irqs.h>
#include <processor/irqs-hw.h>
#include <misc/bitops.h>
#include <misc/nvrm_drf.h>

#include <reg-access/reg-access.h>
#include <spe-pm.h>
#include <spe-vic.h>
#include <stdio.h>
#include <tke/tke-tegra.h>
#include <processor/tke-tegra-hw.h>

#include <bpmp-ipc.h>
#include <hsp/hsp-tegra.h>
#include <processor/hsp-tegra-hw.h>
#include <ivc-channels.h>

#include <osa/rtos-task.h>
#include <osa/rtos-semaphore.h>
#include <osa/rtos-timer.h>
#include <late-init.h>
#include <tcu.h>

#if defined(ENABLE_GPCDMA_FUNC)
#include <gpcdma/gpcdma.h>
#include <gpcdma/gpcdma-errors.h>
#include <processor/gpcdma-hw.h>
#endif

#if defined(ENABLE_AODMIC_APP)
#include "app/aodmic-app.h"
#endif

#if defined(ENABLE_GPIO_APP)
#include <gpio/tegra-gpio.h>
extern struct tegra_gpio_id *chips[];
#include <gpio-aon.h>
#include "app/gpio-app.h"
#endif

#if defined(ENABLE_TIMER_APP)
#include "app/timer-app.h"
#endif

#if defined(ENABLE_I2C_APP)
#include "app/i2c-app.h"
#endif

#include "config.h"
#include "init_padctrl.h"
#include "debug_init.h"
#include "main.h"
#include "tegra-lic.h"

#define TEGRA_GPIO_AON_CHIPS_INDEX 1u
#define TEGRA_GPIO_AON_NUM_CHIPS   1u

struct TimeServerInformation timeserverdata;

int main(void)
{
  timeserverdata.ticks_persecond = 31250000;
  timeserverdata.phaseoffset_ticks = 0;
  timeserverdata.synced=0;
	
	spe_vic_init();

	lic_init();

	init_padctrl();

	tegra_tsc_init();

	debug_early_init();

	tegra_hsp_init(&tegra_hsp_id_aon);

	if (bpmp_ipc_init())
		printf("bpmp_ipc_init() failed\r\n");

	if (spe_late_init()) {
		error_hook("late init failed\r\n");
		goto out;
	}

	/* Start the tasks and timer running. */
	rtosTaskInitializeScheduler(NULL);

out:
	for( ;; )
		;

	return 0;
}

void main_task(void *params)
{
	/* For T19x, tegra_clk_init() and debug_init()
	 * cannot be called prior to late_init
	 * because they call tegra_platform_is_fpga()
	 * under the hood which requires the SOC domain
	 * to be powered on.
	 */
	tegra_clk_init();
	debug_init(SCRATCH_SCRATCH_COMB);
	if (ivc_init_channels_ccplex())
		printf("ivc_init_channels_ccplex() failed\r\n");

#if defined(ENABLE_GPCDMA_FUNC)
	gpcdma_init(&gpcdma_id_aon);
#endif

#if defined(ENABLE_AODMIC_APP)
	aodmic_app_init();
#endif

#if defined(ENABLE_GPIO_APP)
	if (tegra_gpio_init(&chips[TEGRA_GPIO_AON_CHIPS_INDEX],
				TEGRA_GPIO_AON_NUM_CHIPS))
		error_hook("tegra_gpio_init() failed\r\n");
	gpio_app_init();
#endif

#if defined(ENABLE_TIMER_APP)
	timer_app_init();
#endif

#if defined(ENABLE_I2C_APP)
	i2c_app_init();
#endif

	rtosTaskDelete(NULL);
}

/* Referenced from FreeRTOSConfig.h */
void setup_timer_interrupt(void)
{
	tegra_tke_set_up_tick(&tegra_tke_id_timer0,
				TIMER_CLK_SRC,
				configCPU_CLOCK_HZ / configTICK_RATE_HZ);
}

/* Referenced from freertos */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	printf("Stack Overflow");
	rtosTaskDisableInterrupts();
	for( ;; )
		;
}

void vApplicationGetIdleTaskMemory(rtosTaskBuffer **ppxIdleTaskTCBBuffer,
				   rtosStackType **ppxIdleTaskStackBuffer,
				   portUInt32Type *pulIdleTaskStackSize)
{
	static rtosTaskBuffer xIdleTaskTCB;
	static rtosStackType uxIdleTaskStack[configMINIMAL_STACK_SIZE];

	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(rtosTaskBuffer **ppxTimerTaskTCBBuffer,
				    rtosStackType **ppxTimerTaskStackBuffer,
				    portUInt32Type *pulTimerTaskStackSize)
{
	static rtosTaskBuffer xTimerTaskTCB;
	static rtosStackType uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
