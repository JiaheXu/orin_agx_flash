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
#include <address_map_new.h>
#include <stdbool.h>
#include <late-init.h>
#include <spe-clk.h>
#include <spe-pm.h>
#include <processor/hsp-tegra-hw.h>
#include <hsp/hsp-tegra.h>
#include <reg-access/reg-access.h>
#include <address_map_new.h>
#include <araopm.h>
#include <misc/nvrm_drf.h>
#include <chipid/chip-id.h>
#include "config.h"
#include "main.h"
#include <osa/rtos-task.h>
#include <err-hook.h>

#define AOPM_AON_BLOCK_REG (NV_ADDRESS_MAP_AON_PM_IMPL_BASE + AOPM_AON_BLOCK_0)

static const rtosTaskParameters main_task_params = {
	.pvTaskCode = &main_task,
	.pcTaskName = (char *)"main_task",
	.uxPriority = configMAX_PRIORITIES - 1,
	.pvParameters = NULL,
	.uxStackDepthBytes = 256,
};

static bool spe_can_access_soc(void)
{
	uint32_t val;
	uint32_t aon2snic_req;

	val = readl(AOPM_AON_BLOCK_REG);
	aon2snic_req = NV_DRF_VAL(AOPM, AON_BLOCK, AON2SNIC_REQ, val);

	return (aon2snic_req == 0UL);
}

int spe_late_init(void)
{
	/* check if SPE access to SOC is unblocked */
	if (tegra_platform_is_silicon()) {
		if (!spe_can_access_soc())
		{
			printf("spe waiting on blocked SOC access...!\r\n");
			while (!spe_can_access_soc())
				;
		}
	}

	spe_clk_setup_cpu_nic(CLK_SRC_PLLP);
	spe_clk_setup_apb(CLK_SRC_PLLP);

	spe_pm_init();
	tegra_hsp_init(&tegra_hsp_id_bpmp);
	tegra_hsp_init(&tegra_hsp_id_top1);

	if (rtosTaskCreate(&main_task_params, NULL) != rtosPASS)
		error_hook("Couldn't create main_task");

	return 0;
}