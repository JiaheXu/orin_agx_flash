/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION. All rights reserved.
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
#include <osa/rtos-task.h>

#include <address_map_new.h>
#include <arclk_rst.h>
#include <araopm.h>
#include <argpio_aon_sw.h>
#include <arpmc_impl.h>
#include <arwake.h>
#include <arscratch.h>

#include <misc/nvrm_drf.h>
#include <debug/print.h>
#include <delay/delay.h>
#include <misc/macros.h>
#include <reg-access/reg-access.h>
#include <hsp/hsp-tegra.h>
#include <processor/hsp-tegra-hw.h>

#include <bpmp-ipc.h>
#include <bpmp-ipc-protocol.h>
#include <spe-clk.h>
#include <spe-pm.h>
#include <spe-vic.h>

#define RAM_PG_CTRL_OFFSET 0
#define RAM_PG_STATUS_OFFSET (PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0 - \
			      PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0)
#define RAM_CLAMP_CTRL_OFFSET (PMC_IMPL_PART_AOPG_CACHE_CLAMP_CONTROL_0 - \
			       PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0)

#define aopm_addr(offset) (NV_ADDRESS_MAP_AON_PM_BASE + offset)

static enum pm_ram_pg_status cache_pg_state = PM_RAM_ON;

static bool pllaon_disabled = true;

static struct tegra_hsp_suspend_ctx top_hsp_ctx;

void set_pllaon_state(bool enable)
{
	rtosTaskEnterCritical();
	pllaon_disabled = !enable;
	rtosTaskExitCritical();
}

bool is_pllaon_enabled(void)
{
	return !pllaon_disabled;
}

static enum pm_ram_pg_status pm_ram_powergate(uint32_t base)
{
	uint32_t val;

	/* Start PG */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			 INTER_PART_DELAY_EN, ENABLE);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				 SRAM_SLEEP_EN, ON, val);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				 LOGIC_SLEEP, ON, val);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				 START, PENDING, val);
	writel(val, base + RAM_PG_CTRL_OFFSET);

	/* Wait until PG is done */
	do {
		val = readl(base + RAM_PG_CTRL_OFFSET);
		val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				 START, val);
	} while (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0_START_DONE);

	/* Verify status */
	val = readl(base + RAM_PG_STATUS_OFFSET);
	val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_STATUS,
			 SRAM_SLEEP_STS, val);
	if (val == PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0_SRAM_SLEEP_STS_ON)
		return PM_RAM_OFF;
	else
		return PM_RAM_ERROR;
}

static enum pm_ram_pg_status pm_ram_unpowergate(uint32_t base)
{
	uint32_t val;

	/* Start un-PG */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			 INTER_PART_DELAY_EN, ENABLE);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				 START, PENDING, val);
	writel(val, base + RAM_PG_CTRL_OFFSET);

	/* Wait until un-PG is done */
	do {
		val = readl(base + RAM_PG_CTRL_OFFSET);
		val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				 START, val);
	} while (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0_START_DONE);

	/* Verify status */
	val = readl(base + RAM_PG_STATUS_OFFSET);
	val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_STATUS,
			 SRAM_SLEEP_STS, val);
	if (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0_SRAM_SLEEP_STS_OFF)
		return PM_RAM_ERROR;

	return PM_RAM_ON;
}

void pm_hw_init(void)
{
}

void pm_turn_off_caches(void)
{
	if (cache_pg_state != PM_RAM_ON)
		return;

	cache_pg_state = pm_ram_powergate(NV_ADDRESS_MAP_PMC_IMPL_BASE +
					  PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
}

void pm_turn_on_caches(void)
{
	if (cache_pg_state != PM_RAM_OFF)
		return;

	cache_pg_state = pm_ram_unpowergate(NV_ADDRESS_MAP_PMC_IMPL_BASE +
					    PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
}

enum pm_ram_pg_status get_cache_power_status(void)
{
	return cache_pg_state;
}

static void suspend_soc_drivers(void)
{
    if (tegra_hsp_suspend(&tegra_hsp_id_top1, &top_hsp_ctx)) {
        pr_error("Couldn't suspend HSP doorbells!");
    }
}

static void resume_soc_drivers(void)
{
    tegra_hsp_resume(&tegra_hsp_id_top1, &top_hsp_ctx);
}

void pm_suspend_sc7(void)
{
	suspend_soc_drivers();
	spe_clk_setup_cpu_nic(CLK_SRC_OSC_UNDIV);
	spe_clk_setup_apb(CLK_SRC_OSC_UNDIV);
}

void pm_resume_sc7(void)
{
	resume_soc_drivers();
	if (!pllaon_disabled) {
		disable_pllaon();
	}
	spe_clk_setup_cpu_nic(CLK_SRC_PLLP);
	spe_clk_setup_apb(CLK_SRC_PLLP);
}
