/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION. All rights reserved.
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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Hardware headers */

/* Late FSP headers */
#include <irq/safe-irqs.h>
#include <osa/rtos-mutex.h>
#include <osa/rtos-values.h>
#include <clk/clk-tegra.h>
#include <delay/delay.h>
#include <error/common-errors.h>
#include <misc/macros.h>
#include <tke/tke-tegra.h>
#include <debug/assert.h>

/* Module-specific FSP header files */
#include <i2c/i2c-config.h>
#include <i2c/i2c-tegra.h>
#include <i2c/i2c.h>
#include <port/i2c-port.h>
#include <processor/i2c-tegra-hw.h>
#include <soc-common/i2c-dependencies.h>

#define I2C_NUM_CTRL	2

struct i2c_config_data pi2c_config_data[I2C_NUM_CTRL];
struct i2c_tegra_handle hi2c_tegra_all[I2C_NUM_CTRL];
struct i2c_handle hi2c_all[I2C_NUM_CTRL];
static bool irq_raised[I2C_NUM_CTRL];
static rtosMutexHandle rwlock[I2C_NUM_CTRL];
static uint8_t MutexBuffer[I2C_NUM_CTRL][rtosMutexSize()];

error_t appfw_mutex_create(uint32_t ctrl_id);

/* VALID_I2C_IDS are defined in the cluster specific i2c-port.h file */
static inline bool is_valid_i2c_id(uint32_t ctrl_id)
{
    bool ret;

    if (ctrl_id < I2C_MAX_CTRLS) {
        ret = (BIT(ctrl_id) & VALID_I2C_IDS) > 0U;
    } else {
        ret = false;
    }

    return ret;
}

static uint32_t map_id(uint32_t ctrl_id)
{
	uint32_t ret;

	switch (ctrl_id) {
	case 2U:
		ret = 0U;
		break;
	case 8U:
		ret = 1U;
		break;
	default:
		/* Invalid Entry */
		ret = 2U;
		break;
	}

	return ret;
}

error_t appfw_wait_irq(uint32_t ctrl_id, uint32_t timeout_us)
{
	error_t ret = E_TIMEOUT;
	uint64_t timer_start;
	uint64_t timer_cur;
	const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = irq_safe_enable(hi2c_hw->irq_no);
	if (ret != E_SUCCESS)
		goto out;

	timer_start = appfw_timer_cur_us();
	do {
		if (irq_raised[(map_id(ctrl_id))]) {
			ret = E_SUCCESS;
			break;
		}
		timer_cur = appfw_timer_cur_us();
	} while ((timer_cur - timer_start) < timeout_us);

	irq_raised[map_id(ctrl_id)] = false;
	ret = irq_safe_disable(hi2c_hw->irq_no);

out:
	return ret;
}

error_t appfw_mutex_acquire(uint32_t ctrl_id, uint32_t timeout)
{
	error_t ret;

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = rtosMutexAcquire(rwlock[map_id(ctrl_id)],
		(uint32_t)tegra_tke_convert_usecs_to_ticks((const uint64_t)timeout));

	if (ret == rtosPASS)
		ret = E_SUCCESS;

out:
	return ret;
}

error_t appfw_mutex_release(uint32_t ctrl_id)
{
	error_t ret;

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = rtosMutexRelease(rwlock[map_id(ctrl_id)]);
	if (ret == rtosPASS)
		ret = E_SUCCESS;

out:
	return ret;
}

error_t appfw_mutex_create(uint32_t ctrl_id)
{
	error_t ret;

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = rtosMutexCreate(MutexBuffer[map_id(ctrl_id)],
			      &rwlock[map_id(ctrl_id)]);
	if (ret == rtosPASS)
		ret = E_SUCCESS;

out:
	return ret;
}

uint64_t appfw_timer_cur_us(void)
{
	return tegra_tke_get_usec();
}

void appfw_udelay(uint32_t delay)
{
	udelay(delay);
}

error_t appfw_clk_reset_pulse(uint32_t ctrl_id, uint32_t delay)
{
	error_t ret;
	const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = tegra_clk_reset_pulse(hi2c_hw->rst_data, delay);

out:
	return ret;
}

int64_t appfw_configure_ctrl_clk_rate(uint32_t ctrl_id)
{
	error_t ret;
	const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);
	struct i2c_config_data *config_data = i2c_get_config_data(ctrl_id);

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = tegra_clk_set_rate(hi2c_hw->clk_data,
				 config_data->ctrl_clk_rate);
	if (ret != 0)
		ret = E_INVALID_PARAM;
	else
		ret = config_data->ctrl_clk_rate;

out:
	return ret;
}

error_t appfw_clk_enable(uint32_t ctrl_id)
{
	error_t ret;
	const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = tegra_clk_enable(hi2c_hw->clk_data);

out:
	return ret;
}

error_t appfw_clk_disable(uint32_t ctrl_id)
{
	error_t ret;
	const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = tegra_clk_disable(hi2c_hw->clk_data);

out:
	return ret;
}

static void tegra_i2c_irq(void *id)
{
	struct i2c_hw_handle *hi2c_hw = id;
	uint32_t ctrl_id = hi2c_hw->ctrl_id;

	irq_raised[map_id(ctrl_id)] = true;
	tegra_i2c_irq_handler (ctrl_id);
}

void appfw_irq_enable(uint32_t ctrl_id)
{
    const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

    (void)irq_safe_enable(hi2c_hw->irq_no);
}

void appfw_irq_disable(uint32_t ctrl_id)
{
    const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

    (void)irq_safe_disable(hi2c_hw->irq_no);
}

struct i2c_tegra_handle *i2c_get_tegra_handle(uint32_t ctrl_id)
{
	struct i2c_tegra_handle *ret;

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = NULL;
		goto out;
	}

	hi2c_tegra_all[map_id(ctrl_id)].ctrl_id = ctrl_id;
	ret = &hi2c_tegra_all[map_id(ctrl_id)];

out:
	return ret;
}

error_t appfw_port_init(uint32_t ctrl_id)
{
	error_t ret;
	const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = E_INVALID_PARAM;
		goto out;
	}

	ret = irq_safe_set_handler(hi2c_hw->irq_no, tegra_i2c_irq,
				   (void *)hi2c_hw);
	if (ret != E_SUCCESS) {
		i2c_pr_error("%s: failed to set handler, ret - 0x%x\n", __func__,
						(int32_t)ret);
		goto out;
	}

	ret = appfw_mutex_create(ctrl_id);
	if (ret != E_SUCCESS) {
		i2c_pr_error("%s: failed to acquire mutex, ret - 0x%x\n", __func__,
						(int32_t)ret);
		goto out;
	}

out:
	return ret;
}

struct i2c_config_data *i2c_get_config_data(uint32_t ctrl_id)
{
	struct i2c_config_data *ret;

	if (!is_valid_i2c_id(ctrl_id)) {
		ret = NULL;
		goto out;
	}

	pi2c_config_data[map_id(ctrl_id)].bus_clk_rate = 100000U;
	pi2c_config_data[map_id(ctrl_id)].ctrl_clk_rate = 136000000U;
	pi2c_config_data[map_id(ctrl_id)].hs_master_code = 0U;
	ret = &pi2c_config_data[map_id(ctrl_id)];

out:
	return ret;
}
