/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
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

#include <port/i2c-port.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <dt.h>
#include <bpmp/time.h>
#include <bpmp/irq.h>
#include <clk.h>

#include <bpmp/arch/arm.h>

#include <reset.h>
#include <i2c/i2c-tegra.h>
#include <processor/i2c-tegra-hw.h>
#include <soc-common/clk-tegra-hw.h>


static struct i2c_config_data pi2c_config_data[2];
struct thread threaded_irq_i2c;
static struct event msg_complete[I2C_MAX_CTRLS];

static inline uint32_t map_id(uint32_t ctrl_id)
{
    uint32_t ret;

    switch (ctrl_id) {
    case TEGRA194_I2C5:
        ret = 0;
    case TEGRA194_I2C10:
        ret = 1;
    default:
        ret = 2; /* Invalid Entry */

    return ret;
}

bpmp_irq_ret_t bpmp_socpwri2c_irq_handler(const bpmp_irq_no_t irq_no)
{
    bpmp_irq_disable(irq_no);

    wakeup_irq_thread(&threaded_irq_i2c, TEGRA194_I2C5);

    return BPMP_IRQ_RESCHEDULE;
}

error_t appfw_event_signal(uint32_t ctrl_id)
{
    error_t ret = E_SUCCESS;

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    event_signal(&msg_complete[ctrl_id], false);

out:
    return ret;
}

error_t appfw_event_unsignal(uint32_t ctrl_id)
{
    error_t ret = E_SUCCESS;

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    event_unsignal(&msg_complete[ctrl_id]);

out:
    return ret;
}

error_t appfw_event_init(uint32_t ctrl_id)
{
    error_t ret = E_SUCCESS;

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    event_init(&msg_complete[ctrl_id], false, 0);

out:
    return ret;
}

error_t appfw_event_wait(uint32_t ctrl_id, uint32_t timeout)
{
    error_t ret = E_SUCCESS;

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    ret = event_wait_timeout(&msg_complete[ctrl_id], timeout);

out:
    return ret;
}

#ifdef THREADED_IRQS
void appfw_wake_irq_thread(uint32_t ctrl_id)
{
    UNUSED(ctrl_id);
    return;
}
#endif

uint64_t appfw_timer_cur_us(void)
{
    return bpmp_time_monotonic_us64();
}

void appfw_udelay(uint32_t delay)
{
    udelay(delay);
}

void appfw_irq_init(uint32_t ctrl_id)
{
    static bool once = true;
    const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);
    static BPMP_ARCH_THREAD_STACK_DEFINE(thread_irq_stack_i2c, 1024);

    if (!is_valid_i2c_id(ctrl_id)) {
        goto out;
    }

    if (bpmp_irq_is_valid(hi2c_hw->irq_no)) {
        if (once) {
            create_threaded_irq_handler(tegra_i2c_irq_handler,
                        &threaded_irq_i2c,
                        thread_irq_stack_i2c,
                        BPMP_ARCH_THREAD_STACK_SIZEOF(thread_irq_stack_i2c));
            once = false;
        }
        bpmp_irq_enable(BPMP_INT_SOCPWRI2C);
    }
    UNUSED(ctrl_id);

out:
    return;
}

void appfw_irq_enable(uint32_t ctrl_id)
{
    if (!is_valid_i2c_id(ctrl_id)) {
        goto out;
    }

    bpmp_irq_enable(BPMP_INT_SOCPWRI2C);

out:
    return;
}

error_t appfw_clk_reset_pulse(uint32_t ctrl_id, uint32_t delay)
{
    error_t ret = E_SUCCESS;
    const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    reset_module(hi2c_hw->rst_id);
    UNUSED(delay);

out:
    return ret;
}

int64_t appfw_configure_ctrl_clk_rate(uint32_t ctrl_id)
{
    int64_t ret;
    const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);
    struct i2c_config_data *config_data = i2c_get_config_data(ctrl_id);

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    ret = clock_set_rate(hi2c_hw->clk_id, config_data->ctrl_clk_rate);

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

    ret = clock_enable(hi2c_hw->clk_id);

out:
    return ret;
}

error_t appfw_clk_disable(uint32_t ctrl_id)
{
    error_t ret = E_SUCCESS;
    const struct i2c_hw_handle *hi2c_hw = i2c_get_hw_handle(ctrl_id);

    if (!is_valid_i2c_id(ctrl_id)) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    clock_disable(hi2c_hw->clk_id);

out:
    return ret;
}

struct i2c_config_data *i2c_get_config_data(uint32_t ctrl_id)
{
    struct i2c_config_data *config_data = NULL;
    if (!is_valid_i2c_id(ctrl_id)) {
        goto out;
    }

    /* TODO: add dt parsing api to update pi2c_config_data[map_id(ctrl_id)] */
    config_data = &pi2c_config_data[map_id(ctrl_id)];

out:
    return config_data;
}
