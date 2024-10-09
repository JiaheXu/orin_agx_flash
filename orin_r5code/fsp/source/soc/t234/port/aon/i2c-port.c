/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION. All rights reserved.
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
#include <stdbool.h>                      // for bool, false, true
#include <stddef.h>                       // for NULL
#include <stdint.h>                       // for uint32_t, uint64_t, int64_t

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <irq/safe-irqs.h>
#include <osa/rtos-mutex.h>               // for rtosMutexAcquire, rtosMutex...
#include <osa/rtos-values.h>              // for rtosPASS
#include <clk/clk-tegra.h>                // for tegra_clk_disable, tegra_cl...
#include <delay/delay.h>                  // for udelay
#include <error/common-errors.h>          // for error_t, MODULE_ID_COMMON
#include <tke/tke-tegra.h>                // for tegra_tke_convert_usecs_to_...
#include <debug/assert.h>                 // for ASSERT, ...

/* Module-specific FSP header files */
#include <i2c/i2c-config.h>               // for i2c_config_data
#include <i2c/i2c-tegra.h>                // for i2c_config_data, i2c_tegra_...
#include <i2c/i2c.h>                      // for i2c_handle
#include <port/i2c-port.h>                // for VALID_I2C_IDS
#include <processor/i2c-tegra-hw.h>       // for i2c_get_hw_handle, i2c_hw_h...
#include <soc-common/i2c-dependencies.h>  // for is_valid_i2c_id, appfw_clk_...

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
CT_ASSERT(FSP__OSA__RTOS_MUTEX_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSA__RTOS_VALUES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DELAY__DELAY_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PORT__I2C_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__I2C_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEPENDENCIES_H, "Header file missing or invalid.")

struct i2c_config_data pi2c_config_data[2];
struct i2c_tegra_handle hi2c_tegra_all[2];
static bool irq_raised[2];
static rtosMutexHandle rwlock[2];
static uint8_t MutexBuffer[2][rtosMutexSize()];

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
    if (ret != E_SUCCESS) {
        goto out;
    }
    timer_start = appfw_timer_cur_us();
    do {
        if (irq_raised[(map_id(ctrl_id))]) {
            ret = E_SUCCESS;
            break;
        }
        timer_cur = appfw_timer_cur_us();
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
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
    if (ret == rtosPASS) {
        ret = E_SUCCESS;
    }

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
    if (ret == rtosPASS) {
        ret = E_SUCCESS;
    }

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

    ret = rtosMutexCreate(MutexBuffer[map_id(ctrl_id)], &rwlock[map_id(ctrl_id)]);
    if (ret == rtosPASS) {
        ret = E_SUCCESS;
    }

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

    ret = tegra_clk_set_rate(hi2c_hw->clk_data, config_data->ctrl_clk_rate);
    if (ret != 0) {
        ret = E_INVALID_PARAM;
    } else {
        ret = config_data->ctrl_clk_rate;
    }

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
