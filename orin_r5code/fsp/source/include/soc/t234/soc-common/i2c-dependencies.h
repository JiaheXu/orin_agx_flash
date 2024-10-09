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

#ifndef SOC_COMMON__I2C_DEPENDENCIES_H
#define SOC_COMMON__I2C_DEPENDENCIES_H
#define FSP__SOC_COMMON__I2C_DEPENDENCIES_H             1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>
#include <error/common-errors.h>

/* Module-specific FSP headers */
#include <processor/i2c-tegra-hw.h>
#include <soc-common/i2c-defs.h>

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
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__I2C_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEFS_H, "Header file missing or invalid.")

#ifdef I2C_CUSTOM_PORT_INIT
/* @brief Custom port init. Any FW specific OS implementations can be added in
 * this API.
 */
error_t appfw_port_init(uint32_t ctrl_id);
#else

/**
 * @brief Create Mutex lock.
 *
 * @jama_func_req_id 6914595
 *
 * @parma[in] ctrl_id         I2C Controller ID
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
error_t appfw_mutex_create(uint32_t ctrl_id);
#endif

/**
 * @brief Release Mutex lock.
 *
 * @jama_func_req_id 7879682
 *
 * @parma[in] ctrl_id         I2C Controller ID
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
error_t appfw_mutex_release(uint32_t ctrl_id);

/**
 * @brief Acquire Mutex lock.
 *
 * @jama_func_req_id 7879682
 *
 * @parma[in] ctrl_id         I2C Controller ID
 * @parma[in] timeout         timeout for the acquire operation.
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
error_t appfw_mutex_acquire(uint32_t ctrl_id, uint32_t timeout);

/**
 * @brief Wait I2C controller interrupt.
 *
 * @jama_func_req_id 6996948
 *
 * @parma[in] ctrl_id         I2C Controller ID
 * @parma[in] timeout         timeout in ms
 * @retval    E_SUCCESS       Succesfully received IRQ
 * @retval    E_TIMEOUT       IRQ not received within timeout
 */
error_t appfw_wait_irq(uint32_t ctrl_id, uint32_t timeout);

/**
 * @brief Get the current timer value in microseconds.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436, 6996745, 7882415
 *
 * @retval    64-bit             Current timer value
 */
uint64_t appfw_timer_cur_us(void);

/**
 * @brief Halt the task execution for specified microseconds.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436, 6996745, 7882415
 *
 * @param[in] delay           Time to halt the task execution.
 */
void appfw_udelay(uint32_t delay);

/**
 * @brief Assert, delay and deassert the reset signal.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436
 *
 * @param[in] ctrl_id         I2C Controller ID
 * @param[in] delay           Delay between assert and deassert
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
error_t appfw_clk_reset_pulse(uint32_t ctrl_id, uint32_t delay);

/**
 * @brief Configure the clock rate for the controller corresponding to the
 * controller ID passed. In case the FW has a clk_init before i2c_init,
 * clk_get_rate() can be returned. If the FW does not have a dedicated clk_init,
 * clk_set_rate() is to be returned.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436
 *
 * @param[in] ctrl_id         I2C Controller ID
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
int64_t appfw_configure_ctrl_clk_rate(uint32_t ctrl_id);

/**
 * @brief Enable the clk for the controller corresponding to the controller ID.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436, 6996745, 7882415, 7071357
 *
 * @param[in] ctrl_id         I2C Controller ID
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
error_t appfw_clk_enable(uint32_t ctrl_id);

/**
 * @brief Disable the clk for the controller corresponding to the controller ID.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436, 6996745, 7882415, 7879823
 *
 * @param[in] ctrl_id         I2C Controller ID
 * @retval    E_SUCCESS       Message complete signalled.
 * @retval    E_INVALID_PARAM Invalid ctrl_id.
 * @retval    FW_error_code   error from FW event signal api.
 */
error_t appfw_clk_disable(uint32_t ctrl_id);

/**
 * @brief Enable the irq corresponding to the controller id passed
 *
 * @jama_func_req_id 6996809, 9478454, 9452436
 *
 * @param[in] ctrl_id         I2C Controller ID
 */
void appfw_irq_enable(uint32_t ctrl_id);

/**
 * @brief Disable the irq corresponding to the controller id passed
 *
 * @jama_func_req_id 6996809, 9478454, 9452436
 *
 * @param[in] ctrl_id         I2C Controller ID
 */
void appfw_irq_disable(uint32_t ctrl_id);

/**
 * @brief Return the configuration data corresponding to the controller id passed.
 *
 * @jama_func_req_id 6914597
 *
 * @param[in] ctrl_id   I2C Controller ID
 * @retval    NULL         Invalid controller ID
 */
struct i2c_config_data *i2c_get_config_data(uint32_t ctrl_id);

/**
 * @brief Return the pointer to the i2c_tegra_handle structure corresponding to
 * the controller id passed.
 *
 * @jama_func_req_id 6914597
 *
 * @param[in] ctrl_id         I2C Controller ID
 * @retval    NULL            Invalid controller ID
 */
struct i2c_tegra_handle *i2c_get_tegra_handle(uint32_t ctrl_id);

#endif      /* I2C_DEPENDENCIES_H */
