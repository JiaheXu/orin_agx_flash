/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef I2C__I2C_HW_OPS_H
#define I2C__I2C_HW_OPS_H
#define FSP__I2C__I2C_HW_OPS_H                          1

/* Compiler headers */
#include <stdint.h>                    // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>            // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>       // for error_t, FSP__ERROR__COMMON_ERRORS_H
#include <misc/macros.h>               // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <i2c/i2c-tegra.h>             // for i2c_tegra_handle, FSP__I2C__I2C_TEGRA_H

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Do data transfer over the I2C bus.
 *
 * Function pointer to the function which implements the data transfer over the
 * bus. The data to be transfer is passed as a parameter to this function.
 *
 * @pre tegra_i2c_init() must be called before calling this function. Else,
 * hwops structure is not initialized and api returns E_INVALID_OPS.
 *
 * @jama_func_req_id 6996745, 7882415
 *
 * @param[in] hi2c_tegra    Pointer to the HW I2C handle which is returned from
 *                          tegra_i2c_init().
 * @param[in] msgs          Pointer to I2C messages containing the list of
 *                          messages to be transferred.
 * @param[in] num_msgs      Number of messages in the msgs parameter.
 * @param[in] timeout       Maximum time to be allow to complete the transfer in
 *                          microseconds.
 *                          If data transfer is not completed in this stipulated
 *                          time, data transfer is aborted and returned to the
 *                          caller as E_TIMEOUT. If client can wait forever, 0
 *                          is to be passed.
 * @retval E_SUCCESS        Data transfer on the bus is successfully completed.
 * @retval E_INVALID_STATE  Data transfer failed as bus is in suspended state.
 * @retval CUSTOM_RETURN    Return value from appfw_clk_enable in port layer.
 * @retval CUSTOM_RETURN    Return value from appfw_clk_reset_pulse in port layer.
 * @retval CUSTOM_RETURN    Return value from appfw_configure_ctrl_clk_rate in
 *                          port layer.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval E_TIMEOUT        Possible reasons: +
 *                          * Data transfer is not completed in given time. +
 *                          * Flush Fifos failed due to timeout. +
 *                          * Failed to load config register. +
 *                          Transfer is aborted. In this case, the function
 *                          cleans the residuals FIFOs and gets ready for next
 *                          transfer.
 * @retval E_I2C_ARB_LOST   Arbitration error happened during data transfer on
 *                          bus and transfer aborted.
 * @retval E_I2C_NACK       Device has not acknowledged the transfer.
 * @retval E_I2C_IO         IO error happened during data transfer.
 * @retval E_I2C_TX_FIFO_OVERFLOW HW Transfer fifo overflow during the transfer.
 * @retval E_I2C_RX_FIFO_UNDERFLOW HW Receive fifo underflow during the transfer.
 * @retval CUSTOM_RETURN    Return value from appfw_clk_disable in port layer.
 */
typedef error_t (*i2c_hw_xfer)(struct i2c_tegra_handle *hi2c_tegra,
                               struct i2c_xfer_msg *msgs,
                               uint32_t num_msgs, uint32_t timeout);

/**
 * @brief Suspend the I2C bus and controller.
 *
 * Suspend the I2C bus and controller. After this call, no data transfer is
 * possible on the bus until it is resumed by calling i2c_hw_resume().
 *
 * @pre tegra_i2c_init() must be called before calling this function. Else,
 * hwops structure is not initialized and api returns E_INVALID_OPS.
 *
 * @jama_func_req_id 7879823
 *
 * @param[in] hi2c_tegra    Pointer to HW I2C handle which is returned from
 *                          tegra_i2c_init().
 * @retval E_SUCCESS        Bus and controller are successfully suspended for
 *                          any further transfers.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function.
 * @retval E_INVALID_STATE  Bus is already in the suspended state.
 * @retval E_I2C_IO         IO error happened during bus suspend.
 */
typedef error_t (*i2c_hw_suspend)(struct i2c_tegra_handle *hi2c_tegra);

/**
 * @brief Resume the I2C bus and controller.
 *
 * Resume the I2C bus and controller. After this call, the bus is resumed for
 * new data transfer which was suspended after call i2c_hw_suspend().
 *
 * @pre The function tegra_i2c_init() must be called before this function. Else,
 * hwops structure is not initialized and api returns E_INVALID_OPS. Also the
 * function i2c_hw_suspend() must be called for the bus to be in suspended state.
 *
 * @jama_func_req_id 7071357
 *
 * @param[in] hi2c_hw       Pointer to I2C handle which is returned from
 *                          tegra_i2c_init().
 * @retval E_SUCCESS        Bus and controller are successfully resumed for
 *                          further transfers.
 * @retval E_INVALID_PARAM  Invalid handle is passed to the function.
 * @retval E_INVALID_STATE  Bus is not in suspended state and trying to resume.
 * @retval E_I2C_IO         IO error happened during bus resume.
 */
typedef error_t (*i2c_hw_resume)(struct i2c_tegra_handle *hi2c_tegra);

/**
 * @brief Get the state of I2C bus HW.
 *
 * Return if the SCL, SDA or BUS is busy or not.
 *
 * @jama_fun_req_id 16052696
 *
 * @pre tegra_i2c_init() must be called before calling this function. Else,
 * hwops structure is not initialized and api returns E_INVALID_OPS.
 *
 * @param[in] hi2c_tegra    Pointer to HW I2C handle which is returned from
 *                          tegra_i2c_init().
 * @param[out] status       refer "I2C HW bus status" macros for the return values.
 * @retval E_SUCCESS        Bus HW status is successfully read.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval E_INVALID_OPS    Invalid resume hw_ops is assigned in tegra handle.
 */
typedef error_t (*i2c_get_bus_status)(const struct i2c_tegra_handle *hi2c_tegra,
                        uint32_t *state);

/**
 * @brief Structure for hw specific functions.
 *
 * This structure contains the function pointers for HW specific funtions. The
 * functions are implemented in HW specific I2C driver and the APIs are exported
 * via this structure. The client of this driver needs to use the HW ops for the
 * required functionalities.
 *
 * @get_config_data Function pointer to get the I2C config data from HW
 *                  specific drivers. The HW specific driver returns the pointer
 *                  to config data structure used during the initialization.
 * @xfer            Function pointer to do data transfer on I2C bus.
 * @suspend         Function pointer to suspend the controller and bus. The
 *                  client of HW driver calls this function to suspend the
 *                  controller and stop further transfers until it is resumed.
 * @resume          Function pointer to resume the controller and bus. The
 *                  client of HW driver calls this function to resume the
 *                  controller and start data transfers on the bus.
 */
struct i2c_hw_ops
{
    i2c_hw_xfer            xfer;
    i2c_hw_suspend         suspend;
    i2c_hw_resume          resume;
    i2c_get_bus_status     get_bus_state;
};

#endif      /* I2C__I2C_HW_OPS_H */
