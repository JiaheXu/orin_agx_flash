/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef I2C__I2C_H
#define I2C__I2C_H
#define FSP__I2C__I2C_H                                 1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */

/* Config based headers */
#ifdef CONFIG_I2C_ENABLE_PRINTS
#include <debug/debug-print.h>
#endif

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
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#ifdef CONFIG_I2C_ENABLE_PRINTS
#define i2c_pr_error dbg_printf
#define i2c_pr_info dbg_printf
#ifdef CONFIG_I2C_ENABLE_DEBUG
#define i2c_pr_debug dbg_printf
#else
#define i2c_pr_debug(...)
#endif
#else
#define i2c_pr_debug(...)
#define i2c_pr_info(...)
#define i2c_pr_error(...)
#endif

// IWYU pragma: no_forward_declare i2c_xfer_msg
struct i2c_handle;
struct i2c_xfer_msg;
struct i2c_config_data;

/**
 * @brief Contains all relevant data for an i2c controller.
 *
 * @ctrl_id         ID of the controller.
 * @i2c_hw_ops      Funcition pointers for HW operations
 * @i2c_hw_handle   Configuration data specific to the HW
 * @enabled         State of controller +
 * 					Enabled - True +
 * 					Disabled - False
 */
struct i2c_handle
{
    uint32_t ctrl_id;
    struct i2c_hw_ops *hw_ops;
    struct i2c_tegra_handle *hi2c_tegra;
    bool pre_init_done;
    bool enabled;
};

/**
 * @brief Initialize OS resources for I2C controller.
 *
 * This function initializes the OS resources i2c driver requires.
 * Sets pre_init_done to true upon successful pre-init.
 *
 * @jama_func_req_id 6914595
 *
 * @param[in]  ctrl_id          Controller ID which need to be initialized.
 * @param[out] hi2c             Pointer to I2C handle structure where I2C
 *                              handle is saved.
 * @retval E_SUCCESS            Initialization of controller successfully done.
 * @retval E_INVALID_PARAM      Input parameter is passed to the function.
 * @retval E_INVALID_CONFIG     Null configuration data in i2c-port.c.
 **/
error_t i2c_controller_pre_init(uint32_t ctrl_id,
                            struct i2c_handle *hi2c);

/**
 * @brief Initialize a controller with static configuration and controller ID.
 *
 * This function initializes the controller with static bus and controller
 * configuration and controller ID. The list of controller IDs corresponding to
 * the available HW controllers are given in soc-common/i2c-defs.h.
 * Calls i2c_controller_pre_init internally if pre_init_done is not true.
 *
 * @jama_func_req_id 6914595
 *
 * @param[in]  ctrl_id          Controller ID which need to be initialized.
 * @param[out] hi2c             Pointer to I2C handle structure where I2C
 *                              handle is saved.
 * @retval E_SUCCESS            Initialization of controller successfully done.
 * @retval E_INVALID_PARAM      Input parameter is passed to the function.
 * @retval E_I2C_INVALID_OPS    HW specific operation is invalid.
 * @retval E_INVALID_CONFIG     Null configuration data in i2c-port.c.
 * @retval E_TIMEOUT            Possible reasons: +
 *                                 * Flush Fifos failed due to timeout. +
 *                                 * Failed to load config register.
 **/
error_t i2c_controller_init(uint32_t ctrl_id,
                            struct i2c_handle *hi2c);

/**
 * @brief Write data into the register of slave device.
 *
 * Write the data into a register of slave device. If more than one byte is
 * written then register address is automatically incremented.
 *
 * @jama_func_req_id 10516980
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @param[in] slave_address The slave device address to communicate with. This
 *                          can be 7-bit or 10-bit.
 * @param[in] reg_address   The register address in the slave device.
 * @param[in] flags         Transfer flags. Multiple transfer flags can be ORed
 *                          here.
 * @param[in] pdata         Pointer to data to written to the slave device.
 * @param[in] num_data      Number of bytes to be write.
 * @param[in] timeout       Maximum time to be allow to complete the transfer.
 *                          Unit of timeout is in Microseconds.
 *                          If data transfer is not completed in this time then
 *                          data transfer is aborted and returned to the caller
 *                          as E_TIMEOUT. If client can wait forever, 0 is to be
 *                          passed.
 * @retval E_SUCCESS        Data write into the device is successfully
 *                          completed.
 * @retval E_TIMEOUT        Data transfer is not completed in given time and the
 *                          transfer is aborted. In this case, the function
 *                          cleans the residual FIFOs and gets ready for the
 *                          next transfer.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_acquire in port layer.
 * @retval CUSTOM_RETURN    Return value from xfer routine in i2c_hw_ops.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_release in port layer.
 */
error_t i2c_reg_write_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                           uint8_t reg_address, uint16_t flags, uint8_t const *pdata,
                           uint16_t num_data, uint32_t timeout);

/**
 * @brief Read data from a register of slave device.
 *
 * @jama_func_req_id 10516944
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * Read data from the given register of slave device. If more than one byte is
 * read then register address is automatically incremented.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @param[in] slave_address The slave device address to communicate with. This
 *                          can be 7-bit or 10-bit.
 * @param[in] reg_address   The register address in the slave device.
 * @param[in] flags         Transfer flags. Multiple transfer flags can be ORed
 *                          here.
 * @param[in] pdata         Pointer to data buffer where read data is to be
 *                          stored.
 * @param[in] num_data      Number of bytes to be read.
 * @param[in] timeout       Maximum time to be allow to complete the transfer.
 *                          Unit of timeout is in Microseconds.
 *                          If data transfer is not completed in this time then
 *                          data transfer is aborted and returned to the caller
 *                          as E_TIMEOUT. If client can wait forever, 0 is to be
 *                          passed.
 * @retval E_SUCCESS        Data read into the device is successfully
 *                          completed.
 * @retval E_TIMEOUT        Data transfer is not completed in given time and the
 *                          transfer is aborted. In this case, the function
 *                          cleans the residual FIFOs and gets ready for the
 *                          next transfer.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_acquire in port layer.
 * @retval CUSTOM_RETURN    Return value from xfer routine in i2c_hw_ops.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_release in port layer.
 */
error_t i2c_reg_read_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                          uint8_t reg_address, uint16_t flags, uint8_t *pdata,
                          uint16_t num_data, uint32_t timeout);

/**
 * @brief Send the data to I2C bus.
 *
 * This function sends the data onto the I2C bus to a slave device.
 *
 * @jama_func_req_id 10517040
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @param[in] slave_address The slave device address to communicate with. This
 *                          can be 7-bit or 10-bit.
 * @param[in] flags         Transfer flags. Multiple transfer flags can be ORed
 *                          here.
 * @param[in] pdata         Pointer to data buffer which need to be sent to the
 *                          device.
 * @param[in] num_data      Number of bytes to be sent.
 * @param[in] timeout       Maximum time to be allow to complete the transfer.
 *                          Unit of timeout is in Microseconds.
 *                          If data transfer is not completed in this time then
 *                          data transfer is aborted and returned to the caller
 *                          as E_TIMEOUT. If client can wait forever, 0 is to be
 *                          passed.
 * @retval E_SUCCESS        Sending of data to the bus is successful.
 * @retval E_TIMEOUT        Data transfer is not completed in given time and the
 *                          transfer is aborted. In this case, the function
 *                          cleans the residual FIFOs and gets ready for the
 *                          next transfer.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_acquire in port layer.
 * @retval CUSTOM_RETURN    Return value from xfer routine in i2c_hw_ops.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_release in port layer.
 */
error_t i2c_send_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                      uint16_t flags, const uint8_t *pdata, uint32_t num_data,
                      uint32_t timeout);

/**
 * @brief Receive data from I2C bus.
 *
 * This function reads the data on the I2C bus from a slave device.
 *
 * @jama_func_req_id 10517019
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @param[in] slave_address The slave device address to communicate with. This
 *                          can be 7-bit or 10-bit.
 * @param[in] flags         Transfer flags. Multiple transfer flags can be ORed
 *                          here.
 * @param[in] pdata         Pointer to data buffer where receive data is to be
 *                          saved.
 * @param[in] num_data      Number of bytes to be receive.
 * @param[in] timeout       Maximum time to be allow to complete the transfer.
 *                          Unit of timeout is in Microseconds.
 *                          If data transfer is not completed in this time then
 *                          data transfer is aborted and returned to the caller
 *                          as E_TIMEOUT. If client can wait forever, 0 is to be
 *                          passed.
 * @retval E_SUCCESS        Receiving of data to the bus is successfully
 *                          completed.
 * @retval E_TIMEOUT        Data transfer is not completed in given time and the
 *                          transfer is aborted. In this case, the function
 *                          cleans the residual FIFOs and gets ready for the
 *                          next transfer.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_acquire in port layer.
 * @retval CUSTOM_RETURN    Return value from xfer routine in i2c_hw_ops.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_release in port layer.
 */
error_t i2c_receive_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                         uint16_t flags, uint8_t *pdata, uint32_t num_data,
                         uint32_t timeout);

/**
 * @brief Data transfer(read or write or both) over the I2C bus with a slave
 * device
 *
 * This function implements the data transfer, read or write or combination of
 * both over the I2C bus with a slave device. Data for multiple byte transfer
 * can be provided as an array of messages. The repeat-start of data transfer is
 * used between transfers. If flag M_NO_START is provided then start will not be
 * sent again.
 *
 * @jama_func_req_id 7879682
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @param[in] msgs          Pointer to I2C messages containing the list of
 *                          message to be transferred.
 * @param[in] num_msgs      Number of messages in the msgs parameter.
 * @param[in] timeout       Maximum time to be allow to complete the transfer.
 *                          Unit of timeout is in Microseconds.
 *                          If data transfer is not completed in this time then
 *                          data transfer is aborted and returned to the caller
 *                          as E_TIMEOUT. If client can wait forever, 0 is to be
 *                          passed.
 * @retval E_SUCCESS        Data transfer to the bus is successfully completed.
 * @retval E_TIMEOUT        Data transfer is not completed in given time and the
 *                          transfer is aborted. In this case, the function
 *                          cleans the residual FIFOs and gets ready for the
 *                          next transfer.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_acquire in port layer.
 * @retval CUSTOM_RETURN    Return value from xfer routine in i2c_hw_ops.
 * @retval CUSTOM_RETURN    Return value from appfw_mutex_release in port layer.
 */
error_t i2c_do_transfer(const struct i2c_handle *hi2c,
                        struct i2c_xfer_msg *msgs,
                        uint32_t num_msgs, uint32_t timeout);

/**
 * @brief Suspend the I2C bus and controller.
 *
 * Suspend the I2C bus and controller. After this call, no data transfer is
 * possible on the bus untill it is resumed by calling i2c_resume().
 *
 * @jama_func_req_id 6914608
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @retval E_SUCCESS        Bus and controller are successfully suspended for
 *                          any further transfers.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval E_INVALID_OPS    Invalid suspend hw_ops is assigned in tegra handle.
 * @retval E_INVALID_STATE  Bus is already in the suspended state.
 * @retval E_I2C_IO         IO error happened during bus suspend.
 */
error_t i2c_suspend(const struct i2c_handle *hi2c);

/**
 * @brief Resume the I2C bus and controller.
 *
 * Resume the I2C bus and controller. After this call, the bus(which was
 * suspended after call i2c_suspend) is resumed for new data transfers.
 *
 * @jama_func_req_id 6914601
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 * Also the function i2c_suspend() must be called for the bus to be in suspended
 * state.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @retval E_SUCCESS        Bus and controller are successfully resumed for
 *                          further transfers.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function. +
 *                          T194 - allowed size of transfer is 125 words.
 * @retval E_INVALID_OPS    Invalid resume hw_ops is assigned in tegra handle.
 * @retval E_INVALID_STATE  Bus is not in suspended state and trying to resume.
 * @retval E_I2C_IO         IO error happened during bus resume.
 */
error_t i2c_resume(const struct i2c_handle *hi2c);

/**
 * @brief Get the state of I2C bus HW.
 *
 * Return if the SCL, SDA or BUS is busy or not.
 *
 * @jama_fun_req_id 6914597
 *
 * @pre The function i2c_controller_init() must be called before this function.
 * Else, hi2c->hwops structure is not initialized and api returns
 * E_INVALID_OPS.
 *
 * @param[in] hi2c          Pointer to I2C handle which is returned from
 *                          i2c_controller_init().
 * @param[out] state        refer "I2C HW bus status" macros for the return values.
 * @retval E_INVALID_PARAM  Invalid parameter is passed to the function.
 * @retval E_INVALID_OPS    Invalid resume hw_ops is assigned in tegra handle.
 * @retval E_SUCCESS        Bus HW status is successfully read.
 */
error_t i2c_get_bus_state(const struct i2c_handle *hi2c, uint32_t *state);

/**
 * @brief Defines to interpret the bus status return from i2c_get_bus_state api.
 *
 * @macro-title I2C HW bus status
 *
 * @I2C_SCL_BUSY                    Bit representation for I2C SCL is busy
 * @I2C_SDA_BUSY                    Bit representation for I2C SDA is busy
 * @I2C_BUS_BUSY                    Bit representation for I2C BUS is busy
 * @I2C_IS_SCL_HIGH                 parse the return values from i2c_get_bus_state
 *                                  api and return true if SCL is high.
 * @I2C_IS_SDA_HIGH                 parse the return values from i2c_get_bus_state
 *                                  api and return true if SDA is high.
 * @I2C_IS_BUS_FREE                 parse the return values from i2c_get_bus_state
 *                                  api and return true if bus is free to use.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#define I2C_SCL_BUSY                BIT(0)
#define I2C_SDA_BUSY                BIT(1)
#define I2C_BUS_BUSY                BIT(2)
#define I2C_IS_SCL_HIGH(status)     ((status & I2C_SCL_BUSY) != 0U)
#define I2C_IS_SDA_HIGH(status)     ((status & I2C_SDA_BUSY) != 0U)
#define I2C_IS_BUS_FREE(status)     ((status & I2C_BUS_BUSY) != 0U)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
