/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef I2C__I2C_TEGRA_H
#define I2C__I2C_TEGRA_H
#define FSP__I2C__I2C_TEGRA_H                           1

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

/**
 * @brief Defines for I2C transfer flags constants.
 *
 * @macro-title SW Defined I2C Transfer flags
 *
 * @I2C_XFER_FLAG_RD            This flag tells that given transfer is read type
 *                              i.e master reads data from slave.
 * @I2C_XFER_FLAG_TEN           This flag tells that slave address is 10-bit
 *                              address.
 * @I2C_XFER_FLAG_NOSTART       This flag tells that data transfer happens without
 *                              repeat start.
 * @I2C_XFER_FLAG_IGNORE_NAK    This flag tells that transfer will continue even if
 *                              there no ACK from slave.
 */
#define I2C_XFER_FLAG_RD                            BIT(0)
#define I2C_XFER_FLAG_TEN                           BIT(1)
#define I2C_XFER_FLAG_NOSTART                       BIT(2)
#define I2C_XFER_FLAG_IGNORE_NAK                    BIT(3)

#define I2C_PKT_XFER_MAX_BYTES                      500U

// IWYU pragma: no_forward_declare i2c_hw_ops
struct i2c_tegra_handle;
struct i2c_hw_ops;

/**
 * @brief I2C transfer message structure contains the message information -
 * slave address, transfer flags, message data buffer and message length.
 *
 * @dev_addr   Slave address. It can be 7-bit or 10-bit address.
 * @xfer_flags Transfer flags. Multiple transfer flags can be combined with OR
 *             operation. Check - SW Defined I2C Transfer flags section.
 * @buf_len    Length of the message. +
 *              Valid range - <1, 512>
 * @prbuf      Pointer to read message buffer.
 * @pwbuf      Pointer to write message buffer.
 **/
struct i2c_xfer_msg
{
    uint16_t dev_addr;
    uint16_t xfer_flags;
    uint16_t buf_len;
    uint8_t *prbuf;
    const uint8_t *pwbuf;
};

/**
 * @brief tegra specific controller data
 *
 * @ctrl_id             Controller id
 * @hi2c_hw             HW specific data which is considered const throughout
 *                      the unit.
 * @pi2c_config         Configurable parameters(Clock and HW timing).
 * @is_suspended        suspended state - True; Resumed - False
 * @is_initialized      initialized state - True; Uninitialized - False
 */
struct i2c_tegra_handle
{
    uint32_t ctrl_id;
    const struct i2c_hw_handle *hi2c_hw;
    const struct i2c_config_data *pi2c_config;
    bool pre_init_done;
    /* used at runtime */
    bool is_suspended;
    bool is_initialized;
    struct i2c_xfer_msg *cur_msg;
    uint32_t bytes_to_xfer;
    error_t isr_ret;
    bool end_xfer;
    uint32_t rx_index;
    uint32_t tx_index;
};

/**
 * @brief Initialize Tegra I2C controller OS Resources.
 *
 * This function implements the Tegra I2C controller specific OS resources
 * initialization. This function gets the configuration data from the static
 * data in i2c-port.c. If config data is null, corresponding error is returned.
 * Sets pre_init_done to true upon successful pre-init.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436
 *
 * @param[in]  ctrl_id          I2C Controller ID which need to be initialized.
 *                              The client needs to get the controller ID of
 *                              required I2C bus from i2c-tegra.c.
 * @param[out] hi2c_tegra       Pointer of tegra I2C handle structure where
 *                              tegra specific parameters for an i2c controller
 *                              are stored.
 * @param[out] hw_ops           Pointer to Pointer of structure of I2C HW
 *                              operations. The HW specific controller driver
 *                              needs to update this structure. I2C core will
 *                              use these HW functions to access the bus and
 *                              configure the controller.
 * @retval E_SUCCESS            Initialization of controller is successful
 * @retval E_INVALID_CONFIG     Null configuration data in i2c-port.c.
 * @retval E_INVALID_PARAM      Input parameter is invalid.
 */
error_t tegra_i2c_pre_init(uint32_t ctrl_id, struct i2c_tegra_handle *hi2c_tegra);

/**
 * @brief Initialize Tegra I2C controller
 *
 * This function implements the Tegra I2C controller specific initialization.
 * This function gets the configuration data from the static data in i2c-port.c
 * If config data is null, corresponding error is returned.
 * Calls tegra_i2c_pre_init internally if pre_init_done not true.
 *
 * @jama_func_req_id 6996809, 9478454, 9452436
 *
 * @param[in]  ctrl_id          I2C Controller ID which need to be initialized.
 *                              The client needs to get the controller ID of
 *                              required I2C bus from i2c-tegra.c.
 * @param[out] hi2c_tegra       Pointer of tegra I2C handle structure where
 *                              tegra specific parameters for an i2c controller
 *                              are stored.
 * @param[out] hw_ops           Pointer to Pointer of structure of I2C HW
 *                              operations. The HW specific controller driver
 *                              needs to update this structure. I2C core will
 *                              use these HW functions to access the bus and
 *                              configure the controller.
 * @retval E_SUCCESS            Initialization of controller is successful
 * @retval E_INVALID_CONFIG     Null configuration data in i2c-port.c.
 * @retval E_INVALID_PARAM      Input parameter is invalid.
 * @retval E_TIMEOUT            Possible reasons: +
 *                              * Flush Fifos failed due to timeout. +
 *                              * Failed to load config register.
 */
error_t tegra_i2c_init(uint32_t ctrl_id, struct i2c_tegra_handle *hi2c_tegra,
                   struct i2c_hw_ops **hw_ops);

/**
 * @brief Interrupt handler for the I2C controller
 *
 * This function is used to implement the I2C interrupt handler. This function
 * is called from the R5 ISR handler when I2C controller raises an interrupt.
 * In case the FW uses threaded IRQs feature, THREADED_IRQS feature needs to
 * defined. The FW needs to define the thread supposed to woken in case of the
 * interrupt.
 * The I2C controller raises the interrupt for the following reasons:
 *
 * - Space in Tx FIFO
 * - Data available in Rx FIFO
 * - Current Packet transfer completed.
 * - All Packet transfer completed.
 * - All Packets transfer Completed
 *
 * @jama_func_req_id 6996948
 *
 * @param[in] ctrl_id Controller Id for which the interrupt is raised. This is
 *                    provided when interrupt is setup.
 */
void tegra_i2c_irq_handler(uint32_t ctrl_id);

#endif        /* FSP_I2C_I2C_TEGRA_H */
