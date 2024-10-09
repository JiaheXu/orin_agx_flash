/*
 * Copyright (c) 2021-2022 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef SBSA_UART_H
#define SBSA_UART_H
#define FSP__UART__SBSA_UART_H        1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <uart/uart.h>

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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__UART__UART_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Initialize the already initialized UART instance
 *
 * This function initializes and configures the UART context for an instance
 * that has already been initialized by a 3rdparty or another CPU.
 *
 * @param[in] ctrl      UART instance.
 * @param[in] conf      UART configuration parameters.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      NULL paramters passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 */
error_t sbsa_uart_3rdparty_init(struct tegra_uart_ctlr *ctlr,
                                const struct tegra_uart_conf * const conf);

/**
 * @brief Initialize the UART instance and configure it
 *
 * This function initializes and configures the UART context for an instance.
 *
 * @param[in] ctrl      UART instance.
 * @param[in] conf      UART configuration parameters.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      NULL paramters passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 * @retval E_UART_ERR_CONFIG    invalid configuration
 */
error_t sbsa_uart_init(struct tegra_uart_ctlr * const ctlr,
                       const struct tegra_uart_conf * const conf);
/**
 * @brief Deinitialize the UART instance and configure it
 *
 * This function deinitializes instance.
 *
 * @param[in] ctrl      UART instance.
 * @retval E_SUCCESS                    indicates success
 * @retval E_UART_NULL_PTR              NULL paramters passed
 * @E_CAR_CLOCK_OP_NOT_SUPPORTED        Error value returned if the clock does not support the operation.
 * @E_CAR_FMON_HW_LOGIC                 Error value returned if the FMON register logic algorithm failed.

 *
 */
error_t sbsa_uart_deinit(struct tegra_uart_ctlr * const ctlr);

/**
 * @brief Initialize the UART instance and configure it
 *
 * This function initializes and configures the UART context for an instance.
 *
 * @pre the function sbsaa_uart_init() has been called
 *
 * @param[in] ctrl      UART instance.
 * @param[in] conf      UART configuration parameters.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      NULL paramters passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 * @retval E_UART_ERR_CONFIG    error in configuring the UART
 */
error_t sbsa_uart_init_hw(struct tegra_uart_ctlr * const ctlr,
                          const struct tegra_uart_conf * const conf);

/**
 * @brief Flush the UART HW TX FIFO.
 *
 * This function flushes the software HW TX FIFO of the UART.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 * @retval E_UART_FLUSH_TIMEOUT flush timed out
 */
error_t sbsa_uart_flush_tx_hw_fifo(const struct tegra_uart_ctlr * const ctlr);

/**
 * @brief Flush the UART SW TX buffer.
 *
 * This function flushes the software TX buffer of the UART.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] timeout   timeout to flush the buffer.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 * @retval E_UART_FLUSH_TIMEOUT flush timed out
 */
error_t sbsa_uart_flush_tx_fifo(const struct tegra_uart_ctlr * const ctlr);

/**
 * @brief Write a character to the UART HW TX FIFO.
 *
 * This function writes a character to the UART HW TX FIFO.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] c         character to be written.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 */
error_t sbsa_uart_write_char(const struct tegra_uart_ctlr * const ctlr,
                             const char c);

/**
 * @brief Read a character from the UART HW RX FIFO.
 *
 * This function reads a character from the UART HW RX FIFO.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] c         pointer to where character read is to be written.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 */
error_t sbsa_uart_read_char(const struct tegra_uart_ctlr * const ctlr,
                            char *c);

/**
 * @brief Read from the UART RX buffer.
 *
 * This function reads data from the UART RX buffer.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] buf       pointer to store the data being read.
 * @param[in] count     byte count of the data to be read.
 * @param[in] read      byte count read.
 * @param[in] timeout   timeout to return.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 * @retval E_UART_WRITE_TIMEOUT flush timed out
 */
error_t sbsa_uart_read(const struct tegra_uart_ctlr * const ctlr,
                       char *buf,
                       uint32_t count,
                       uint32_t *read,
                       uint64_t timeout);

/**
 * @brief Write to the UART TX buffer.
 *
 * This function writes the data to the SW TX buffer which in turn gets
 * pushed to the HW TX FIFO.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] buf       pointer to the data to be written.
 * @param[in] count     byte count of the data to be written.
 * @param[in] written   byte count written.
 * @param[in] timeout   timeout to return.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 * @retval E_UART_WRITE_TIMEOUT flush timed out
 */
error_t sbsa_uart_write(const struct tegra_uart_ctlr * const ctlr,
                        const char *buf,
                        uint32_t count,
                        uint32_t *written,
                        uint64_t timeout);

/**
 * @brief Write to the UART HW TX fifo.
 *
 * This function writes data to the UART HW TX FIFO.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] buf       pointer to the data to be written.
 * @param[in] count     byte count of the data to be written.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 */
error_t sbsa_uart_write_now(const struct tegra_uart_ctlr * const ctlr,
                            const char *buf,
                            uint32_t count);

/**
 * @brief Write a string to the UART TX buffer.
 *
 * This function writes a string to the SW TX buffer which in turn gets
 * pushed to the HW TX FIFO.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 * @param[in] buf       pointer to the string.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 * @retval E_UART_INVALID_PARAM invalid parameter passed
 */
error_t sbsa_uart_write_string(const struct tegra_uart_ctlr * const ctlr,
                               const char *buf);

/**
 * @brief suspend the UART context.
 *
 * This function saves the UART context across power gating the HW.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 */
error_t sbsa_uart_suspend(struct tegra_uart_ctlr * const ctlr);

/**
 * @brief resume the UART context.
 *
 * This function restores the UART context across power gating the HW.
 *
 * @pre the function sbsa_uart_init() has been called
 *
 * @param[in] ctlr      UART controller instance.
 *
 * @retval E_SUCCESS            indicates success
 * @retval E_UART_NULL_PTR      invalid id paramter passed
 */
error_t sbsa_uart_resume(struct tegra_uart_ctlr * const ctlr);

#endif
