/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef WATCHDOG__TEGRA_SAFE_WDT_H
#define WATCHDOG__TEGRA_SAFE_WDT_H
#define FSP__WATCHDOG__TEGRA_SAFE_WDT_H                      1

/**
 * @file tegra-safe-wdt.h
 * @brief Defines safe driver to Tegra WDT hardware
 */

/* Compiler headers */
#include <stdbool.h>              // for bool
#include <stdint.h>               // for uint8_t, uint32_t
                                  // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>  // for error_t, FSP__ERROR__COMMON_ERRORS_H
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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

/*
 * Definition for tegra_wdt_id that allows the APIs to take a pointer to
 * it without actually defining its contents here.
 */
// IWYU pragma: no_forward_declare tegra_wdt_id
struct tegra_wdt_id;

/**
 * @brief Watchdog Callback function
 *
 * This is used to define the call back function that can be
 * supplied that is called when a watchdog IRQ or FIQ occurs.
 *
 * @pre None
 *
 * @param[in]  status  Status of the watchdog timer when the callback
 *                     function was called
 * @param[in]  data    Pointer to data to be supplied to the callback
 *                     function.  The pointer was specified when the watchdog
 *                     timer was initialized.
 *
 * @return None
 */
typedef void (*tegra_wdt_callback)(uint32_t status, void *data);

/**
 * @brief Watchdog Initialization Structure
 *
 * Provides the initialization parameters for a watchdog timer.  Used in
 * tegra_safe_wdt_setup().
 */
struct tegra_wdt_conf
{
    uint8_t             tmrsrc;                 ///! TKE timer to use
    uint8_t             period;                 ///! number of timer ticks
                                                ///!  in watchdog period
    bool                irq_en;                 ///! Enable IRQ
    bool                fiq_en;                 ///! Enable FIQ
    bool                remoteIrq_en;           ///! Enable generating remote
                                                ///!  interrupt
    bool                sys_dbg_rst_en;
    bool                sys_por_rst_en;
    bool                tsc_ref_en;
    uint8_t             err_threshold;
    bool                challenge_response;     ///! Only allow timer updates
                                                ///!  after proper
                                                ///!  challenge / response
    bool                irq_ack_by_callback;    ///! IRQ is acknowleged by
                                                ///!  callback function
    bool                fiq_ack_by_callback;    ///! IRQ is acknowleged by
                                                ///!  callback function
    tegra_wdt_callback  irq_callback;           ///! IRQ callback function
    void                *irq_data;              ///! Data supplied to IRQ
                                                ///!  callback function
    tegra_wdt_callback  fiq_callback;           ///! FIQ callback function
    void                *fiq_data;              ///! Data supplied to IRQ
                                                ///!  callback function
};

/*
 * Tegra WDT driver
 *
 * To use WDT driver,
 * 1. Call tegra_wdt_setup to setup watchdog timer first
 * 2. Call tegra_wdt_start to enable watchdog timer
 *
 * The wdt driver IRQ/FIQ handler will ack the watchdog automatically. If
 * you want to do it from the callback or from a scheduld task, set
 * irq_ack_by_callback/fiq_ack_by_callback.
 */

/**
 * @brief Initialize watchdog timer
 *
 * This function will initialize the specified watchdog timer according to
 * the configuration parameters supplied.
 *
 * @note This function can only be called if the specified watchdog timer
 * is disabled.  If the specified watchdog timer is enabled, then it is
 * necessary to call either tegra_safe_wdt_stop() or
 * tegra_safe_wdt_stop_from_isr() to first stop the watchdog timer.
 *
 * This function must be called from a non-interrupt context.
 *
 * @pre The specified watchdog timer must not be running.
 *
 * @param[in]  id    pointer to the tegra_wdt_id structure that represents
 *                   the watchdog timer being setup
 * @param[in]  conf  pointer to the tegra_wdt_conf structure that contains
 *                   the configuration parameters to be used when initializing
 *                   the watchdog timer.
 *
 * @return Error Code
 *
 * @retval E_SUCCESS                    watchdog timer was successfully
 *                                      initialized
 * @retval E_WDT_ERR_NULL_PTR           one of the pointers passed was NULL
 * @retval E_WDT_ERR_IRQ_CONFIG         The watchdog's IRQ or FIQ was
 *                                      enabled but no corresponding
 *                                      callback function was specified
 * @retval E_WDT_ERR_TMRSRC_INVALID     value specified for tmrsrc was invalid
 * @retval E_WDT_ERR_THRESHOLD_INVALID  value specified for threshold was
 *                                      invalid
 */
error_t tegra_safe_wdt_setup(struct tegra_wdt_id *id,
                             const struct tegra_wdt_conf *conf);

/**
 * @brief Start Watchdog Timer
 *
 * This function will start the specified watchdog timer using the parameters
 * that were supplied as part of tegra_safe_wdt_setup().
 *
 * This function can be called from either an interrupt context or a
 * non-interrupt context.
 *
 * @pre The function tegra_safe_wdt_setup() must have been previously called.
 *
 * @param[in]  id  pointer to the tegra_wdt_id structure that represents the
 *                 watchdog timer being started.
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   id was NULL
 */
error_t tegra_safe_wdt_start(const struct tegra_wdt_id *id);

/**
 * @brief Acknowlege Watchdog Timer
 *
 * This function will acknowlege the specified watchdog timer and
 * reset its operation so that it will restart its count.
 *
 * This function can be called from either an interrupt context or a
 * non-interrupt context.
 *
 * @pre tegra_safe_wdt_setup() must have been previously called.
 *
 * @param[in]  id  pointer to the tegra_wdt_id structure that represents the
 *                 watchdog timer being acknowleged.
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   id was NULL
 */
error_t tegra_safe_wdt_ack(const struct tegra_wdt_id *id);

/**
 * @brief Stop watchdog timer
 *
 * This function will stop the specified watchdog timer.  It will stop it
 * from counting and thus prevent it from generating an IRQ or FIQ from the
 * watchdog timer.
 *
 * This function must be called from an interrupt context.
 *
 * @pre tegra_safe_wdt_setup() must have been previously called.
 *
 * @param[in]  id  pointer to the tegra_wdt_id structure that represents the
 *                 watchdog timer being stopped.
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   id was NULL
 */
error_t tegra_safe_wdt_stop_from_isr(const struct tegra_wdt_id *id);

/**
 * @brief Stop watchdog timer
 *
 * This function will stop the specified watchdog timer.  It will stop it
 * from counting and thus prevent it from generating an IRQ or FIQ from the
 * watchdog timer.
 *
 * This function must be called from a non-interrupt context.
 *
 * @pre tegra_safe_wdt_setup() must have been previously called.
 *
 * @param[in]  id  pointer to the tegra_wdt_id structure that represents the
 *                 watchdog timer being stopped.
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   id was NULL
 */
error_t tegra_safe_wdt_stop(const struct tegra_wdt_id *id);

/**
 * @brief Watchdog timer interrupt handler
 *
 * This function is responsible for servicing the watchdog IRQ.  The
 * behavior of this function is dependent upon how the watchdog timer
 * was configured.
 *
 * This function is called from the first level interrupt handler for
 * the watchdog interrupt, so it will be called only from an interrupt
 * context.
 *
 * @pre tegra_safe_wdt_setup() must have been previously called.
 * @pre the watchdog timer must have been previously started
 *
 * @param[in]  id  pointer to the tegra_wdt_id structure that represents the
 *                 watchdog timer that has generated an interrupt.
 *
 * @return None
 */
void tegra_safe_wdt_irq(void *data);

/**
 * @brief Watchdog timer fast interrupt handler
 *
 * This function is responsible for servicing the watchdog FIQ.  The
 * behavior of this function is dependent upon how the watchdog timer
 * was configured.
 *
 * This function is called from the first level interrupt handler for
 * the watchdog fast interrupt, so it will be called only from an interrupt
 * context.
 *
 * @pre tegra_safe_wdt_setup() must have been previously called.
 * @pre the watchdog timer must have been previously started
 *
 * @param[in]  id  pointer to the tegra_wdt_id structure that represents the
 *                 watchdog timer that has generated an FIQ interrupt.
 *
 * @return None
 */
void tegra_safe_wdt_fiq(void *data);

/**
 * @brief Read watchdog timer status
 *
 * This function will return the current status of the specified watchdog
 * timer.
 *
 * This function can be called from either an interrupt context or a
 * non-interrupt context.
 *
 * @pre None
 *
 * @param [in]  id      pointer to the tegra_wdt_id structure that represents
 *                      the watchdog timer whose status is to be returned
 * @param [in]  status  pointer to a 32-bit value that will be updated as a
 *                      result of this call that contains the watchdog's
 *                      timer status.  The value is interpreted as follows:
 *
 *   bit 0      Enabled                 1b when the counter is active
 *
 *                                      0b when the counter is disabled
 *
 *   bit 1      LocalIRQStatus          Current status of interrupt
 *
 *   bit 2      LocalFIQStatus          Current status of FIQ
 *
 *   bit 3      RemoteInterruptStatus   Current status of remote interrupt
 *
 *   bit 11:4   CurrentCount            Current value of the counter
 *
 *   bit 14:12  CurrentExpirationCount  Current count of expiration since
 *                                      last start operation
 *
 *   bit 16     CurrentError            Current error reported to HSM
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   one of the pointers passed was NULL
 *
 */
error_t tegra_safe_wdt_read_status(const struct tegra_wdt_id *id,
                                   uint32_t *status);


/**
 * @brief Enable windowed mode operation for the watchdog timer
 *
 * This function sets the timer window parameters, start level and start count
 * and enables the windowed mode operation.
 *
 * @pre tegra_safe_wdt_setup should be run before calling this function
 *
 * @param[in]  id         pointer to the tegra_wdt_id structure that represents
 *                        the watchdog timer whose status is to be returned
 * @param[in] start_level Threshold for expiration level
 * @param[in] start_count Number of cycles where rejection takes place at the
 *                        start of that level
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   one of the pointers passed was NULL
 */
error_t tegra_safe_enable_wdt_windowed_operation(const struct tegra_wdt_id *id,
                                                 uint8_t start_level,
                                                 uint8_t start_count);

/**
 * @brief Disable windowed mode operation for the watchdog timer
 *
 * This function disables the windowed mode operation for the watchdog timer
 *
 * @pre none
 *
 * @param[in]  id         pointer to the tegra_wdt_id structure that represents
 *                        the watchdog timer whose status is to be returned
 *
 * @return Error Code
 * @retval E_SUCCESS            status was returned successfully
 * @retval E_WDT_ERR_NULL_PTR   one of the pointers passed was NULL
 */
error_t tegra_safe_disable_wdt_windowed_operation(const struct tegra_wdt_id *id);
#endif /* WATCHDOG__WDT_TEGRA_H */
