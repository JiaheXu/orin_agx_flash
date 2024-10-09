/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef GPIO__TEGRA_GPIO_H
#define GPIO__TEGRA_GPIO_H
#define FSP__GPIO__TEGRA_GPIO_H  1

/* Compiler headers */
#include <stdbool.h>              // for bool
#include <stdint.h>               // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>  // for E_SUCCESS, error_t
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module FSP headers */
#include <gpio/tegra-gpio-priv.h> // for struct tegra_gpio_id

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built int32_to the FSP source code.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define MAX_GPIO_CONTROLLERS               2U

// IWYU pragma: no_forward_declare tegra_gpio_id
struct tegra_gpio_id;

/**
 * @typedef-title Define data types for GPIO IRQ type and level
 */
typedef uint32_t tegra_gpio_irq_type;
typedef uint32_t tegra_gpio_irq_level;

/**
 * @macro-title Valid values of tegra_gpio_irq_type
 */
#define TEGRA_GPIO_IRQ_NONE          ((tegra_gpio_irq_type)0UL)
#define TEGRA_GPIO_IRQ_LEVEL         ((tegra_gpio_irq_type)1UL)
#define TEGRA_GPIO_IRQ_SINGLE_EDGE   ((tegra_gpio_irq_type)2UL)
#define TEGRA_GPIO_IRQ_DOUBLE_EDGE   ((tegra_gpio_irq_type)3UL)

/**
 * @macro-title Valid values of tegra_gpio_irq_level
 */
#define TEGRA_GPIO_IRQ_LOW_LEVEL     ((tegra_gpio_irq_level)0UL)
#define TEGRA_GPIO_IRQ_FALLING_EDGE  ((tegra_gpio_irq_level)0UL)
#define TEGRA_GPIO_IRQ_HIGH_LEVEL    ((tegra_gpio_irq_level)1UL)
#define TEGRA_GPIO_IRQ_RISING_EDGE   ((tegra_gpio_irq_level)1UL)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Construct a global GPIO ID.
 *
 * No error-range-checking is performed on the parameters or returned value.
 *
 * @param[in] ctrl_id    The unique ID of the GPIO controller.
 * @param[in] gpio       The GPIO ID local to the controller.
 *
 * @func_req_id 17373197
 *
 * @retval global_gpio_id The global GPIO ID.
 */
static inline uint32_t gpio_global_id(uint8_t ctrl_id,
                                      uint16_t gpio)
{
    return ((uint32_t)ctrl_id << 16) + (uint32_t)gpio;
}

/*
 * @brief Initialize the GPIO controller
 *
 * This function enables the interrupts for the GPIO controller
 * populated through the soc data and maps them to the generic GPIO
 * interrupt handler.
 *
 * @func_req_id 17372885
 *
 * @param[in] ids     Array of pointer to the controller SoC data.
 * @param[in] num_ctrls Number of GPIO controllers to be initialized
 *
 * @retval E_SUCCESS                    For Success
 * @retval E_GPIO_INVALID_GPIO_IRQ      For invalid IRQ number
 * @retval E_GPIO_INVALID_CONT_DATA     NULL pointer to controller data/Number
 *                                      of controllers exceeds the maximum limit
 */
error_t tegra_gpio_init(struct tegra_gpio_id **ids, uint32_t num_ctrls);

/**
 * @brief Disable the GPIO interrupts across SC7 entry/power
 * gating the hardware.
 *
 * This function disables the interrupts for the GPIO controllers
 * populated through the soc data.
 *
 * @func_req_id 17375798
 *
 * @pre GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @param[in] ctrl_id ID of the GPIO controller to be suspended.
 *
 * @retval E_SUCCESS                    For Success
 * @retval E_GPIO_INVALID_GPIO_IRQ      For invalid IRQ number
 * @retval E_GPIO_INVALID_GPIO_CONT_ID  Invalid controller ID
 * @retval E_GPIO_INVALID_CONT_DATA     NULL pointer to controller data
 */
error_t tegra_gpio_suspend(uint32_t ctrl_id);

/**
 * @brief Enable the GPIO interrupts across SC7 exit/un-power gating
 * the hardware.
 *
 * This function disables the interrupts for the GPIO controller
 * populated through the soc data.
 *
 * @pre GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17375798
 *
 * @param[in] ctrl_id ID of the GPIO controller to be resumed.
 *
 * @retval E_SUCCESS                    For Success
 * @retval E_GPIO_INVALID_GPIO_IRQ      For invalid IRQ number
 * @retval E_GPIO_INVALID_GPIO_CONT_ID  Invalid controller ID
 * @retval E_GPIO_INVALID_CONT_DATA     NULL pointer to controller data
 */
error_t tegra_gpio_resume(uint32_t ctrl_id);

/**
 * @brief Configure a GPIO to be an input.
 *
 * @pre GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17373494
 *
 * @param[in] gpio Global GPIO number.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_direction_in(uint32_t gpio);

/**
 * @brief Configure a GPIO to be an output, and set its initial output value.
 * This is the raw value at the pin; the GPIO API performs no
 * internal conversions to account for active-low signals, etc.
 *
 * @func_req_id 17373974
 *
 * @pre GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @param[in] gpio  Global GPIO number.
 * @param[in] value The value to drive on the GPIO.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_direction_out(uint32_t gpio,
                                 bool value);

/*
 * @brief Get direction of the GPIO
 *
 * Returns direction of GPIO as an integer value
 *
 * 0: GPIO configured for input mode
 *
 * 1: GPIO configured for output mode.
 *
 * @param[in]  gpio      Global GPIO number
 * @param[out] gpio_dir  An in/out parameter that captures direction of the GPIO
 *
 * @pre GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_get_direction(uint32_t gpio,
                                 uint32_t *gpio_dir);

/**
 * @brief Retrieve the raw state of a GPIO.
 *
 * Returns raw value at the pin.
 *
 * False:       Success, signal is low.
 *
 * True:        Success, signal is high.
 *
 * The GPIO API performs no internal conversions to account
 * for active-low signals, etc.
 *
 * @func_req_id 17374043
 *
 * @param[in]  gpio        Global GPIO number.
 * @param[out] gpio_val    An in/out parameter that captures the raw value
 *                         at the pin
 *
 * @pre GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_get_value(uint32_t gpio,
                             bool *gpio_val);

/**
 * @brief Retrieve the state of an output GPIO.
 *
 * Returns raw value at the pin.
 *
 * False:       Success, output signal is low.
 *
 * True:        Success, output signal is high.
 *
 * The GPIO API performs no internal conversions to account
 * for active-low signals, etc.
 *
 * @func_req_id 17374070
 *
 * @pre
 * - GPIO must be configured as an output
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @param[in]  gpio        Global GPIO number.
 * @param[out] gpio_val An in/out parameter that captures the raw value
 *                         at the pin
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_get_output_value(uint32_t gpio,
                                    bool *gpio_val);

/**
 * @brief Retrieve the state of an input GPIO.
 *
 * Returns raw value at the pin
 *
 * False:       Success, input signal is low.
 *
 * True:        Success, input signal is high.
 *
 * The GPIO API performs no internal conversions to account
 * for active-low signals, etc.
 *
 * @func_req_id 17374190
 *
 * @pre
 * - GPIO must be configured as an input.
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @param[in]  gpio        Global GPIO number.
 * @param[out] gpio_val An in/out parameter that captures the raw value
 *                        at the pin
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_get_input_value(uint32_t gpio,
                                   bool *gpio_val);

/**
 * @brief Set the output state for a GPIO.
 *
 * Sets raw value at the pin.
 *
 * Request to drive the GPIO can take one of the following values
 *
 * True:  signal level high
 *
 * False: Signal level low.
 *
 * @func_req_id 17374406
 *
 * @pre
 * - GPIO must be configured as an output.
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @param[in] gpio  Global GPIO number.
 * @param[in] value The value to drive on the GPIO.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_set_value(uint32_t gpio,
                             bool value);

/**
 * @brief Set the debounce value for a GPIO.
 *
 * @pre
 * - GPIO must be configured as an output
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17374592
 *
 * @param[in] gpio        Global GPIO number.
 * @param[in] debounce_ms The value to program for debounce in miliseconds.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_set_debounce(uint32_t gpio,
                                uint32_t debounce_ms);

/**
 * @brief Set the irq type for a GPIO.
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17375066
 *
 * @param[in] gpio   Global GPIO number.
 * @param[in] type   Indicates the type based on tegra_gpio_irq_type.
 * @param[in] level  Indicates the level based on tegra_gpio_irq_level.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_IRQ_LEVEL      Invalid IRQ level
 * @retval E_GPIO_INVALID_IRQ_TYPE       Invalid IRQ type
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_set_irq_type(uint32_t gpio,
                                tegra_gpio_irq_type type,
                                tegra_gpio_irq_level level);

/**
 * @brief Enable the IRQ for a GPIO.
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17375444
 *
 * @param[in] gpio   Global GPIO number.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_GPIO_IRQ       Invalid IRQ number
 * @retval E_GPIO_INAVLID_IRQ_HANDLER    NULL GPIO IRQ handler
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_enable_irq(uint32_t gpio);

/**
 * @brief Enable the timestamping functionality for a GPIO.
 *
 * This API needs to be called for a GPIO that you are interested in
 * monitoring using the HW timestamping engines such as GTE.
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17375795
 *
 * @param[in] gpio   Global GPIO number.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_enable_timestamp(uint32_t gpio);

/**
 * @brief Disable the timestamping functionality for a GPIO.
 *
 * @pre
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 * - GPIO must be configured as an input
 *
 * @func_req_id 17375795
 *
 * @param[in] gpio   Global GPIO number.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_disable_timestamp(uint32_t gpio);

/**
 * @brief Disable the IRQ for a GPIO.
 *
 * @pre
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 * - GPIO must be configured as an input
 *
 * @func_req_id 17375444
 *
 * @param[in] gpio   Global GPIO number.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_GPIO_IRQ       Invalid IRQ number
 * @retval E_GPIO_INAVLID_IRQ_HANDLER    NULL GPIO IRQ handler
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_disable_irq(uint32_t gpio);

/**
 * @brief Register an interrupt service routine for a GPIO.
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17490935
 *
 * @param[in] gpio      Global GPIO number.
 * @param[in] handler   Pointer to the service routine.
 * @param[in] data      opaque data pointer for the service routine.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_GPIO_IRQ       Invalid IRQ number
 * @retval E_GPIO_INAVLID_IRQ_HANDLER    NULL GPIO IRQ handler
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_set_irq_handler(uint32_t gpio,
                                   void (*handler)(void *hdata),
                                   void *data);

/**
 * @brief Unregister an interrupt service routine for a GPIO.
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 17490959
 *
 * @param[in] gpio  Global GPIO number.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_GPIO_IRQ       Invalid IRQ number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_clear_irq_handler(uint32_t gpio);

/**
 * @brief Read IRQ status of a GPIO pin
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 19739553
 *
 * @param[in]  gpio        Global GPIO
 * @param[out] irq_status  An in/out parameter to save IRQ status of the GPIO
 *                         pin.
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_GPIO_IRQ       Invalid IRQ number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_read_irq_status(uint32_t gpio, bool *irq_status);

/**
 * @brief Clear IRQ status of a GPIO pin
 *
 * @pre
 * - GPIO must be configured as an input
 * - GPIO controller should have been initialized as part of tegra_gpio_init
 *
 * @func_req_id 19739562
 *
 * @param[in] gpio Global GPIO number
 *
 * @retval E_SUCCESS                     Success
 * @retval E_GPIO_INVALID_GPIO_CONT_ID   Invalid controller ID
 * @retval E_GPIO_INVALID_GPIO           Invalid GPIO number
 * @retval E_GPIO_INVALID_GPIO_IRQ       Invalid IRQ number
 * @retval E_GPIO_INVALID_CONT_DATA      NULL pointer to controller data
 */
error_t tegra_gpio_clear_irq_status(uint32_t gpio);

/**
 * @brief GPIO interrupt handler.
 *
 * This function calls the interrupt handler corresponding to the
 * GPIO pin as mapped by the client using tegra_gpio_set_irq_handler.
 *
 * @pre
 * - GPIO controller should have been initialized as part of tegra_gpio_init.
 *
 * @func_req_id 19739889
 *
 * @param[in] data Pointer to the GPIO controller data
 *
 * @return None
 */
void tegra_gpio_irq_handler(void *data);

#endif
