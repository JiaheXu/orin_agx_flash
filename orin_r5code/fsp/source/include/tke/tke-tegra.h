/*
 * Copyright (c) 2015-2020 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TKE__TKE_TEGRA_H
#define TKE__TKE_TEGRA_H
#define FSP__TKE__TKE_TEGRA_H                           1

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */

/*
 * Declaration for tegra_tke_id that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
// IWYU pragma: no_forward_declare tegra_tke_id
struct tegra_tke_id;

/**
 * @brief ISR Callback function - When an interrupt is raised, this function
 * will be called by tegra_tke_isr for the timer that was registered. The
 * callback function is registered during the set-up stage of the timer(function
 * pointer is passed as an argument to the API).
 */
typedef void (*tegra_tke_timer_callback)(void * data);

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
/**
 * @brief Defines for the available clock sources.
 *
 * @macro-title Timer Clock Sources
 *
 * @TEGRA_TKE_CLK_SRC_USECCNT       1 MHz
 * @TEGRA_TKE_CLK_SRC_OSCCNT        clk_m
 * @TEGRA_TKE_CLK_SRC_TSC_BIT0      31.25 MHz
 * @TEGRA_TKE_CLK_SRC_TSC_BIT12     7.63 KHz
 */
#define TEGRA_TKE_CLK_SRC_USECCNT       0U
#define TEGRA_TKE_CLK_SRC_OSCCNT        1U
#define TEGRA_TKE_CLK_SRC_TSC_BIT0      2U
#define TEGRA_TKE_CLK_SRC_TSC_BIT12     3U

/** Maximum divisor for tke timers */
#define TEGRA_TKE_MAX_TIMER 0x20000000UL
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Initialize TSC and set up TSC to nanosecond conversion.
 * The FW can use the fixed TSC to nanosecond conversion present in tke-tegra-tsc-fixed.c
 * to avoid this api call.
 *
 * @jama_func_req_id 10797055
 */
void tegra_tsc_init(void);

/**
 * @brief Set up and start a Tegra timer.
 *
 * Timer has a PCV counter which starts from one minus the divisor and
 * counts downwards. The interrupt or timer output is triggered when
 * the PCV is 0 and a clock source tick occurs.
 * If the param callback is non-NULL, an interrupt will also be enabled.
 *
 * @jama_func_req_id 10797115
 *
 * @param[in] id          The context id struct of the timer block.
 * @param[in] clk_src_sel Clock source used to drive the timer. +
 *                        Available CLK sources - <<Timer Clock Sources>>
 * @param[in] periodic    If true, start timer in periodic mode.
 *                        If false, start timer in one-shot mode.
 * @param[in] divisor     The period of timer in clock source ticks.
 *                        Divisor must be TEGRA_TKE_MAX_TIMER or less.
 * @param[in] callback    The timer expiration callback invoked in irq context.
 * @param[in] data        The parameter to the callback.
 */
void tegra_tke_set_up_timer(struct tegra_tke_id *id, uint32_t clk_src_sel,
                        bool periodic, uint32_t divisor,
                        tegra_tke_timer_callback callback, void *data);

/**
 * @brief Stop a Tegra timer.
 *
 * @jama_func_req_id 10797141
 *
 * @param[in] id        The context id struct of the timer block.
 */
void tegra_tke_stop_timer(const struct tegra_tke_id *id);

/**
 * @brief Get current period counter value of a Tegra timer.
 *
 * @jama_func_req_id 10802167
 *
 * @param[in] id        The context id struct of the timer block.
 *
 * @retval 32-bit       Period counter value.
 */
uint32_t tegra_tke_get_pcv(const struct tegra_tke_id *id);

/**
 * @brief Enable the timer interrupt.
 *
 * @jama_func_req_id 10803499
 *
 * @param[in] id        The context id struct of the timer block.
 */
void tegra_tke_enable_timer_irq(const struct tegra_tke_id *id);

/**
 * @brief Disable the timer interrupt.
 *
 * @jama_func_req_id 10805710
 *
 * @param[in] id        The context id struct of the timer block.
 */
void tegra_tke_disable_timer_irq(const struct tegra_tke_id *id);

/**
 * @brief Clear pending timer interrupt.
 *
 * @jama_func_req_id 10805857
 *
 * @param[in] id        The context id struct of the timer block.
 */
void tegra_tke_clear_timer_irq(const struct tegra_tke_id *id);

/**
 * @brief Timer interrupt function.
 *
 * @jama_func_req_id 10805902
 *
 * @param[in] tke_id      The context id struct of the timer block.
 */
void tegra_tke_irq(void *tke_id);

/**
 * @brief Get the 64-bit TSC timer from top_tke_base block.
 *
 * @jama_func_req_id 10806040
 *
 * @retval 64-bit       Current TSC value
 */
uint64_t tegra_tke_get_tsc64(void);

/**
 * @brief Get the TSC timer from top_tke_base block.
 *
 * @jama_func_req_id 10806043
 *
 * @param[out] tsc_hi   The upper 32 bits of TSC counter.
 * @param[out] tsc_lo   The lower 32 bits of TSC counter.
 */
void tegra_tke_get_tsc(uint32_t *tsc_hi, uint32_t *tsc_lo);

/**
 * @brief Get the number of microseconds elapsed since given TCS timestamp.
 *
 * @pre Requires invoking tegra_tsc_init() from file "tke-tegra-tsc-var.c" to
 * calculate tsc-to-nanoseconds conversion ratio dynamically. +
 * If fixed predefined tsc-to-nanoseconds conversion ratio is used from file
 * "tke-tegra-tsc-fixed.c", no precondition is required.
 *
 * @jama_func_req_id 11266658
 *
 * @param[in] prev_tsc_hi   The upper 32 bits of TSC timestamp.
 * @param[in] prev_tsc_lo   The lower 32 bits of TSC timestamp.
 *
 * @retval 64-bit           Elapsed time in microseconds.
 * @retval 0                pre-condition not met(only in specific cases mentioned).
 */
uint64_t tegra_tke_get_elapsed_usec(uint32_t prev_tsc_hi, uint32_t prev_tsc_low);

/**
 * @brief Get the usec counter value from top_tke_base.
 *
 * @jama_func_req_id 10815565
 *
 * @retval 32-bit           TKE counter value in microseconds
 */
uint32_t tegra_tke_get_usec(void);

/**
 * @brief Get the osc counter value from top_tke_base.
 *
 * @jama_func_req_id 10816111
 *
 * @retval 32-bit           OSC counter value.
 */
uint32_t tegra_tke_get_osc(void);

/**
 * @brief Get the low 32 bits of TSC counter value from top_tke_base.
 *
 * @jama_func_req_id 10816201
 *
 * @retval 32-bit           Lower 32-bits of TSC Counter value.
 */
uint32_t tegra_tke_get_tsc32(void);

/**
 * @brief Get the current TSC counter value as nanoseconds from top_tke_base.
 *
 * @pre Requires invoking tegra_tsc_init() from file "tke-tegra-tsc-var.c" to
 * calculate tsc-to-nanoseconds conversion ratio dynamically. +
 * If fixed predefined tsc-to-nanoseconds conversion ratio is used from file
 * "tke-tegra-tsc-fixed.c", no precondition is required.
 *
 * @jama_func_req_id 10816420
 *
 * @retval 64-bit           Nanoseconds of TSC Counter value.
 * @retval 0                pre-condition not met(only in specific cases mentioned).
 */
uint64_t tegra_tke_get_tsc_ns(void);

/**
 * @brief Converts a TSC timestamp to nanoseconds.
 *
 * @pre Requires invoking tegra_tsc_init() from file "tke-tegra-tsc-var.c" to
 * calculate tsc-to-nanoseconds conversion ratio dynamically. +
 * If fixed predefined tsc-to-nanoseconds conversion ratio is used from file
 * "tke-tegra-tsc-fixed.c", no precondition is required.
 *
 * @jama_func_req_id 11223170
 *
 * @retval 64-bit           Nanoseconds converted value
 * @retval 0                pre-condition not met(only in specific cases mentioned).
 */
uint64_t tegra_tke_tsc_to_ns(uint64_t tsc);

/**
 * @brief Set up SafeRTOS tick timer.
 *
 * @jama_func_req_id 10816738
 *
 * @param[in] id            The context id struct of the timer block.
 * @param[in] clk_src       Clock source used to drive the timer. +
 *                          Available CLK sources <<Timer Clock Sources>>
 * @param[in] divisor       The period of timer in clock source ticks.
 */
void tegra_tke_set_up_tick(struct tegra_tke_id *id, uint32_t clk_src,
                                uint32_t divisor);

/**
 * @brief get elapsed microseconds
 *
 * @jama_func_req_id 10816807
 *
 * @param[in] start         Value of TSC at the start of period
 *
 * @retval 64-bit           Elapsed time in microseconds.
 */
uint64_t tegra_tke_get_elapsed_usecs64(const uint64_t start);

/**
 * @brief Convert a microsecond value to RTOS ticks
 *
 * @jama_func_req_id 10816846
 *
 * @param[in] value         Value in microseconds to be converted to RTOS ticks
 *
 * @retval 64-bit           Tick value.
 */
uint64_t tegra_tke_convert_usecs_to_ticks(const uint64_t value);


void tegra_tke_timer_writel_non_static(const struct tegra_tke_id *id, uint32_t val, uint32_t reg);

#endif  /* FSP_TKE_TKE_TEGRA_H */
