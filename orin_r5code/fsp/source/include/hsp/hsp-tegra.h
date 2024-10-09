/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef HSP__HSP_TEGRA_H
#define HSP__HSP_TEGRA_H
#define FSP__HSP__HSP_TEGRA_H                           1

/* Compiler headers */
#include <stdint.h>                     // for uint32_t
#include <stdbool.h>                    // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <ospl/rtos-port.h>             // for FSP__OSPL__RTOS_PORT_H, rtosBool
#include <error/common-errors.h>        // for error_t, FSP__ERROR__COMMON...

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file hsp-tegra.h
 * @brief functions that provide access to various HSP registers.
 */

/** @defgroup hsp HSP – Hardware Synchronization Primitives
 *
 * HSP is a simple hardware block for communication between CCPLEX and
 * auxiliary processor. There are SoC-wide "top" HSP blocks and HSP
 * blocks specific to their auxiliary processor (SCE, APE, RCE, SPE
 * and BPMP). The Top1 and HSP blocks for auxiliary processors provide
 * 5 shared interrupts (8 in Top1), 8 shared mailboxes and 4 shared
 * semaphores. The Top0 HSP block also provide doorbells and
 * arbitrated semaphores, but only 2 shared semaphores.
 *
 * Each hardware instance is represented by hardware interface
 * descriptor with type struct tegra_hsp_id specifying the hardware
 * resources (like base address and interrupt line number) specific to
 * the hardware instance and its user. There can be multiple interface
 * descriptors referring to the same hardware instance, for example,
 * the mailboxes on AON HSP are used by multiple processors for the
 * combined UART.
 *
 * The hardware interface descriptor has a configuration field represented
 * by struct tegra_hsp_conf that is read-only.
 *
 * All driver methods are passed a pointer to the hardware interface
 * descriptor as their first argument.
 *
 * If a driver method is provided an uninitialized hardware interface
 * descriptor, invalid shared mailbox number or invalid shared
 * semaphore number, the method is a no-op will not read or write to
 * the hardware, unless otherwise specified.
 *
 * All argument pointers must be non-NULL unless otherwise specified.
 *
 * @par HSP Dimensioning
 *
 * Each HSP instance has a dimensioning register describing the HW
 * layout of the device:
 * - number of shared mailboxes
 * - number of shared semaphores
 * - number of arbitrated semaphores (no implementation in common driver)
 * - number of doorbells
 * - number of shared interrupts
 * The address map of the HSP instances vary and the above information
 * is used to calculate offsets to blocks inside the HSP address
 * space.
 *
 * @par HSP Shared Interrupts
 *
 * One of the shared interrupts lines is connected directly to the
 * interrupt controller of the auxiliary processor. The rest of the
 * shared interrupt lines are connected to LIC (Legacy Interrupt
 * Controller, intermediate interrupt controller that can deliver
 * interrupts to either CCPLEX CPUs or any of the auxiliary CPUs.
 *
 * The shared interrupt can combine interrupts from multiple sources:
 * shared mailboxes, doorbell and arbitrated semaphores. One interrupt
 * line is exclusively owned by single entity (VM running on CCPLEX or
 * auxiliary processor).
 *
 * @par HSP Shared Mailboxes
 *
 * The shared mailboxes are 32-bit wide registers with a very simple
 * interrupt mechanism controlled by their most significant bit.
 * Their simplest operating model is single writer/single reader.
 *
 * Each shared mailbox has the following resources:
 * - A 32 bit register
 * - Full and empty interrupts controlled by the value of the MSB.
 *   The MSB is interpreted as a Valid bit, so the full interrupt is
 *   the value of the MSB and the empty interrupt is the inverted
 *   value of the MSB.
 * - Enable bits for full and empty interrupts
 *
 * @par HSP Shared Semaphores
 *
 * The shared semaphores are formed by three registers, each 32 bits wide:
 * - A read-only register showing the current value of the semaphore
 * - Two write-only registers to set and clear individual semaphore bits
 *
 * These semaphores can be used for synchronizing processors acting in
 * a producer/consumer relationship, i.e. a pair of them can be used
 * to implement either a two or four way handshake. Correct operation
 * of the shared semaphores requires statically allocating an owner
 * for each bit, as it is impossible to implement an atomic test and
 * set (or similar operation) given the register interface.
 *
 * @{
 */

#define HSP_SM_TYPE_32          0UL
#define HSP_SM_TYPE_128         1UL
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/*
 * Declaration for tegra_hsp_id that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
// IWYU pragma: no_forward_declare tegra_hsp_id
struct tegra_hsp_id;

/**
 * @brief HSP context saving structure
 *
 * Provides the context parameters to save and restore across power gating
 * the hardware/SC7.  Used in tegra_hsp_[suspend/resume]().
 *
 * @db_enable	save the enable masters across power gating the HW
 * @si_enable	save the shared interrupt enable status
 */
struct tegra_hsp_suspend_ctx {
    uint32_t db_enable;
    uint32_t si_enable;
};

/**
 * @brief HSP mailbox 128-bit descriptor
 *
 * Provides a data type for the 128-bit data for the wider mailboxes.
 *
 * @d0          data represeting 0-31 bits of the 128-bit data
 * @d1          data represeting 32-63 bits of the 128-bit data
 * @d2          data represeting 64-95 bits of the 128-bit data
 * @d3          data represeting 96-127 bits of the 128-bit data
 * @tag         tag value to trigger the interrupt
 */
typedef struct {
    uint32_t d0;
    uint32_t d1;
    uint32_t d2;
    uint32_t d3;
    uint32_t tag;
} hsp128_t;

/**
 * @brief HSP mailbox data field descriptor
 *
 * Provides a data type for the data field of the shared mailboxes.
 *
 * @data        data field of the 128-bit mailbox
 * @val         data field of the 32 bit mailbox
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
typedef struct {
    union {
        hsp128_t data;
        uint32_t val;
    } sm;
} hsp_sm_data_t;
END_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief HSP doorbell callback function
 *
 * This is used to define the callback function that can be
 * supplied and is called when a doorbell irq occurs.
 *
 * @param[in] source id of the processor that raised the doorbell signal
 * @param[in] higher_prio_task_woken required for task context switch
 *
 * @return none
 */
typedef void (*tegra_hsp_db_callback)(uint32_t source,
                                      rtosBool *higher_prio_task_woken);

/**
 * @brief HSP mailbox callback function
 *
 * This is used to define the callback function that can be
 * supplied and is called when a mailbox irq occurs.
 *
 * @param[in] data  opaque data pointer for the callback
 * @param[in] value pointer to the content of the shared mailbox
 *
 * @return none
 */
typedef void (*tegra_hsp_sm_callback)(void *data,
                                      hsp_sm_data_t *value);

/**
 * @brief Initialize the HSP instance and setup doorbel callback
 *
 * @jama_func_req_id 12997720
 *
 * This function initializes the HSP context for an instance, registers the
 * callback for doorbell irq and enables the masters to raise the doorbell.
 *
 * @param[in] id HSP instance.
 * @param[in] enabled_masters bit mask of the master ids that can raise the
 *            doorbell signal. valid masters are as follows:
 *            TEGRA_HSP_DB_MASTER_CCPLEX
 *            TEGRA_HSP_DB_MASTER_DPMU
 *            TEGRA_HSP_DB_MASTER_BPMP
 *            TEGRA_HSP_DB_MASTER_SPE
 *            TEGRA_HSP_DB_MASTER_SCE
 *            TEGRA_HSP_DB_MASTER_DMA
 *            TEGRA_HSP_DB_MASTER_TSECA
 *            TEGRA_HSP_DB_MASTER_TSECB
 *            TEGRA_HSP_DB_MASTER_JTAGM
 *            TEGRA_HSP_DB_MASTER_CSITE
 *            TEGRA_HSP_DB_MASTER_APE
 *            TEGRA_HSP_DB_MASTER_RCE
 *            TEGRA_HSP_DB_MASTER_NON_SECURE
 * @param[in] callback callback that gets called from HSP DB irq handler.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_DB no doorbell support
 * @retval E_HSP_ERR_INVALID_CALLBACK invalid doorbell callback
 */
error_t tegra_hsp_db_init(struct tegra_hsp_id *id,
                          uint32_t enabled_masters,
                          tegra_hsp_db_callback callback);

/**
 * @brief raise the doorbell signal for the target processor.
 *
 * @jama_func_req_id 10710297
 *
 * This function raises the doorbell signal to the target processor.
 *
 * @pre the function tegra_hsp_db_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] target master ID of the target. valid targets are:
 *            TEGRA_HSP_DB_DPMU
 *            TEGRA_HSP_DB_CCPLEX
 *            TEGRA_HSP_DB_CCPLEX_TZ
 *            TEGRA_HSP_DB_BPMP
 *            TEGRA_HSP_DB_SPE
 *            TEGRA_HSP_DB_SCE
 *            TEGRA_HSP_DB_APE
 *            TEGRA_HSP_DB_RCE
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_DB no doorbell support
 */
error_t tegra_hsp_db_ring(const struct tegra_hsp_id *id,
                          uint32_t target);

/**
 * @brief enable the master ids that can raise doorbell signal
 *
 * @jama_func_req_id 10715379
 *
 * This function enables the master ids that can raise a doorbell
 * signal to the this target processor.
 *
 * @pre the function tegra_hsp_db_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] enabled_masters bit mask of the master ids that can raise the
 *            doorbell signal. Range: [0 - UINT16_MAX]
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_DB no doorbell support
 */
error_t tegra_hsp_db_enable_master(const struct tegra_hsp_id *id,
                                   uint32_t enabled_masters);

/**
 * @brief disable the master ids that can raise doorbell signal
 *
 * @jama_func_req_id 10716165
 *
 * This function disbles the master ids to prevent servicing doorbell
 * signals from this.
 *
 * @pre the function tegra_hsp_db_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] disabled_masters bit mask of the master ids that are disabled from
 *            raising the doorbell signal. Range: [0- UNIT16_MAX].
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_DB no doorbell support
 */
error_t tegra_hsp_db_disable_master(const struct tegra_hsp_id *id,
                                    uint32_t disabled_masters);

/**
 * @brief HSP doorbell irq handler.
 *
 * @jama_func_req_id 12999372
 *
 * This function gets called whenever a remote processor has raised a doorbell
 * signal for this master.
 *
 * @pre the function tegra_hsp_db_init() has been called
 *
 * @param[in] data pointer for HSP context.
 *
 * @return None
 */
void tegra_hsp_db_irq_handler(void *data);

/**
 * @brief HSP mailbox irq handler.
 *
 * @jama_func_req_id 12999352
 *
 * This function gets called whenever a mailbox full/empty interrupt is
 * triggered.
 *
 * @pre the function tegra_hsp_init() has been called
 *
 * @param[in] data pointer for HSP context.
 *
 * @return None
 */
void tegra_hsp_irq_handler(void *data);

/**
 * @brief save the HSP context across SC7 entry/power gating the hardware
 *
 * @jama_func_req_id 10707279
 *
 * This function preserves the context such as the enabled doorbells and SIs
 * across power gating the hardware(SC7).
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sctx A pointer to where the saved context can be stored.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 */
error_t tegra_hsp_suspend(const struct tegra_hsp_id *id,
                          struct tegra_hsp_suspend_ctx *sctx);

/**
 * @brief resume the HSP context across SC7 exit/unpower gating the hardware
 *
 * @jama_func_req_id 10707285
 *
 * This function restores the context such as the enabled doorbells and SIs
 * across power gating the hardware(SC7).
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sctx A pointer to where the saved context was previously stored.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 */
error_t tegra_hsp_resume(const struct tegra_hsp_id *id,
                         const struct tegra_hsp_suspend_ctx *sctx);

/**
 * @brief Initialize the HSP context
 *
 * @jama_func_req_id 10708482
 *
 * This function initializes the HSP context for an instance, enables the
 * HSP doorbell and shared mailbox interrupts.
 *
 * @param[in] id pointer to the HSP descriptor instance.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 */
error_t tegra_hsp_init(struct tegra_hsp_id *id);

/**
 * @brief Enable HSP shared mailbox full interrupt.
 *
 * @jama_func_req_id 10709553
 *
 * This function enables the HSP shared mailbox full interrupt. This function
 * defaults to 32-bit shared mailbox types if the client has not explicitly
 * called tegra_hsp_set_sm_type() to indicate the mailbox width.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] cb callback to be called on an IRQ.
 * @param[in] data opaque data pointer for the callback.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_NO_INTR no HSP interrupt support
 */
error_t tegra_hsp_sm_full_enable(struct tegra_hsp_id *id,
                                 uint32_t sm,
                                 tegra_hsp_sm_callback cb,
                                 void *data);

/**
 * @brief Disable HSP shared mailbox full interrupt.
 *
 * @jama_func_req_id 10709868
 *
 * This function disables the HSP shared mailbox full interrupt.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_NO_INTR no HSP interrupt support
 */
error_t tegra_hsp_sm_full_disable(struct tegra_hsp_id *id,
                                  uint32_t sm);

/**
 * @brief Enable HSP shared mailbox empty interrupt.
 *
 * @jama_func_req_id 10709979
 *
 * This function enables the HSP shared mailbox empty interrupt. This function
 * defaults to 32-bit shared mailbox types if the client has not explicitly
 * called tegra_hsp_set_sm_type() to indicate the mailbox width.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] cb callback to be called on an IRQ.
 * @param[in] data opaque data pointer for the callback.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_NO_INTR no HSP interrupt support
 */
error_t tegra_hsp_sm_empty_enable(struct tegra_hsp_id *id,
                                  uint32_t sm,
                                  tegra_hsp_sm_callback cb,
                                  void *data);

/**
 * @brief Disable HSP shared mailbox empty interrupt.
 *
 * @jama_func_req_id 10710072
 *
 * This function disables the HSP shared mailbox empty interrupt.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_NO_INTR no HSP interrupt support
 */
error_t tegra_hsp_sm_empty_disable(struct tegra_hsp_id *id,
                                   uint32_t sm);

/**
 * @brief write to the HSP shared mailbox register with TAG bit
 *
 * @jama_func_req_id 10708650
 *
 * This function writes the mailbox register with the specified value in
 * the data field and sets the TAG bit.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] value value to be written to the data field.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_INVALID_DATA if 0xDEAD1001 is written due to HW bug 200395605
 */
error_t tegra_hsp_sm_produce(const struct tegra_hsp_id *id,
                             uint32_t sm,
                             uint32_t value);

/**
 * @brief read the HSP shared mailbox by clearing the contents
 *
 * @jama_func_req_id 10708695
 *
 * This function returns the data field of the shared mailbox register
 * by clearing the contents of it.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] data pointer to where the mailbox data field is written to.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_EMPTY_MBOX mailbox tag field is empty
 */
error_t tegra_hsp_sm_consume(const struct tegra_hsp_id *id,
                             uint32_t sm,
                             uint32_t *data);

/**
 * @brief read the HSP shared mailbox without clearing it
 *
 * @jama_func_req_id 10709247
 *
 * this function fetches the data field of the mailbox register
 * without modifying its contents.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] data pointer to where the mailbox data field is written to.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_EMPTY_MBOX mailbox tag field is empty
 */
error_t tegra_hsp_sm_peek(const struct tegra_hsp_id *id,
                          uint32_t sm,
                          uint32_t *data);

/**
 * @brief clear contents of the shared mailbox register
 *
 * @jama_func_req_id 10709412
 *
 * This function clears the contents of the shared mailbox register.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 */
error_t tegra_hsp_sm_vacate(const struct tegra_hsp_id *id,
                            uint32_t sm);

/**
 * @brief 128-bit shared mailbox irq handler
 *
 * @jama_func_req_id 10709412
 *
 * This function serves as the 128-bit shared mailbox irq handler and is
 * called from the main HSP irq handler.
 *
 * @param[in] id   HSP instance.
 * @param[in] sm   Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 *
 * @return none
 */
void tegra_hsp_sm128_irq_handler(const struct tegra_hsp_id *id,
                                 uint32_t sm);

/**
 * @brief check if mailbox is empty or not
 *
 * @jama_func_req_id 10709223
 *
 * This function indicates whether the mailbox is empty or not.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index to which data has to be written.
 *            Range: [0- MAX SM INDEX of the instance]
 *
 * @retval True indicates that the mailbox is empty
 * @retval False indicates that the mailbox is not empty
 */
bool tegra_hsp_sm_is_empty(const struct tegra_hsp_id *id,
                           uint32_t sm);

/**
 * @brief read the contents of the shared semaphore register
 *
 * @jama_func_req_id 10701256
 *
 * This function returns the current value of the shared semaphore register.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared semaphore index to which data has to be written.
 *            Range: [0- MAX SS INDEX of the instance]
 * @param[in] value A pointer to where the read value should be returned in.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_SHRD_SMPHR invalid shared semaphore index
 */
error_t tegra_hsp_ss_read(const struct tegra_hsp_id *id,
                          uint32_t index,
                          uint32_t *value);

/**
 * @brief set specific bits in a shared semaphore register
 *
 * @jama_func_req_id 10701259
 *
 * This function sets the bits specified by the data parameter in
 * the shared semaphore.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared semaphore index to which data has to be written.
 *            Range: [0- MAX SS INDEX of the instance]
 * @param[in] data Specify the bits to be set in the semaphore.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_SHRD_SMPHR invalid shared semaphore index
 */
error_t tegra_hsp_ss_set(const struct tegra_hsp_id *id,
                         uint32_t index,
                         uint32_t data);

/**
 * @brief clear specific bits in a shared semaphore register
 *
 * @jama_func_req_id 10707267
 *
 * This function clears the bits specified by the data parameter in
 * the shared semaphore register.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared semaphore index to which data has to be written.
 *            Range: [0- MAX SS INDEX of the instance]
 * @param[in] data Specify the bits to be cleared in the semaphore.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_SHRD_SMPHR invalid shared semaphore index
 */
error_t tegra_hsp_ss_clear(const struct tegra_hsp_id *id,
                           uint32_t index,
                           uint32_t data);

/**
 * @brief Return number of shared mailboxes supported by hardware.
 *
 * @jama_func_req_id 12999380
 *
 * @param[in] id    Pointer to hardware interface descriptor
 *
 * @return Number of shared mailboxes.
 *
 * @note An uninitialized @a id is accepted.
 */
uint32_t tegra_hsp_sm_count(const struct tegra_hsp_id *id);

/**
 * @brief Return number of shared semaphores supported by hardware.
 *
 * @jama_func_req_id 12999420
 *
 * @param[in] id    Pointer to hardware interface descriptor
 *
 * @return Number of shared semaphores.
 *
 * @note An uninitialized @a id is accepted.
 */
uint32_t tegra_hsp_ss_count(const struct tegra_hsp_id *id);

/**
 * @brief Return number of doorbells supported by hardware.
 *
 * @jama_func_req_id 12999432
 *
 * @param[in] id    Pointer to hardware interface descriptor
 *
 * @return Number of doorbells.
 *
 * @note An uninitialized @a id is accepted.
 */
uint32_t tegra_hsp_db_count(const struct tegra_hsp_id *id);

#endif
