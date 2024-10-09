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

#ifndef LIC__LIC_TEGRA_H
#define LIC__LIC_TEGRA_H
#define FSP__LIC__LIC_TEGRA_H                      1

/* Compiler headers */
#include <stdint.h>                     // for uint32_t
#include <stddef.h>                     // for NULL
                                        // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for error_t, FSP__ERROR__COMMON...
#include <irq/safe-irqs.h>              // for irq_callback_fn
#include <misc/macros.h>                // for END_RFD_BLOCK, START_RFD_BLOCK

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")
    CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")

/**
 * @file lic-tegra.h
 * @brief functions that provide access to various LIC registers.
 */

/**
 * @brief Maximum number of slices per LIC channel
 *
 * Defines the maximum number of slices that are available per channel.
 */
#define LIC_MAX_SLICES      11

/**
 * @brief Define a LIC interrupt
 *
 * Fills the LIC irq structure with the info associated with a LIC interrupt.
 */
#define TEGRA_LIC_INTERRUPT(_local_irq, _irq) \
    { \
        .local_irq    = (uint8_t)(_local_irq), \
        .bit          = (uint8_t)(CONCAT(NV_ADDRESS_MAP_, CONCAT(_irq, _INTR_ID)) & 31U), \
        .slice        = (uint8_t)(CONCAT(NV_ADDRESS_MAP_, CONCAT(_irq, _INTR_ID)) >> 5U), \
        .routine      = NULL, \
        .cb_data      = NULL, \
    }

/**
 * @brief Define a LIC interrupt with handler
 *
 * Fills the LIC irq structure with the info associated with a LIC interrupt.
 * This also fills the handler to support static mapping.
 */
#define TEGRA_LIC_INTERRUPT_H(_local_irq, _irq, _isr, _isr_data) \
    { \
        .local_irq    = (uint8_t)(_local_irq), \
        .bit          = (uint8_t)(CONCAT(NV_ADDRESS_MAP_, CONCAT(_irq, _INTR_ID)) & 31U), \
        .slice        = (uint8_t)(CONCAT(NV_ADDRESS_MAP_, CONCAT(_irq, _INTR_ID)) >> 5U), \
        .routine      = _isr, \
        .cb_data      = _isr_data, \
    }

/*
 * Declaration for tegra_lic_id that allows the APIs to take a pointer to it
 * without actually defining its contents here.
 */
// IWYU pragma: no_forward_declare tegra_lic_id
struct tegra_lic_id;

/**
 * @brief LIC irq info
 *
 * Structure that describes the info associated with a LIC irq.
 *
 * @local_irq    VIC irq number corresponding to the LIC channel
 * @bit          bit for the irq to be set in the respective slice
 * @slice        slice corresponding to the irq with in the LIC channel
 * @routine      handler corresponding to the irq
 * @cb_data      opaque pointer to irq handler data
 */
typedef struct {
    uint8_t          local_irq;
    uint8_t          bit;
    uint8_t          slice;
    irq_callback_fn  routine;
    void             *cb_data;
} lic_irq_t;

/**
 * @brief LIC mailbox irq handler.
 *
 * This function gets called whenever an interrupt is asserted on one of the
 * LIC lines connected to local VIC.
 *
 * @pre the function tegra_lic_init() has been called
 *
 * @param[in] data opaque pointer for LIC context.
 *
 * @return None
 */
void tegra_lic_irq_handler(void *data);

/**
 * @brief Enable LIC interrupt
 *
 * This function enables the LIC interrupt for an irq that is not directly
 * connected to the local VIC.
 *
 * @pre the function tegra_lic_init() has been called
 *
 * @param[in] id  LIC instance.
 * @param[in] irq irq to be enabled.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_LIC_NULL_PTR invalid id paramter passed
 * @retval E_LIC_NO_INTR no LIC interrupt support
 */
error_t tegra_lic_irq_enable(const struct tegra_lic_id *id,
                             uint32_t irq);

/**
 * @brief Disable LIC interrupt
 *
 * This function enables the LIC interrupt for an irq that is not directly
 * connected to the local VIC.
 *
 * @pre the function tegra_lic_init() has been called
 *
 * @param[in] id  LIC instance.
 * @param[in] irq irq to be enabled.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_LIC_NULL_PTR invalid id paramter passed
 * @retval E_LIC_NO_INTR no LIC interrupt support
 */
error_t tegra_lic_irq_disable(const struct tegra_lic_id *id,
                              uint32_t irq);
/**
 * @brief Initialize the LIC context
 *
 * This function initializes the LIC context for an instance, enables the
 * LIC interrupts.
 *
 * @param[in] id           pointer to the LIC descriptor instance.
 * @param[in] lic_map      pointer to the LIC irq mapping for the cluster
 * @param[in] lic_map_size size of the LIC irq map for the cluster
 *
 * @retval E_SUCCESS indicates success
 * @retval E_LIC_NULL_PTR       invalid id paramter passed
 * @retval E_LIC_NO_CHANS       no LIC channels supported
 * @retval E_LIC_NO_MAP         no LIC map provided
 * @retval E_LIC_NO_IRQ         no LIC irqs supported
 * @retval E_LIC_INVALID_PARAM  invalid id fields
 */
error_t tegra_lic_init(struct tegra_lic_id *id,
                       const lic_irq_t *lic_map,
                       const uint32_t lic_map_size);

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")
#endif
