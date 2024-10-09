/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stdint.h>               // for uint32_t
#include <string.h>               // for NULL

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/arm-vic.h>          // for arm_vic_set_isr_vect_addr, FSP__ARM...
#include <misc/linker-symbols.h>  // for IMAGE_SYM_BASE
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <error/common-errors.h>  // for error_t, E_SUCCESS

/* Module-specific FSP headers */
#include <vic/tegra-vic.h>        // for num_isr_map_entries, FSP__VIC__TEGR...
#include <vic/sections-vic.h>     // Immune from CT_ASSERT protection
#include <vic/vic-ld.h>           // for vic_isr_map_s, Image$$VIC_ISR_MAP$$...
#include <vic/vic-errors.h>       // for E_VIC_INVALID_VECTOR_ID, ....

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
CT_ASSERT(FSP__CPU__ARM_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__LINKER_SYMBOLS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__TEGRA_VIC_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__VIC_LD_H, "Header file missing or invalid.")
CT_ASSERT(FSP__VIC__VIC_ERRORS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief safe_tegra_vic_init()
 *
 * This function will initialize the various VICs and setup the vector
 * addresses for each vector.
 *
 * No input argument.
 *
 * @pre Client driver has mapped ISR for respective device
 * interrupts using isr_map_entry.
 *
 * @func_req_id 9373448
 *
 * @retval E_SUCCESS                   For success
 * @retval E_VIC_INVALID_VECTOR        ISR map entry with invalid vector ID
 * @retval E_VIC_NULL_IRQ_HANDLER      ISR map entry with NULL interrupt handler
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
SECTION_VIC_INIT_TEXT error_t tegra_safe_vic_init(void)
{
    uint32_t                    i;
    const struct vic_isr_map_s  *entry;
    uint32_t                    vic;
    uint32_t                    irq_num;
    error_t                     err = E_SUCCESS;

    for (i = 0; i < num_isr_map_entries(); i += 1U) {
        entry = &(((struct vic_isr_map_s *)&IMAGE_SYM_BASE(VIC_ISR_MAP))[i]);
        if (entry->vector > MAX_VIC_INTERRUPT_VECTOR) {
            err = E_VIC_INVALID_VECTOR;
            break;
        }
        if (entry->isr_handler == NULL) {
            err = E_VIC_NULL_IRQ_HANDLER;
            break;
        }

        vic = TEGRA_INTERRUPT_TO_VIC(entry->vector);
        irq_num = TEGRA_INTERRUPT_TO_IRQ(entry->vector);
        arm_vic_set_isr_vect_addr(vic_base_addr[vic],
                                  irq_num,
                                  entry->isr_handler);
    }

    return err;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_15_4, "Approval: Bug 200532006, DR: SWE-FSP-029-SWSADR.docx")


/**
 * @brief tegra_vic_init()
 *
 * This function will initialize the various VICs and set up the vector
 * addresses for each vector. The function skips the entries with invalid
 * value of vector id or isr_handler
 *
 * No input arguments and return value.
 *
 * @pre Client driver has mapped ISR for respective device
 * interrupts using isr_map_entry.
 *
 * @func_req_id 9373448
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx")
SECTION_VIC_INIT_TEXT void tegra_vic_init(void)
{
    uint32_t             i;
    const struct vic_isr_map_s  *entry;
    uint32_t             vic;
    uint32_t             irq_num;

    for (i = 0; i < num_isr_map_entries(); i += 1U) {
        entry = &(((struct vic_isr_map_s *)&IMAGE_SYM_BASE(VIC_ISR_MAP))[i]);
        if ((entry->vector <= MAX_VIC_INTERRUPT_VECTOR) &&
            (entry->isr_handler != NULL)) {
            vic = TEGRA_INTERRUPT_TO_VIC(entry->vector);
            irq_num = TEGRA_INTERRUPT_TO_IRQ(entry->vector);
            arm_vic_set_isr_vect_addr(vic_base_addr[vic],
                                      irq_num,
                                      entry->isr_handler);
        }
    }
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
