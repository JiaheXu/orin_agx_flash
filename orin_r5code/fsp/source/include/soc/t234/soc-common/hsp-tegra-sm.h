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

#ifndef HSP__HSP_TEGRA_SM_H
#define HSP__HSP_TEGRA_SM_H
#define FSP__HSP__HSP_TEGRA_SM_H                           1

/* Compiler headers */
#include <stdint.h>                     // for uint32_t
#include <stdbool.h>                    // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for error_t, FSP__ERROR__COMMON...
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <hsp/hsp-tegra.h>              // hsp128_t, struct tegra_hsp_id...

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
 * @brief check if 128-bit mailbox is empty or not
 *
 * @jama_func_req_id 10709223
 *
 * This function indicates whether the 128-bit mailbox is empty or not.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index to which data has to be written.
 *            Range: [0- MAX SM INDEX of the instance]
 *
 * @retval True indicates that the mailbox is empty
 * @retval False indicates that the mailbox is not empty
 */
bool tegra_hsp_sm_128_is_empty(const struct tegra_hsp_id *id,
                               uint32_t sm);

/**
 * @brief clear contents of the 128-bit shared mailbox register
 *
 * @jama_func_req_id 10709412
 *
 * This function clears the contents of the 128-bit shared mailbox register.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 */
error_t tegra_hsp_sm_vacate_128(const struct tegra_hsp_id *id,
                                uint32_t sm);

/**
 * @brief read the 128-bit HSP shared mailbox without clearing it
 *
 * @jama_func_req_id 10709247
 *
 * this function fetches the 128-bit data field of the mailbox register
 * without modifying its contents.
 *
 * @param[in] id    HSP instance.
 * @param[in] sm    Shared mailbox index. Range: [0- MAX SM INDEX supported]
 * @param[in] data  pointer to where the 128-bit mailbox data field is
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_EMPTY_MBOX mailbox tag field is empty
 */
error_t tegra_hsp_sm_peek_128(const struct tegra_hsp_id *id,
                              uint32_t sm,
                              hsp128_t *data);

/**
 * @brief read the 128-bit HSP shared mailbox by clearing the contents
 *
 * @jama_func_req_id 10708695
 *
 * This function returns the 128-bit data field of the shared mailbox register
 * by clearing the contents of it.
 *
 * @param[in] id    HSP instance.
 * @param[in] sm    Shared mailbox index. Range: [0- MAX SM INDEX supported]
 * @param[in] data  pointer to where the 128-bit mailbox data field is
 *                  written to.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_EMPTY_MBOX mailbox tag field is empty
 */
error_t tegra_hsp_sm_consume_128(const struct tegra_hsp_id *id,
                                 uint32_t sm,
                                 hsp128_t *data);

/**
 * @brief write to the 128-bit HSP shared mailbox register with TAG bit
 *
 * @jama_func_req_id 10708650
 *
 * This function writes the mailbox register with the specified value in
 * the 128-bit data field and sets the TAG bit.
 *
 * @param[in] id HSP instance.
 * @param[in] sm Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] value value to be written to the 128-bit data field.
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 * @retval E_HSP_ERR_INVALID_DATA if 0xDEAD1001 is written due to HW bug 200395605
 */
error_t tegra_hsp_sm_produce_128(const struct tegra_hsp_id *id,
                                 uint32_t sm,
                                 hsp128_t const *value);

/**
 * @brief set HSP shared mailbox type.
 *
 * @jama_func_req_id 10710072
 *
 * This function sets the HSP shared mailbox type as to whether it is treated
 * as 32-bit or 128-bit.
 *
 * @pre the function tegra_hsp_db_init()/tegra_hsp_init() has been called
 *
 * @param[in] id   HSP instance.
 * @param[in] sm   Shared mailbox index. Range: [0- MAX SM INDEX of the instance]
 * @param[in] type Shared mailbox type. Range: [HSP_SM_TYPE_32 - HSP_SM_TYPE_128]
 *
 * @retval E_SUCCESS indicates success
 * @retval E_HSP_ERR_NULL_PTR invalid id paramter passed
 * @retval E_HSP_ERR_NO_MBOX invalid shared mailbox index
 */
error_t tegra_hsp_set_sm_type(struct tegra_hsp_id *id,
                              uint32_t sm,
                              uint32_t type);

#endif
