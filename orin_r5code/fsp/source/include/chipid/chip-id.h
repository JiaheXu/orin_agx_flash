/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef CHIPID__CHIP_ID_H
#define CHIPID__CHIP_ID_H
#define FSP__CHIPID__CHIP_ID_H                          1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>        // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
/* Module-specific FSP headers */

/**
 * @brief Defines for the Tegra CHIP ID
 *
 * @macro-title Tegra SOC Name
 * @TEGRA_SOC_NAME_T18X Chip ID for Tegra T18x
 * @TEGRA_SOC_NAME_T19X Chip ID for Tegra T19x
 */
#define TEGRA_SOC_NAME_T18X 0x18
#define TEGRA_SOC_NAME_T19X 0x19

/**
 * @brief Defines for the Tegra Major Revision number
 *
 * @macro-title Tegra Major Revision
 * @TEGRA_MAJOR_REVISION_A Tegra Chip Major Revision 'A'
 * @TEGRA_MAJOR_REVISION_B Tegra Chip Major Revision 'B'
 */
#define TEGRA_MAJOR_REVISION_A 0x1
#define TEGRA_MAJOR_REVISION_B 0x2

/**
 * @brief Defines for the Tegra Minor Revision number
 *
 * @macro-title Tegra Minor Revision
 * @TEGRA_MINOR_REVISION_1 Tegra Chip Minor Revision number 1
 * @TEGRA_MINOR_REVISION_2 Tegra Chip Minor Revision number 2
 */
#define TEGRA_MINOR_REVISION_1 0x1
#define TEGRA_MINOR_REVISION_2 0x2

/**
 * @brief Defines for the Tegra Sub Revision
 *
 * @macro-title Tegra Sub Revision
 * @TEGRA_SUB_REVISION_P Tegra Chip Sub Revision 'P'
 * @TEGRA_SUB_REVISION_Q Tegra Chip Sub Revision 'Q'
 * @TEGRA_SUB_REVISION_R Tegra Chip Sub Revision 'R'
 */
#define TEGRA_SUB_REVISION_P 0x1
#define TEGRA_SUB_REVISION_Q 0x2
#define TEGRA_SUB_REVISION_R 0x3
#define TEGRA_SUB_NONE 0x0
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Get the Tegra chip family identifier
 *
 * @jama_func_req_id 8160164
 *
 * @retval TEGRA_SOC_NAME_T18X Tegra chip family identifier for T18X chips
 * @retval TEGRA_SOC_NAME_T19X Tegra chip family identifier for T19X chips
 */
uint8_t tegra_get_chipid(void);

/**
 * @brief Get the chip major revision number
 *
 * @jama_func_req_id 8160170
 *
 * @retval TEGRA_MAJOR_REVISION_A Tegra chip major revision is A
 * @retval TEGRA_MAJOR_REVISION_B Tegra chip major revision is B
 */
uint8_t tegra_get_major_rev(void);

/**
 * @brief Get the minor revision number
 *
 * @jama_func_req_id 8161055
 *
 * @retval TEGRA_MINOR_REVISION_1 Tegra chip minor revision is 1
 * @retval TEGRA_MINOR_REVISION_2 Tegra chip minor revision is 2
 */
uint8_t tegra_get_minor_rev(void);

/**
 * @brief Check if running on actual silicon
 *
 * @jama_func_req_id 8161334
 *
 * @retval True If the API is called on Silicon
 * @retval False If the API is called on non-silicon platform
 */
bool tegra_platform_is_silicon(void);

/**
 * @brief Check if running on fpga
 *
 * @jama_func_req_id 8161334
 *
 * @retval True If the API is called on fpga
 * @retval False If the API is called on silicon or simulation platform
 */
bool tegra_platform_is_fpga(void);

/**
 * @brief Check if running on vdk
 *
 * @jama_func_req_id 8161334
 *
 * @retval True If the API is called on VDK
 * @retval False If the API is called on silicon platform
 */
bool tegra_platform_is_vdk(void);

/**
 * @brief Get sub revision number
 *
 * @jama_func_req_id 8191178
 *
 * @retval TEGRA_SUB_REVISION_P Tegra chip sub revision is 1
 * @retval TEGRA_SUB_REVISION_Q Tegra chip sub revision is 2
 * @retval TEGRA_SUB_REVISION_R Tegra chip sub revision is 3
 * @retval TEGRA_NO_SUB_REVISION Tegra chip with no sub revision
 */
uint8_t tegra_get_sub_rev(void);

/**
 * @brief Get tegra chip sku
 *
 * @jama_func_req_id 8247959
 *
 * @retval >0 Tegra chip SKU ID
 */
uint8_t tegra_get_sku_id(void);

#endif
