/*
 * Copyright (c) 2020 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef HSP__HSP_TEGRA_DEFS_H
#define HSP__HSP_TEGRA_DEFS_H
#define FSP__HSP__HSP_TEGRA_DEFS_H                      1

/* Hardware headers */
#include <address_map_new.h>        // for NV_ADDRESS_MAP_TOP0_HSP_COMMON_SIZE
#include <arhsp_dbell.h>            // for HSP_DBELL_0_ENABLE_0, HSP_DBELL_0...
#include <arhsp_int.h>              // for HSP_INT_DIMENSIONING_0_nDB_SHIFT
#include <arhsp_shrd_mbox.h>        // for HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0...
#include <arhsp_shrd_sem.h>         // for HSP_SHRD_SEM_0_SHRD_SMP_STA_0...

/* Late FSP headers */
#include <misc/macros.h>            // for START_RFD_BLOCk, END_RFD_BLOCK, INLINE_RFD

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
/**
 * @file hsp/hsp-tegra-defs.h
 * @brief defines that are internal to the HSP driver
 */

/**
 * @brief HSP commmon instance size
 *
 * Defines the HSP instance size
 */
#define HSP_COMMON_SIZE ((uint32_t)(NV_ADDRESS_MAP_TOP0_HSP_COMMON_SIZE))

/**
 * @brief HSP interrupt enable register offset
 *
 * Defines the HSP interrupt enable register offset
 */
#define HSP_INT_IE_OFFSET (HSP_INT_IE_1 - HSP_INT_IE_0)

/**
 * @brief HSP shared mailbox registers offset
 *
 * Defines the HSP shared mailbox registers offset
 */
#define HSP_SHRD_MBOX_OFFSET \
        ((uint32_t)HSP_SHRD_MBOX_MBOX_1_SHRD_MBOX_0 - (uint32_t)HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0)

#define HSP_SHRD_MBOX_FULL_IE_OFFSET \
        ((uint32_t)HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_FULL_INT_IE_0)

#define HSP_SHRD_MBOX_EMPTY_IE_OFFSET \
        ((uint32_t)HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_FULL_INT_IE_0)

/**
 * @brief HSP shared mailbox TAG field bit mask
 *
 * Defines the bit mask for the HSP shared mailbox TAG field
 */
#define HSP_SHRD_MBOX_TAG_FIELD ((uint32_t)1 << HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0_TAG_SHIFT)

#define HSP_SHRD_MBOX_TAG_FULL (HSP_SHRD_MBOX_TAG_FIELD)
#define HSP_SHRD_MBOX_TAG_EMPTY 0U

/**
 * @brief HSP shared mailbox DATA field bit mask
 *
 * Defines the bit mask for the HSP shared mailbox DATA field
 */
#define HSP_SHRD_MBOX_DATA_FIELD (HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0_DATA_FIELD)

/**
 * @brief HSP doorbell registers offset
 *
 * Defines the HSP doorbell registers offset
 */
#define HSP_DBELL_OFFSET (HSP_DBELL_1_TRIGGER_0 - HSP_DBELL_0_TRIGGER_0)

/**
 * @brief HSP dimensioning field
 *
 * Defines the bit mask to extract the specific HSP dimensioning field
 * for a HSP instance
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")
#define HSP_INT_DIMENSIONING_FIELD(regval, field) \
        (((regval) >> HSP_INT_DIMENSIONING_0_ ## field ## _SHIFT) & \
                (((uint32_t)0xf << HSP_INT_DIMENSIONING_0_ ## field ## _SHIFT) >> \
                        HSP_INT_DIMENSIONING_0_ ## field ## _SHIFT))
END_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")

/**
 * @brief shared semaphore status register offset
 *
 * Defines the HSP shared semaphore status register offset
 */
#define HSP_SHRD_SEM_STA    (HSP_SHRD_SEM_0_SHRD_SMP_STA_0)

/**
 * @brief shared semaphore set register offset
 *
 * Defines the HSP shared semaphore set register offset
 */
#define HSP_SHRD_SEM_SET    (HSP_SHRD_SEM_0_SHRD_SMP_STA_SET_0)

/**
 * @brief shared semaphore clear register offset
 *
 * Defines the HSP shared semaphore clear register offset
 */
#define HSP_SHRD_SEM_CLR    (HSP_SHRD_SEM_0_SHRD_SMP_STA_CLR_0)

/**
 * @brief shared semaphore registers offset
 *
 * Defines the HSP shared semaphore registers offset
 */
#define HSP_SHRD_SEM_OFFSET  \
    ((uint32_t)HSP_SHRD_SEM_1_SHRD_SMP_STA_0 - (uint32_t)HSP_SHRD_SEM_0_SHRD_SMP_STA_0)

/**
 * @brief HSP interrupt register mailbox full assert field
 *
 * Defines the mailbox full assert field for HSP interrupt register
 */
#define HSP_INT_IR_0_mbox_full_assert_FIELD \
    ((uint32_t)0xFFU << HSP_INT_IR_0_mbox_full_asserted_SHIFT)

/**
 * @brief invalid data pattern for HSP shared mailbox register
 *
 * Defines the invalid data pattern for the HSP shared mailbox register
 * due to the HW bug 200395605.
 */
#if defined(HW_BUG_200395605) && (HW_BUG_200395605 == 1)
    #define DEAD1001 MK_U32_CONST(0xdead1001)
#endif
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
