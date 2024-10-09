/* Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef BASE__MODULE_ID_H
#define BASE__MODULE_ID_H
#define FSP__BASE__MODULE_ID_H                          1

#include <misc/macros.h>

/*
 * Module IDs are chosen so that when they are combined with a
 * subsystem's local errors, the resultant global value is a negative
 * (as in less than 0) value.
 */

START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#define MODULE_ID_RTOS          0UL
#define MODULE_ID_FDT           1UL
#define MODULE_ID_COMMON        2UL
#define MODULE_ID_ABORT         3UL
#define MODULE_ID_ASSERT        4UL
#define MODULE_ID_VIC           5UL
#define MODULE_ID_TKE           6UL
#define MODULE_ID_WDT           7UL
#define MODULE_ID_WATCHDOG      8UL
#define MODULE_ID_I2C           9UL
#define MODULE_ID_HSP           10UL
#define MODULE_ID_AST           11UL
#define MODULE_ID_LIC           12UL
#define MODULE_ID_TCU           13UL
#define MODULE_ID_SMATH         14UL
#define MODULE_ID_UART          15UL
#define MODULE_ID_GPCDMA        16UL
#define MODULE_ID_GPIO          17UL
#define MODULE_ID_SPI           18UL
#define MODULE_ID_CAR           19UL
#define MODULE_ID_COMB_UART     20UL
#define MODULE_ID_AODMIC        21UL
#define MODULE_ID_ADCC          22UL
#define MODULE_ID_SHA_NVRISCV   23UL


#define MODULE_ID_APPLICATION   64UL

/*
 * Error ID manipulation
 *
 * Note: These can't use the BIT manipulation macros because the compiler
 * will complain about shifts being out of range when used as an initializer
 * index in an array.
 */
START_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")
#define MODULE_ERROR_SIZE               (24UL)
#define MODULE_ERROR_ID(_id_)           (((uint32_t)(MODULE_ID_##_id_)) << MODULE_ERROR_SIZE)
#define MODULE_ERROR_VAL(_id_, _code_)  (MODULE_ERROR_ID(_id_) | (uint32_t)(_code_))
#define MODULE_ERROR(_id_, _code_)      ((error_t)-(int32_t)MODULE_ERROR_VAL(_id_, _code_))
#define MODULE_ERROR_CODE_MASK          ((1UL << MODULE_ERROR_SIZE) - 1UL)
#define MODULE_ERROR_ID_MASK            ((1UL << (32UL - MODULE_ERROR_SIZE)) - 1UL)
#define MODULE_ERROR_CODE(_err_)        (((uint32_t)(-(_err_))) & MODULE_ERROR_CODE_MASK)
#define MODULE_ERROR_MODULE(_err_)      ((((uint32_t)(-(_err_))) >> MODULE_ERROR_SIZE) & MODULE_ERROR_ID_MASK)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* BASE__MODULE_ID_H */
