/* Copyright (c) 2019-2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef I2C__I2C_ERRORS_H
#define I2C__I2C_ERRORS_H
#define FSP__I2C__I2C_ERRORS_H                          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>	// IWYU pragma: export
                                // IWYU pragma: no_include "base/module-id.h"
#include <misc/macros.h>        // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief Defines I2C unit specific errors. +
 * Refer include/base/module-id.h for MODULE_ERROR definition.
 *
 * @macro-title I2C Errors
 *
 * @E_I2C_ARB_LOST           Arbitration error happened during data transfer.
 * @E_I2C_NACK               Slave device has not acknowledged the transfer.
 * @E_I2C_IO                 IO error happened during I2C function execution.
 * @E_I2C_INVALID_OPS        HW specific operations are invalid.
 * @E_I2C_RX_FIFO_UNDERFLOW  RX Fifo underflow during a transaction.
 * @E_I2C_TX_FIFO_OVERFLOW   TX Fifo overflow during a transaction.
 * @E_I2C_EVENT_SIGNAL_FAIL  msg complete signal failed.
 * @E_I2C_SCL_BUSY           I2C SCL HW bus line is busy.
 * @E_I2C_SDA_BUSY           I2C SDA HW bus line is busy.
 * @E_I2C_BUS_BUSY           I2C HW bus line is busy. Transaction in progress.
 */
#define E_I2C_ARB_LOST                   MODULE_ERROR(I2C, 1UL)
#define E_I2C_NACK                       MODULE_ERROR(I2C, 2UL)
#define E_I2C_IO                         MODULE_ERROR(I2C, 3UL)
#define E_I2C_INVALID_OPS                MODULE_ERROR(I2C, 4UL)
#define E_I2C_RX_FIFO_UNDERFLOW          MODULE_ERROR(I2C, 5UL)
#define E_I2C_TX_FIFO_OVERFLOW           MODULE_ERROR(I2C, 6UL)
#define E_I2C_EVENT_SIGNAL_FAIL          MODULE_ERROR(I2C, 7UL)
#define E_I2C_SCL_BUSY                   MODULE_ERROR(I2C, 8UL)
#define E_I2C_SDA_BUSY                   MODULE_ERROR(I2C, 9UL)
#define E_I2C_BUS_BUSY                   MODULE_ERROR(I2C, 10UL)
#define E_I2C_FIFO_STATUS                MODULE_ERROR(I2C, 11UL)
#define E_I2C_STALE_INT                  MODULE_ERROR(I2C, 12UL)
#define E_I2C_XFER_COMPLETE              MODULE_ERROR(I2C, 13UL)
#define E_I2C_RX_FIFO_ERROR              MODULE_ERROR(I2C, 14UL)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif
