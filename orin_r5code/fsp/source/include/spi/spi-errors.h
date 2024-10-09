/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef SPI__SPI_ERRORS_H
#define SPI__SPI_ERRORS_H
#define FSP__SPI__SPI_ERRORS_H                          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>     // IWYU pragma: export
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
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @file uart-errors.h
 * @brief Error codes that are specific to the SPI
 */

#define E_SPI_NULL_PTR                 MODULE_ERROR(SPI, 0UL)
#define E_SPI_INVALID_PARAM            MODULE_ERROR(SPI, 1UL)
#define E_SPI_ERR_CONFIG               MODULE_ERROR(SPI, 2UL)
#define E_SPI_INIT_FAIL                MODULE_ERROR(SPI, 3UL)
#define E_SPI_NO_INIT                  MODULE_ERROR(SPI, 4UL)
#define E_SPI_REINIT                   MODULE_ERROR(SPI, 5UL)
#define E_SPI_CLK_EN                   MODULE_ERROR(SPI, 6UL)
#define E_SPI_CLK_RST                  MODULE_ERROR(SPI, 7UL)
#define E_SPI_CLK_SET_RATE             MODULE_ERROR(SPI, 8UL)
#define E_SPI_FIFO_NON_EMPTY           MODULE_ERROR(SPI, 9UL)
#define E_SPI_XFER_NOT_SUPPORTED       MODULE_ERROR(SPI, 10UL)
#define E_SPI_DMA_XFER_FAIL            MODULE_ERROR(SPI, 11UL)
#define E_SPI_XFER_ERR                 MODULE_ERROR(SPI, 12UL)
#define E_SPI_INVALID_CLK_RATE         MODULE_ERROR(SPI, 13UL)
#define E_SPI_CTLR_BUSY                MODULE_ERROR(SPI, 14UL)
#define E_SPI_PORT_INIT_FAIL           MODULE_ERROR(SPI, 15UL)
#define E_SPI_PORT_DMA_INIT_FAIL       MODULE_ERROR(SPI, 16UL)
#define E_SPI_PORT_SYNC_TIMEOUT        MODULE_ERROR(SPI, 17UL)
#define E_SPI_FD_MODE_NO_SUPPORT       MODULE_ERROR(SPI, 18UL)

#endif  /* SPI__SPI_ERRORS_H */
