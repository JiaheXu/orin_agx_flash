/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef GPCDMA__GPCDMA_ERRORS_H
#define GPCDMA__GPCDMA_ERRORS_H
#define FSP__GPCDMA__GPCDMA_ERRORS_H                          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
// IWYU pragma: no_include "base/module-id.h"
#include <base/module-id.h>         // IWYU pragma: export
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file gpcdma-errors.h
 * @brief Error codes that are specific to the gpcdma
 */

#define E_GPCDMA_NULL_PTR               MODULE_ERROR(GPCDMA, 0UL)
#define E_GPCDMA_INVALID_PARAM          MODULE_ERROR(GPCDMA, 1UL)
#define E_GPCDMA_INVALID_XFER_DIR       MODULE_ERROR(GPCDMA, 2UL)
#define E_GPCDMA_CHAN_BUSY              MODULE_ERROR(GPCDMA, 3UL)
#define E_GPCDMA_NO_CALLBACK_ASYNC      MODULE_ERROR(GPCDMA, 4UL)
#define E_GPCDMA_INVALID_XFER_SIZE      MODULE_ERROR(GPCDMA, 5UL)
#define E_GPCDMA_INVALID_XFER_SIZE_IO   MODULE_ERROR(GPCDMA, 6UL)
#define E_GPCDMA_UNALIGNED_SRC          MODULE_ERROR(GPCDMA, 7UL)
#define E_GPCDMA_UNALIGNED_DST          MODULE_ERROR(GPCDMA, 8UL)
#define E_GPCDMA_INVALID_FC_REQ_ID      MODULE_ERROR(GPCDMA, 9UL)
#define E_GPCDMA_INVALID_ADDR_WRAP      MODULE_ERROR(GPCDMA, 10UL)
#define E_GPCDMA_PORT_SYNC_TIMEOUT      MODULE_ERROR(GPCDMA, 11UL)
#define E_GPCDMA_PORT_INIT_FAIL         MODULE_ERROR(GPCDMA, 12UL)
#define E_GPCDMA_PORT_CHAN_SETUP_FAIL   MODULE_ERROR(GPCDMA, 13UL)
#define E_GPCDMA_NOT_INITED             MODULE_ERROR(GPCDMA, 14UL)
#define E_GPCDMA_CHAN_NOT_BUSY          MODULE_ERROR(GPCDMA, 15UL)
#define E_GPCDMA_INVALID_XFER_MODE      MODULE_ERROR(GPCDMA, 16UL)
#define E_GPCDMA_QUEUE_ALLOC_FAIL       MODULE_ERROR(GPCDMA, 17UL)
#define E_GPCDMA_QUEUE_OP_FAIL          MODULE_ERROR(GPCDMA, 18UL)
#define E_GPCDMA_CONT_MODE_OP_FAIL      MODULE_ERROR(GPCDMA, 19UL)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif  /* GPCDMA__GPCDMA_ERRORS_H */
