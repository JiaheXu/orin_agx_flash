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

#ifndef GPIO__GPIO_ERROR_H
#define GPIO__GPIO_ERROR_H
#define FSP__GPIO__GPIO_ERROR_H   1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
// IWYU pragma: no_include "base/module-id.h"
#include <base/module-id.h>               // IWYU pragma: export
#include <misc/macros.h>                  // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @macro-title GPIO error macros
 *
 * @E_GPIO_INVALID_GPIO_IRQ           Error returned for invalid IRQ number
 * @E_GPIO_INVALID_IRQ_HANDLER        Error returned for invalid/NULL IRQ
 *                                    handler
 * @E_GPIO_INVALID_IRQ_TYPE           Error returned for invalid IRQ type
 * @E_GPIO_INVALID_IRQ_LEVEL          Error returned for invalid IRQ level
 * @E_GPIO_INVALID_GPIO               Error returned for invalid GPIO number
 * @E_GPIO_INVALID_GPIO_CHIPID        Error returned for inavlid chipid value
 * @E_GPIO_INVALID_CHIP_DATA          Error returned for invalid/NULL gpio-ops
 *                                    or chip data
 * @E_GPIO_NULL_INPUT_PARAMETER       Error returned for NULL input parameters
 */
#define E_GPIO_INVALID_GPIO_IRQ           MODULE_ERROR(GPIO, 1UL)
#define E_GPIO_INVALID_IRQ_HANDLER        MODULE_ERROR(GPIO, 2UL)
#define E_GPIO_INVALID_IRQ_TYPE           MODULE_ERROR(GPIO, 3UL)
#define E_GPIO_INVALID_IRQ_LEVEL          MODULE_ERROR(GPIO, 4UL)
#define E_GPIO_INVALID_GPIO               MODULE_ERROR(GPIO, 5UL)
#define E_GPIO_INVALID_GPIO_CTRL_ID       MODULE_ERROR(GPIO, 6UL)
#define E_GPIO_INVALID_CTRL_DATA          MODULE_ERROR(GPIO, 7UL)
#define E_GPIO_NULL_INPUT_PARAMETER       MODULE_ERROR(GPIO, 8UL)


#endif  /* GPIO__GPIO_ERRORS_H */

