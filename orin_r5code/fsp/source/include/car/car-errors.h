/* Copyright (c) 2020-2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef CAR__CAR_ERRORS_H
#define CAR__CAR_ERRORS_H
#define FSP__CAR__CAR_ERRORS_H          1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <base/module-id.h>             // IWYU pragma: export
                                        // IWYU pragma: no_include "base/module-id.h"

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")

/**
 * @file car-errors.h
 * @brief Error codes that are specific to CAR
 */

/*
 * @brief Macro definitions for errors from CAR layer
 *
 * @macro-title CAR error codes
 *
 * @E_CAR_NULL_PTR                      Error value returned if input pointer
 *                                      is NULL.
 * @E_CAR_INVALID_PARAM                 Error value returned if input value is
 *                                      invalid.
 * @E_CAR_INVALID_RESET_ID              Error value returned if input reset ID
 *                                      is invalid.
 * @E_CAR_INVALID_CLOCK_ID              Error value returned if input clock ID
 *                                      is invalid.
 * @E_CAR_INVALID_FMON_ID               Error value returned if input frequency
 *                                      monitor ID is invalid.
 * @E_CAR_PLL_NO_INIT                   Error value returned if PLL has not
 *                                      been initialized yet.
 * @E_CAR_PLL_NO_LOCK                   Error value returned if PLL does not
 *                                      lock after being enabled.
 * @E_CAR_CLOCK_OP_NOT_SUPPORTED        Error value returned if the clock does
 *                                      not support the operation.
 * @E_CAR_CLOCK_HZ_NOT_SUPPORTED        Error value returned if the clock does
 *                                      not support the requested frequency.
 * @E_CAR_FMON_HW_LOGIC                 Error value returned if the FMON register
 *                                      logic algorithm failed.
 * @E_CAR_FMON_HW_TIMEOUT               Error value returned if the FMON HW did not
 *                                      respond correctly within the allocated time.
 * @E_CAR_FMON_FAULT                    Error value returned when a FMON fault is
 *                                      detected.
 * @E_CAR_FMON_NO_DATA                  Error value returned when the FMON HW has no
 *                                      data to work with. This is typical when it
 *                                      first starts.
 */
#define E_CAR_NULL_PTR                  MODULE_ERROR(CAR, 1UL)
#define E_CAR_INVALID_PARAM             MODULE_ERROR(CAR, 2UL)
#define E_CAR_INVALID_RESET_ID          MODULE_ERROR(CAR, 3UL)
#define E_CAR_INVALID_CLOCK_ID          MODULE_ERROR(CAR, 4UL)
#define E_CAR_INVALID_FMON_ID           MODULE_ERROR(CAR, 5UL)
#define E_CAR_PLL_NO_INIT               MODULE_ERROR(CAR, 6UL)
#define E_CAR_PLL_NO_LOCK               MODULE_ERROR(CAR, 7UL)
#define E_CAR_CLOCK_OP_NOT_SUPPORTED    MODULE_ERROR(CAR, 8UL)
#define E_CAR_CLOCK_HZ_NOT_SUPPORTED    MODULE_ERROR(CAR, 9UL)
#define E_CAR_FMON_HW_LOGIC             MODULE_ERROR(CAR, 10UL)
#define E_CAR_FMON_HW_TIMEOUT           MODULE_ERROR(CAR, 11UL)
#define E_CAR_FMON_FAULT                MODULE_ERROR(CAR, 12UL)
#define E_CAR_FMON_NO_DATA              MODULE_ERROR(CAR, 13UL)
#define E_CAR_ERROR_N                   MODULE_ERROR(CAR, 14UL) /* must be the number of errors */
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif /* CAR__CAR_ERRORS_H */

