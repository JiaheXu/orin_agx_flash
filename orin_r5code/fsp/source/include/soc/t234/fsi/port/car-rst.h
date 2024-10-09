/* Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef FSI__CAR_RST_H
#define FSI__CAR_RST_H
#define FSP__FSI__CAR_RST_H             1

/* Compiler headers */
#include <stdint.h>                     // for uint8_t, uint64_t, uint32_t
#include <stdbool.h>                    // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <error/common-errors.h>        // for error_t

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")


/**
 * @brief FSI CAR reset IDs:
 *  Use the reset ID to represent the reset device.
 */
#define FSI_CAR_RST_ID_CAN0             1U
#define FSI_CAR_RST_ID_CAN1             2U
#define FSI_CAR_RST_ID_CHSM_DBGRESETN   3U
#define FSI_CAR_RST_ID_CHSM_NRESET      4U
#define FSI_CAR_RST_ID_CHSM_PRESETDBGN  5U
#define FSI_CAR_RST_ID_CHSM_SYSPORRESET 6U
#define FSI_CAR_RST_ID_CSITE            7U
#define FSI_CAR_RST_ID_LA               8U
#define FSI_CAR_RST_ID_NCORE_0          9U
#define FSI_CAR_RST_ID_NCORE_1          10U
#define FSI_CAR_RST_ID_NCORE_2          11U
#define FSI_CAR_RST_ID_NCORE_3          12U
#define FSI_CAR_RST_ID_NCPUPOR_0        13U
#define FSI_CAR_RST_ID_NCPUPOR_1        14U
#define FSI_CAR_RST_ID_NCPUPOR_2        15U
#define FSI_CAR_RST_ID_NCPUPOR_3        16U
#define FSI_CAR_RST_ID_NTOPRESET        17U
#define FSI_CAR_RST_ID_NPRESETDBG       18U
#define FSI_CAR_RST_ID_SPI0             19U
#define FSI_CAR_RST_ID_UART0            20U
#define FSI_CAR_RST_ID_N                21U


/**
 * @brief Get the reset assert status.
 *
 * @param rst_id The FSI_CAR_RST_ID_? define that is used to identify
 *               a reset signal.
 * @return True or false
 */
bool fsi_rst_is_asserted(uint32_t rst_id);

/**
 * @brief Assert a reset.
 *
 * @param rst_id The FSI_CAR_RST_ID_? define that is used to identify
 *               a reset signal.
 * @return E_SUCCESS
 *         E_CAR_INVALID_RESET_ID
 */
error_t fsi_rst_assert(uint32_t rst_id);

/**
 * @brief Deassert a reset.
 *
 * @param rst_id The FSI_CAR_RST_ID_? define that is used to identify
 *               a reset signal.
 * @return E_SUCCESS
 *         E_CAR_INVALID_RESET_ID
 */
error_t fsi_rst_deassert(uint32_t rst_id);

/**
 * @brief Pulse a reset with an assert, delay for the specified
 *        microseconds, and deassert.
 *
 * @param rst_id The FSI_CAR_RST_ID_? define that is used to identify
 *               a reset signal.
 * @param delay_us The delay in microseconds for the asserted
 *                 reset signal.
 * @return E_SUCCESS
 *         E_CAR_INVALID_RESET_ID
 */
error_t fsi_rst_pulse(uint32_t rst_id, uint64_t delay_us);

#endif /* FSI__CAR_RST_H */

