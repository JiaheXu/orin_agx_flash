/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef LOGGER__LOGGER_DATA_H
#define LOGGER__LOGGER_DATA_H
#define FSP__LOGGER__LOGGER_DATA_H                      1

/* Compiler headers */
#include <stdint.h>
#include <logger-config.h>                              /* Immune from CT_ASSERT protection */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>
#include <misc/macros.h>             // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/*
 * Note: This value can only be increased by changing several other
 * macros.  Notably _GET_NTH_ARG, COUNT_ARGS, SET_ARGS_<n> and
 * SET_EACH_ARG.
 */
#define MAX_LOG_ARGS            8

#if (LOG_ARG_WIDTH == 32)
typedef uint32_t                log_arg_t;
#else
#if (LOG_ARG_WIDTH == 64)
typedef uint64_t                log_arg_t;
#else
#error "Unsupported value for LOG_ARG_WIDTH"
#endif
#endif

typedef log_arg_t               log_args_t[MAX_LOG_ARGS];

#if (LOG_TOKEN_WIDTH == 32)
typedef uint32_t                log_token_t;
#else
#if (LOG_TOKEN_WIDTH == 64)
typedef uint64_t                log_token_t;
#else
#error "Unsupported value for LOG_TOKEN_WIDTH"
#endif
#endif

/*
 * Logging Levels
 *
 * Allows for enabling/disabling different trace points based upon
 * which levels are enabled.
 */
#define LOG_LEVEL_ALWAYS        BIT(31)
#define LOG_LEVEL_PERF          BIT(30)
#define LOG_LEVEL_ERROR         BIT(29)
#define LOG_LEVEL_CRIT          BIT(28)
#define LOG_LEVEL_WARN          BIT(27)
#define LOG_LEVEL_IRQ           BIT(26)
#define LOG_LEVEL_BOOT          BIT(25)
#define LOG_LEVEL_INFO          BIT(17)
#define LOG_LEVEL_DEBUG         BIT(16)

#define PRINT_LEVEL_ALWAYS      BIT(15)
#define PRINT_LEVEL_PERF        BIT(14)
#define PRINT_LEVEL_ERROR       BIT(13)
#define PRINT_LEVEL_CRIT        BIT(12)
#define PRINT_LEVEL_WARN        BIT(11)
#define PRINT_LEVEL_IRQ         BIT(10)
#define PRINT_LEVEL_BOOT        BIT(9)
#define PRINT_LEVEL_INFO        BIT(1)
#define PRINT_LEVEL_DEBUG       BIT(0)

#define LOG_LEVEL_ALL           MASK32(31, 16)
#define PRINT_LEVEL_ALL         MASK32(15, 0)

/*
 * Flags that indicate some general attributes about the log entry.
 */
#define LOG_FL_ERROR            BIT(31)             // error
#define LOG_FL_START            BIT(30)             // start of an operation
#define LOG_FL_END              BIT(29)             // end of an operation
#define LOG_FL_PERF             BIT(28)             // perf related log entry
#define LOG_FL_PRINT            BIT(27)             // print vs log entry
#define LOG_FL_NONE             (0U)                // no flags

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
/*
 * Log Meta-data structure
 */
typedef struct log_metadata_s {
    uint32_t            id;
    uint32_t            flags;
    uint32_t            level;
    uint32_t            n_args;
    const char          *id_name;
    const char          *file;
    uint32_t            line_no;
    const char          *fmt_string;
    uint32_t            num_fixed_strings;
#if (MAX_FIXED_STRINGS > 0)
    const char          *fixed_strings[MAX_FIXED_STRINGS];
#endif
} log_metadata_t;

/*
 * Individual Log Entry
 *
 * The delta time recorded in each log entry is the time from the previous
 * log entry.  The first log entry in a block of log entries will have
 * a delta time of 0 (it is referencing the absolute time of the block).
 */
typedef struct log_entry_s {
    uint32_t            delta_time;     // time from previous log entry
    uint16_t            n_bytes;        // number of bytes in the log entry
    uint8_t             n_fixed_args;   // number of fixed arguments that
                                        // preceed the regular arguments
                                        // that will be interpreted by the
                                        // fixed strings in the meta-data.
    uint8_t             fixed_flags;    // mapping of fixed argument(s) to
                                        // fixed strings in the meta-data.
    log_token_t         token;          // token that maps to trace_info_t
    // Any supplied arguments to the log entry will follow
} log_entry_t;

#endif
