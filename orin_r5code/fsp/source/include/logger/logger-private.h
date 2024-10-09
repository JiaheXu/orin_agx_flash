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
#ifndef LOGGER__LOGGER_PRIVATE_H
#define LOGGER__LOGGER_PRIVATE_H
#define FSP__LOGGER__LOGGER_PRIVATE_H                   1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>             // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <logger-config.h>           /* Immune from CT_ASSERT protection */
#include <logger/logger-data.h>

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
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_DATA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/*
 * Internal definitions that are only used by the logging
 * system itself.  Should not be visible outside of that.
 */
typedef uint32_t                log_buff_state_t;

#define LEB_STATE_EMPTY         0       // buffer is empty
#define LEB_STATE_PARTIAL       1       // buffer has some log entries
#define LEB_STATE_FULL          2       // buffer is full
#define LEB_STATE_TOCOPY        3       // buffer is to be copied
#define LEB_STATE_COPYING       4       // buffer is actively being copied

/*
 * Internal log buffer
 */
typedef struct log_buff {
    uint64_t            start_time;     // time of first entry in buffer
    uint64_t            last_time;      // time of last entry in buffer
    uint32_t            next;           // next available byte in buffer
    uint32_t            remaining;      // remaining bytes in buffer
    log_buff_state_t    state;          // state of the buffer
    uint32_t            pad;            // to align next field
    uint8_t             buffer[LOG_BUFFER_SIZE];
} log_buff_t;

typedef void (*log_hook_t)(const log_metadata_t * const token,
                           const uint32_t n_args,
                           log_args_t args);

extern log_hook_t       log_print_hook;
extern void             *log_log_area;
extern log_buff_t       *log_cur_buff;
extern log_buff_t       log_buffers[NUM_LOG_BUFFERS];
extern uint32_t         cur_buff_num;

void log_buffer_push(log_buff_t * const buff);

#endif
