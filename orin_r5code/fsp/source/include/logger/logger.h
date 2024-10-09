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
#ifndef LOGGER__LOGGER_H
#define LOGGER__LOGGER_H
#define FSP__LOGGER__LOGGER_H                           1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>         // for USED
#include <misc/bitops.h>
#include <misc/macros.h>             // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <logger-config.h>           // for LOG_ENABLE
#include <logger/sections-logger.h>  /* Immune from CT_ASSERT protection */
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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_DATA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

START_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")
#define MODULE_LOG_ID(_id_)     (MODULE_ID_##_id_)

#define LOG_ID(_major_, _minor_, _seq_)                                         \
    ((INSERT(0U, 30, 16, (MODULE_LOG_ID(_major_))))                             \
     | (INSERT(0U, 15, 8, _minor_))                                             \
     | (INSERT(0U, 7, 0, _seq_)))

void log_entry(const log_metadata_t * const token,
               const uint8_t n_args,
               log_args_t args);

extern uint32_t                 log_level;

#define LOG_GET_NTH_ARG(_1_, _2_, _3_, _4_, _5_, _6_, _7_, _8_, _arg_, ...)     \
    _arg_

#define LOG_COUNT_ARGS(...)                                                     \
    LOG_GET_NTH_ARG(__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0,)

#define LOG_XFIRST(_X_, ...)    _X_
#define LOG_FIRST(...)          LOG_XFIRST(##__VA_ARGS__)
#define LOG_REST(_X_, ...)      __VA_ARGS__

#define LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                     \
    (_args_)[(_n_)] = (log_arg_t)_arg_;

#define LOG_SET_ARGS_0(_args_, _n_, ...)
#define LOG_SET_ARGS_1(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)
#define LOG_SET_ARGS_2(_args_, _n_,_arg_, ...)                                  \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_1(_args_, (_n_) + 1, __VA_ARGS__)
#define LOG_SET_ARGS_3(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_2(_args_, (_n_) + 1, __VA_ARGS__)
#define LOG_SET_ARGS_4(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_3(_args_, (_n_) + 1, __VA_ARGS__)
#define LOG_SET_ARGS_5(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_4(_args_, (_n_) + 1, __VA_ARGS__)
#define LOG_SET_ARGS_6(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_5(_args_, (_n_) + 1, __VA_ARGS__)
#define LOG_SET_ARGS_7(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_6(_args_, (_n_) + 1, __VA_ARGS__)
#define LOG_SET_ARGS_8(_args_, _n_, _arg_, ...)                                 \
    LOG_SET_NTH_ARG(_args_, _n_, _arg_)                                         \
    LOG_SET_ARGS_7(_args_, (_n_) + 1, __VA_ARGS__)

#define LOG_SET_EACH_ARG(_args_, ...)                                           \
    LOG_GET_NTH_ARG(__VA_ARGS__,                                                \
                 LOG_SET_ARGS_8, LOG_SET_ARGS_7, LOG_SET_ARGS_6, LOG_SET_ARGS_5,\
                 LOG_SET_ARGS_4, LOG_SET_ARGS_3, LOG_SET_ARGS_2, LOG_SET_ARGS_1,\
                 LOG_SET_ARGS_0,) (_args_, 0, __VA_ARGS__,)

/*
 * This version of the XLOGX macro does not provide the ability to
 * set "fixed strings" as none of the other macros would fill them
 * in or have the run-time ability to set the appropriate values.
 */
#define XLOGX(_level_, _id_, _id_str_, _flags_, _fmt_, ...)                     \
    {                                                                           \
        static const char SECTION_LOG_METADATA_STRINGS xxlog_id_str[] = _id_str_;\
        static const char SECTION_LOG_METADATA_STRINGS xxlog_f_name[] = __FILE__;\
        static const char SECTION_LOG_METADATA_STRINGS xxlog_fmt_str[] = _fmt_; \
        static const log_metadata_t xxlog_metadata USED SECTION_LOG_METADATA_DATA = {\
            .id         = _id_,                                                 \
            .flags      = _flags_,                                              \
            .level      = _level_,                                              \
            .n_args     = LOG_COUNT_ARGS(__VA_ARGS__),                          \
            .id_name    = &xxlog_id_str[0],                                     \
            .file       = &xxlog_f_name[0],                                     \
            .line_no    = __LINE__,                                             \
            .fmt_string = &xxlog_fmt_str[0],                                    \
            .num_fixed_strings = 0UL,                                             \
        };                                                                      \
        log_args_t         xxlog_log_args;                                      \
        if (((_level_ & (LOG_LEVEL_ALWAYS | PRINT_LEVEL_ALWAYS)) != 0UL)          \
            || ((_level_ & log_level) != 0UL)) {                                  \
            LOG_SET_EACH_ARG(xxlog_log_args, __VA_ARGS__);                      \
            log_entry(&xxlog_metadata, LOG_COUNT_ARGS(__VA_ARGS__),             \
                      &xxlog_log_args[0]);                                      \
        }                                                                       \
    }

#define LOGX(_level_, _id_, _id_str_, _flags_, ...)                             \
    XLOGX(_level_, _id_, _id_str_, _flags_, "", __VA_ARGS__);

#define XLOG(_level_, _id_, _id_str_, _flags_, ...)                             \
    if ((LOG_ENABLE==1) && (_level_##_ENABLE==1)) {                             \
        LOGX(_level_, _id_, _id_str_, _flags_, __VA_ARGS__);                    \
    }

#define LOG(_level_, _id_,  _flags_, ...)                                       \
    if (LOG_ENABLE==1) {                                                        \
        LOGX(_level_, _id_, #_id_, _flags_, __VA_ARGS__);                       \
    }

#define LOG_ERROR_ALWAYS(_id_, ...)                                                    \
    LOG(LOG_LEVEL_ERROR, _id_, LOG_FL_ERROR, __VA_ARGS__);

#define LOG_ALWAYS(_id_, _flags_, ...)                                          \
    XLOG(LOG_LEVEL_ALWAYS, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_PERF(_id_, _flags_, ...)                                            \
    XLOG(LOG_LEVEL_PERF, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_ERROR(_id_, _flags_, ...)                                           \
    XLOG(LOG_LEVEL_ERROR, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_CRIT(_id_, _flags_, ...)                                            \
    XLOG(LOG_LEVEL_CRIT, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_WARN(_id_, _flags_, ...)                                            \
    XLOG(LOG_LEVEL_WARN, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_INFO(_id_, _flags_, ...)                                            \
    XLOG(LOG_LEVEL_INFO, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_IRQ(_id_, _flags_, ...)                                             \
    XLOG(LOG_LEVEL_IRQ, _id_, #_id_, _flags_, __VA_ARGS__)

#define LOG_BOOT(_id_, _flags_, ...)                                            \
    XLOG(LOG_LEVEL_BOOT, _id_, #_id_,_flags_, __VA_ARGS__)

#define LOG_DEBUG(_id_, _flags_, ...)                                           \
    XLOG(LOG_LEVEL_DEBUG, _id_, #_id_, _flags_, __VA_ARGS__)

#define XPRINTX(_level_, _fmt_, ...)                                            \
    XLOGX(_level_, DCE_LOG_PRINT, "",  LOG_FL_PRINT, _fmt_, __VA_ARGS__);

#define XPRINT(_level_, _fmt_, ...)                                             \
    if ((PRINT_ENABLE==1) && (_level_##_ENABLE==1)) {                           \
        XPRINTX(_level_, _fmt_, __VA_ARGS__);                                   \
    }

#define LOGGING_PRINT(_level_, _fmt_, ...)                                              \
    if (PRINT_ENABLE==1) {                                                      \
        XPRINTX(_level_, _fmt_, __VA_ARGS__);                                   \
    }

#define print(_fmt_, ...)                                                       \
    XPRINT(PRINT_LEVEL_ALWAYS, _fmt_, __VA_ARGS__);

#define PRINT_ALWAYS(_fmt_, ...)                                                \
    XPRINT(PRINT_LEVEL_ALWAYS, _fmt_, __VA_ARGS__);

#define PRINT_PERF(_fmt_, ...)                                                  \
    XPRINT(PRINT_LEVEL_PERF, _fmt_, __VA_ARGS__);

#define PRINT_ERROR(_fmt_, ...)                                                 \
    XPRINT(PRINT_LEVEL_ERROR, _fmt_, __VA_ARGS__);

#define PRINT_CRIT(_fmt_, ...)                                                  \
    XPRINT(PRINT_LEVEL_CRIT, _fmt_, __VA_ARGS__);

#define PRINT_WARN(_fmt_, ...)                                                  \
    XPRINT(PRINT_LEVEL_WARN, _fmt_, __VA_ARGS__);

#define PRINT_INFO(_fmt_, ...)                                                  \
    XPRINT(PRINT_LEVEL_INFO, _fmt_, __VA_ARGS__);

#define PRINT_IRQ(_fmt_, ...)                                                   \
    XPRINT(PRINT_LEVEL_IRQ, _fmt_, __VA_ARGS__);

#define PRINT_BOOT(_fmt_, ...)                                                  \
    XPRINT(PRINT_LEVEL_BOOT, _fmt_, __VA_ARGS__);

#define PRINT_DEBUG(_fmt_, ...)                                                 \
    XPRINT(PRINT_LEVEL_DEBUG, _fmt_, __VA_ARGS__);

void log_set_level_mask(const uint32_t mask);
uint32_t log_get_level_mask(void);

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")
#endif
