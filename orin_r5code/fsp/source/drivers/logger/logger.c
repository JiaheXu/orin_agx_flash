/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stdbool.h>                 // for bool, false, true
#include <stdint.h>                  // for uint32_t, uint8_t, uint16_t, uin...
#include <string.h>                  // for NULL, memcpy

/* Early FSP headers */
#include <misc/ct-assert.h>          // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <osa/rtos-task.h>           // for rtosTaskEnterCritical, rtosTaskE...
#include <debug/assert.h>            // for ASSERT, FSP__DEBUG__ASSERT_H
#include <irq/irqs.h>                // for in_interrupt, FSP__IRQ__IRQS_H
#include <misc/attributes.h>         // for WEAK, FSP__MISC__ATTRIBUTES_H
#include <misc/bitops.h>             // for LOW32
#include <misc/macros.h>             // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <tke/tke-tegra.h>           // for tegra_tke_get_tsc64

/* Module-specific FSP headers */
#include <logger-config.h>           // Immune from CT_ASSERT protection
#include <logger/logger-private.h>   // for log_buff_t, LEB_STATE_EMPTY, LEB...
#include <logger/logger.h>           // for FSP__LOGGER__LOGGER_H, log_entry
#include <logger/sections-logger.h>  // Immune from CT_ASSERT protection
#include <logger/logger-data.h>      // for log_entry_t, log_arg_t, log_meta...

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
CT_ASSERT(FSP__OSA__RTOS_TASK_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__TKE__TKE_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_PRIVATE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOGGER__LOGGER_DATA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

log_buff_t      log_buffers[NUM_LOG_BUFFERS]            SECTION_LOG_BSS;
uint32_t        cur_buff_num            SECTION_LOG_DATA      = 0;
log_buff_t      *log_cur_buff           SECTION_LOG_DATA      = NULL;
uint32_t        dropped_log_entries     SECTION_LOG_DATA      = 0;

uint32_t log_level                      SECTION_LOG_DATA
                                        = DEFAULT_LOG_LEVEL;

/*
 * Define the print hook as a weak symbol so that if the application wants
 * to define it with an initializer, that will take precedence.
 */
log_hook_t      log_print_hook          WEAK SECTION_LOG_DATA;

/**
 * log_set_level_mask()       - set the level mask for logging
 *
 * @mask:       mask of the levels that will cause log entries to be emitted
 *
 * This function will set the mask of the levels that will allow log entries
 * to be emitted.
 *
 * Return Values:
 *      none
 */
SECTION_LOG_ADMIN_TEXT void
log_set_level_mask(const uint32_t mask)
{
    log_level = mask;
}

/**
 * log_get_level_mask()       - get the current level mask for logging
 *
 * This function will return the current mask of the levels used for logging.
 *
 * Return Values:
 *      current level mask
 */
SECTION_LOG_ADMIN_TEXT uint32_t
log_get_level_mask(void)
{
    return(log_level);
}

/**
 * log_buffer_empty()          - make a log buffer appear empty
 *
 * @buff:       pointer to the log buffer that is to be initialized to the
 *              empty state.
 *
 * This function will initialize a log_buff_t to be in the empty state.
 * This function should be called anytime that a log buffer is to be reset
 * to its empty state.
 *
 * Return Values:
 *      none.
 */
SECTION_LOG_TEXT static void
log_buffer_empty(log_buff_t * const buff)
{
    ASSERT(buff != NULL);

    buff->next = 0;
    buff->state = LEB_STATE_EMPTY;
    buff->remaining = LOG_BUFFER_SIZE;
}

/**
 * log_buffer_switch()        - switch to a new buffer if necessary
 *
 * @buff:       pointer to the current buffer
 *
 * This function determines if the current buffer can be used for storing
 * additional log entries.  If the current buffer cannot be used, it will
 * switch to using another buffer.
 *
 * If the new buffer is chosen its contents will be overwritten (that is
 * when it is chosen, the buffer will be set to the empty state).  The only
 * time that this does not occur is if the buffer is being copied.  In which
 * case any new log entries will be dropped until there is a buffer that is
 * available.
 *
 * Return Values:
 *      NULL            no buffers are available (they're in the copying
 *                      state)
 *      others          pointer to a buffer that can be used to store log
 *                      entries.
 */
SECTION_LOG_TEXT static log_buff_t *
log_buffer_switch(log_buff_t * const buff)
{
    log_buff_t          *new_buff       = buff;
    uint32_t            buff_num;

    ASSERT(buff != NULL);

    switch (new_buff->state) {
    case LEB_STATE_EMPTY:
    case LEB_STATE_PARTIAL:
        /*
         * OK to use the current buffer.
         */
        break;

    case LEB_STATE_FULL:
    case LEB_STATE_TOCOPY:
    case LEB_STATE_COPYING:
        /*
         * Have to switch to the next buffer
         */
        buff_num = (cur_buff_num + 1) % NUM_LOG_BUFFERS;
        new_buff = &log_buffers[buff_num];
        if ((new_buff->state == LEB_STATE_TOCOPY)
            || (new_buff->state == LEB_STATE_COPYING)) {
            /*
             * Cannot touch a buffer that's actively being
             * copied.  So we'll have to drop any trace points
             * until the copy has completed.
             */
            new_buff = NULL;
        } else {
            log_buffer_empty(new_buff);
            cur_buff_num = buff_num;
            log_cur_buff = new_buff;
        }
        break;
    }

    return new_buff;
}

/**
 * log_buffer_get()           - get a buffer to use to hold log entries
 *
 * This function will return a pointer to a log buffer that can be used
 * to hold log entries.
 *
 * Return Values:
 *      NULL            no buffers are available
 *      others          pointer to a buffer that can be used to store log
 *                      entries.
 */
SECTION_LOG_TEXT static log_buff_t *
log_buffer_get(void)
{
    log_buff_t  *buff   = NULL;

    /*
     * Handle initialization
     */
    if (log_cur_buff == NULL) {
        cur_buff_num = 0;
        log_cur_buff = &log_buffers[0];
        buff = log_cur_buff;
        log_buffer_empty(buff);
    } else {
        /*
         * Make sure that the current buffer can be used.
         * If not, move to another one.
         */
        buff = log_buffer_switch(log_cur_buff);
    }

    return buff;
}

/**
 * log_entry() -        add a log entry
 *
 * @token:              pointer to the meta-data structure for the log entry
 * @n_args:             number of actual args in log_args_t structure
 * @args:               pointer to a log_args_t structure
 *
 * This function will add a log entry into a log buffer.  It will construct
 * a log entry and attempt to place it in to a log buffer.
 *
 * Return Values:
 *      none
 */
SECTION_LOG_TEXT void
log_entry(const log_metadata_t * const token,
          const uint8_t n_args,
          log_args_t args)
{
    log_buff_t          *buff           = NULL;
    log_entry_t         *lep;
    uint8_t             *le_args;
    uint64_t            now;
    bool                full            = false;
    bool                from_isr;
    uint32_t            le_size;

    ASSERT(token != NULL);
    ASSERT(args != NULL);

    from_isr = in_interrupt();

    if (!from_isr) {
        rtosTaskEnterCritical();
    }

    /*
     * Determine if there is a print "hook"
     */
    if ((log_print_hook != NULL)
        && ((token->flags & LOG_FL_PRINT) != 0)) {
        (*log_print_hook)(token, n_args, args);
        goto out;
    }

    buff = log_buffer_get();
    if (buff == NULL) {
        /*
         * There are no buffers that have space for any
         * log entries, so just drop this log entry on
         * the floor.
         */
        dropped_log_entries += 1;
        goto out;
    }

    /*
     * See if there is enough space in the current buffer
     * for the new log entry.
     */
    le_size = sizeof(log_entry_t) + (n_args * sizeof(log_arg_t));
    if (le_size > buff->remaining) {
        /*
         * Mark the current buffer as full and queue it
         */
        buff->state = LEB_STATE_FULL;
        log_buffer_push(buff);

        /*
         * Get a new buffer
         */
        buff = log_buffer_get();
        if (buff == NULL) {
            /*
             * There are no buffers that have space for any
             * trace points, so just drop this trace point
             * on the floor.
             */
            dropped_log_entries += 1;
            goto out;
        }
    }

    lep = (log_entry_t *)&buff->buffer[buff->next];
    buff->next += le_size;

    if (buff->next >= LOG_BUFFER_SIZE) {
        buff->state = LEB_STATE_FULL;
        full = true;
    }

    now = tegra_tke_get_tsc64();

    if (buff->state == LEB_STATE_EMPTY) {
        buff->start_time = now;
        buff->last_time = now;
        buff->state = LEB_STATE_PARTIAL;
    }

    lep->delta_time = LOW32(now - buff->last_time);
    buff->last_time = now;

    lep->token = (log_token_t)token;
    lep->n_bytes = (uint16_t)le_size;
    lep->n_fixed_args = 0;
    lep->fixed_flags = 0;
    if (n_args > 0) {
        le_args = ((uint8_t *)lep) + sizeof(log_entry_t);
        memcpy(le_args, args, n_args * sizeof(log_arg_t));
    }

  out:
    if (!from_isr) {
        if (full) {
            log_buffer_push(buff);
        }

        rtosTaskExitCritical();
    }

    return;
}

/**
 * log_buffer_push()            - pushes a log buffer to memory
 *
 * @buff:               Buffer to be pushed
 *
 * This function will push a buffer to memory so that it is visible to
 * the rest of the system.
 *
 * This particular version is a "stub".  It does not do anything to the
 * buffer, so it will be re-used when selected again.
 * It has a "weak" linkage so that the application can implement its own
 * version.
 */
SECTION_LOG_TEXT WEAK void
log_buffer_push(log_buff_t * const buff)
{
    (void)buff;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
