/*
* Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef LOCK__SMPLOCK_UP_H
#define LOCK__SMPLOCK_UP_H
#define FSP__LOCK__SMPLOCK_UP_H                      1

/**
 * @file lock/fsp-smplock.h
 * @brief functions for smplock locking primitive.
 */

/* Compiler headers */
#include <stdint.h>                           // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>                   // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/smplock-types.h>                // for arch_smplock_t
#include <misc/attributes.h>                  // for UNUSED, FSP__MISC__ATTR...
#include <misc/macros.h>                      // for END_RFD_BLOCK, START_RF...

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__CPU__SMPLOCK_TYPES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")

/**
 * @brief FSP smplock structure
 *
 * Structure that encapsulates and abstracts the architecture specific
 * smplock.
 *
 * @slock   architecture specific smplock
 */
typedef struct {
    arch_smplock_t slock;
} smplock_t;

/**
 * @brief initialization of the smplock.
 *
 * This function is a NOP for UP systems.
 *
 * @param[in] lock pointer to the FSP smplock
 *
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline void smp_lock_init(smplock_t *lock)
{
    UNUSED(lock);
}

/**
 * @brief save the interrupts and acquire the lock.
 *
 * This function is a NOP for UP systems.
 *
 * @param[in] lock   pointer to the FSP smplock
 * @param[in] flags  flags to save the irq status
 *
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline void smp_lock_irqsave(smplock_t *lock, uint32_t *flags)
{
    UNUSED(lock);
    UNUSED(flags);

    /**
     * NOTE: Ideally this function should disable the irq , save the
     * irq state wihtout doing any locking. However, these wrappers
     * are primarily added just to add SMP support for the FSP drivers.
     * For UP systems, the expectation currently is that these result
     * in a NOP. Should that change, this needs to be modified.
     */
}

/**
 * @brief restore the interrupts and release the lock.
 *
 * This function is a NOP for UP systems.
 *
 * @param[in] lock   pointer to the FSP smplock
 * @param[in] flags  flags to restore the irq status
 *
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline void smp_unlock_irqrestore(smplock_t *lock, uint32_t *flags)
{
    UNUSED(lock);
    UNUSED(flags);

    /**
     * NOTE: Ideally this function should restore the irq from the
     * saved irq state wihtout doing any locking. However, these wrappers
     * are primarily added just to add SMP support for the FSP drivers.
     * For UP systems, the expectation currently is that these result
     * in a NOP. Should that change, this needs to be modified.
     */
}

/**
 * @brief acquire the lock without any interaction with interrupts.
 *
 * This function is a NOP for UP systems.
 *
 * @param[in] lock pointer to the FSP smplock
 *
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline void smp_lock(smplock_t *lock)
{
    UNUSED(lock);
}

/**
 * @brief release the lock without any interaction with interrupts.
 *
 * This function only releases the smplock and does not interact with the
 * interrupts in anyway.
 *
 * @param[in] lock pointer to the FSP smplock
 *
 * @return None
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
static inline void smp_unlock(smplock_t *lock)
{
    UNUSED(lock);
}

END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
