/*
* Copyright (c) 2020 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef LOCK__SMPLOCK_SMP_H
#define LOCK__SMPLOCK_SMP_H
#define FSP__LOCK__SMPLOCK_SMP_H                      1

/**
 * @file lock/fsp-smplock.h
 * @brief functions for smplock locking primitive.
 */

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

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
 * @param[in] lock pointer to the FSP smplock
 *
 * @return None
 */
void smp_lock_init(smplock_t *lock);

/**
 * @brief save the interrupts and acquire the lock.
 *
 * This function disables interrupts locally and provide the smplock on SMP.
 *
 * @param[in] lock   pointer to the FSP smplock
 * @param[in] flags  flags to save the irq status
 *
 * @return None
 */
void smp_lock_irqsave(smplock_t *lock,
                      uint32_t *flags);

/**
 * @brief restore the interrupts and release the lock.
 *
 * This function restores the interrupts to the state when the lock was
 * acquired and releases the lock.
 *
 * @param[in] lock   pointer to the FSP smplock
 * @param[in] flags  flags to restore the irq status
 *
 * @return None
 */
void smp_unlock_irqrestore(smplock_t *lock,
                           uint32_t *flags);

/**
 * @brief acquire the lock without any interaction with interrupts.
 *
 * This function only acquires the smplock and does not interact with the
 * interrupts in anyway.
 *
 * @param[in] lock pointer to the FSP smplock
 *
 * @return None
 */
void smp_lock(smplock_t *lock);

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
void smp_unlock(smplock_t *lock);

#endif
