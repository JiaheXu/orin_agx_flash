/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__BARRIERS_H
#define CPU__BARRIERS_H
#define FSP__CPU__BARRIERS_H                    1

/**
 * @file barriers.h
 * @brief Definitions for various synchronization barriers
 */

/**
 * @brief Cache Op Completed Barrier
 *
 * This function will stall the pipeline until the pending
 * cache operations have completed.
 *
 * @return none
 */
static inline void barrier_cache_op_complete(void)
{
    __asm__ __volatile__("dsb sy\n\t" : : : "memory");
}

/**
 * @brief Compiler Barrier
 *
 * This function will prevent the compiler from optimizing away
 * the portion of code that the function appears in.
 *
 * @return none
 */
static inline void barrier_compiler(void)
{
    __asm__ __volatile__("" : : : "memory");
}

/**
 * @brief Memory Order Barrier
 *
 * This function will ensure that memory operations that appear
 * after this function (in time) will be commited to memory after
 * memory operations that appear before this function (in time).
 *
 * @return none
 */
static inline void barrier_memory_order(void)
{
    __asm__ __volatile__("dmb sy\n\t" : : : "memory");
}

/**
 * @brief Memory Complete Barrier
 *
 * This function will stall the pipeline such that all pending
 * memory operations complete before the next instruction is
 * executed.
 *
 * @return none
 */
static inline void barrier_memory_complete(void)
{
    __asm__ __volatile__("dsb sy\n\t" : : : "memory");
}

/**
 * @brief Instruction Synchronization Barrier
 *
 * This function causes the pipeline to be flushed such that all
 * instructions that follow are fetched from cache or memory.  This
 * ensures that the certain operations that affect the overall state
 * of the processor are seen by instructions that follow this barrier.
 *
 * @return none
 */
static inline void barrier_instruction_synchronization(void)
{
    __asm__ __volatile__("isb sy\n\t" : : : "memory");
}

#endif
