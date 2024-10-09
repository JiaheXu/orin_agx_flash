/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef CPU__ARMV7_CACHE_H
#define CPU__ARMV7_CACHE_H
#define FSP__CPU__ARMV7_CACHE_H                  1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

/**
 * @file armv7-cache.h
 * @brief Assembly functions that implement the ARM V7 cache instructions
 */

#define CACHE_LINE_SIZE         32UL
#define CACHE_LINE_MASK         (~(CACHE_LINE_SIZE - 1UL))

/**
 * @brief Invalidate branch predictor all
 *
 * Allows for the BPIALL instruction to be placed in C code.  It will
 * invalidate the entire state of the branch predictor.
 *
 * @return none
 */
static inline void
bpiall(void)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c5, 6\n\t"
        : : "r" (0) : "memory"
        );
}

/**
 * @brief Invalidate branch predictor by VA
 *
 * Allows for the BPIMVA instruction to be placed in C code.  It will
 * invalidate the specified virtual address from the branch predictor.
 *
 * @param[in] addr Virtual address to be removed from the branch predictor.
 *
 * @return none
 */
static inline void
bpimva(uint32_t addr)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c5, 7\n\t"
        : : "r" (addr) : "memory"
        );
}

/**
 * @brief Instruction cache line invalidate by VA
 *
 * Allows for the ICIMVAU instruction to be placed in C code.  It will
 * invalidate the cache line that contains the specified virtual address
 * from the iCache.
 *
 * @param[in] addr Virtual address of the cache line in the iCache to
 * be invalidated.
 *
 * @return none
 */
static inline void
icimvau(uint32_t addr)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c5, 1\n\t"
        : : "r" (addr) : "memory"
        );
}

/**
 * @brief Instruction cache invalidate all
 *
 * Allows for the ICIALLU instruction to be placed in C code.  It will
 * invalidate the entire iCache.
 *
 * @return none
 */
static inline void
iciallu(void)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c5, 0\n\t"
        : : "r" (0) : "memory"
        );
}

/**
 * @brief Data cache line clean by VA to PoC
 *
 * Allows for the DCCMVAC instruction to be placed in C code.  It will
 * clean the indicated cache line.  That is, if the cache line is "dirty"
 * it will be written back to DRAM but the data will remain in the dCache.
 *
 * The "clean" is done to the point of coherence in the cache hierarchy.
 *
 * @param[in] addr Virtual address of the cache line in the dCache to
 * be invalidated.
 *
 * @return none
 */
static inline void
dccmvac(uint32_t addr)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c10, 1\n\t"
        : : "r" (addr) : "memory"
        );
}

/**
 * @brief Data cache line clean by VA to PoU
 *
 * Allows for the DCCMVAU instruction to be placed in C code.  It will
 * clean the indicated cache line.  That is, if the cache line is "dirty"
 * it will be written back to DRAM but the data will remain in the dCache.
 *
 * The "clean" is done to the point of unification in the cache hierarchy.
 *
 * @param[in] addr Virtual address of the cache line in the dCache to
 * be cleaned.
 *
 * @return none
 */
static inline void
dccmvau(uint32_t addr)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c11, 1\n\t"
        : : "r" (addr) : "memory"
        );
}

/**
 * @brief Data cache line invalidate by VA to PoC
 *
 * Allows for the DCIMVAC instruction to be placed in C code.  It will
 * invalidate the indicated cache line.
 *
 * The invalidation is done to the point of coherence.
 *
 * @param[in] addr Virtual address of the cache line in the dCache to
 * be invalidated.
 *
 * @return none
 */
static inline void
dcimvac(uint32_t addr)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c6, 1\n\t"
        : : "r" (addr) : "memory"
        );
}

/**
 * @brief Data cache line clean and invaliate by VA to PoC
 *
 * Allows for the DCCIMVAC instruction to be placed in C code.  It will first
 * clean the indicated cache line (e.g. if the line is "dirty" it will be
 * written out to DRAM) before having the cache line invalidated (e.g. removed)
 * from the dCache.
 *
 * This operation is performed to the point of coherence.
 *
 * @param[in] addr Virtual address of the cache line in the dCache to be
 * cleaned and then invalidated.
 *
 * @return none
 */
static inline void
dccimvac(uint32_t addr)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c14, 1\n\t"
        : : "r" (addr) : "memory"
        );
}

/**
 * @brief Invalidate all data caches
 *
 * Allows for the DCIVALL instruction to be placed in C code.  It will cause
 * all data caches to be invalidated.
 *
 * @return none
 */
static inline void
dcivall(void)
{
    __asm__ __volatile__(
        "mcr p15, 0, %0, c15, c5, 0\n\t"
        : : "r" (0) : "memory"
        );
}

/**
 * @brief Data cache line clean by set/way
 *
 * Allows for the DCCSW instruction to be placed in C code.  It will cause
 * the cache line specified by the set_way parameter to be cleaned (e.g. if
 * the cache line is dirty, it will be written to DRAM).
 *
 * @param[in] set_way
 * @parblock
 * This is the set and way in the cache that is to  be cleaned.
 *
 * Bits 31:30 specify the way in the dCache
 *
 * Bits 13:5 specify the set in the dCache
 * @endparblock
 *
 * @return none
 */
static inline void
dccsw(uint32_t set_way)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c7, c10, 2\n\t"
        : : "r" (set_way) : "memory"
        );
}

#endif
