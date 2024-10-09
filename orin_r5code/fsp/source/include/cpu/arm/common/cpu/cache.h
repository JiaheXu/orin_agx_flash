/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__CACHE_H
#define CPU__CACHE_H
#define FSP__CPU__CACHE_H                       1

/* Compiler headers */
#include <stddef.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

/**
 * @file cache.h
 * @brief Interfaces for performing Cache operations.
 */


/**
 * @brief Disable all caches
 *
 * This function will disable the iCache and dCache.  It will also ensure
 * that any dirty lines in the dCache have been completely written to DRAM
 * before this function returns.
 *
 * @func_req_id 8191502
 *
 * @return none
 *
 */
void cache_disable(void);

/**
 * @brief Enable all caches
 *
 * This function will enable the iCache and dCache.  It will ensure that
 * the contents of the caches are invalid (e.g. there is no data or instructions
 * in the cache) before enabling the caches.
 *
 * @func_req_id 8191127
 *
 * @return none
 */
void cache_enable(void);

/**
 * @brief Enable all caches with ECC enabled
 *
 * This function will enable the iCache and dCache with ECC enabled.  It
 * follows the Sequence Reference 8.5.5. of Cortex R5 TRM (Disabling or
 * enabling error checking).
 *
 * @func_req_id 8345060
 *
 * @return none
 */
void cache_enable_ecc(void);

/**
 * @brief Invalidate a region of iCache
 *
 * This function will cause the specified region in the iCache to be
 * invalidated.  That is, that data will no longer reside in the iCache
 * at the completion of this function.
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be invalidated than was actually specified.
 *
 * @param[in] base:   Address of start of range to be invalidated
 * @param[in] length: Number of bytes in the range to be invalidated
 *
 * @func_req_id 8345075
 *
 * @return none
 */
void icache_invalidate(void * const base, const size_t length);

/**
 * @brief Invalidate the entire iCache
 *
 * This function will cause the entire icache contents to be invalidated.
 *
 * @func_req_id 8361851
 *
 * @return none
 */
void icache_invalidate_all(void);

/**
 * @brief Clean a region of dCache
 *
 * This function will cause any "dirty" data in the dCache in the
 * specified range to be written back to memory.  The data may still
 * reside in the dCache (e.g. it was not invalidated).
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be "cleaned" than was actually specified.
 *
 * @param[in] base:   Address of start of range to be cleaned
 * @param[in] length: Number of bytes in the range to be cleaned
 *
 * @func_req_id 8362199
 *
 * @return none
 */
void dcache_clean(const void * const base, const size_t length);

/**
 * @brief Invalidate a region of dCache
 *
 * This function will cause the specified region in the dCache to be
 * invalidated.  That is, that data will no longer reside in the dCache
 * at the completion of this function.
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be invalidated than was actually specified.
 *
 * @param[in] base:   Address of start of range to be invalidated
 * @param[in] length: Number of bytes in the range to be invalidated
 *
 * @func_req_id 8361983
 *
 * @return none
 */
void dcache_invalidate(void *const base, const size_t length);

/**
 * @brief Clean entire dCache
 *
 * This function will cause all of the dirty lines in the dCache to be
 * "cleaned".  That is, any dirty lines in the dCache will be written
 * back to DRAM.
 *
 * This will only clean the L1 dCache as the R5 only has L1 caches.
 *
 * @func_req_id 8362202
 *
 * @return none
 */
void dcache_clean_all(void);

/**
 * @brief Invalidate entire dCache
 *
 * This function will cause the entire dCache to be invalidated
 *
 * @func_req_id 8362076
 *
 * @return none
 */
void dcache_invalidate_all(void);

/**
 * @brief Invalidate a region of dCache
 *
 * This function will cause the specified region in the dCache to be
 * invalidated.  That is, that data will no longer reside in the dCache
 * at the completion of this function.
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be invalidated than was actually specified.
 *
 * @param[in] base:   Address of start of range to be invalidated
 * @param[in] length: Number of bytes in the range to be invalidated
 *
 * @func_req_id 8191673
 *
 * @return none
 *
 * @deprecated dcache_invalidate() should be used instead.
 */
void cache_invalidate(void * const base, size_t length);

/**
 * @brief Clean a region of dCache
 *
 * This function will cause any "dirty" data in the dCache in the
 * specified range to be written back to memory.  The data may still
 * reside in the dCache (e.g. it was not invalidated).
 *
 * If the specified range does not align with integral numbers of
 * cache lines, more data may be "cleaned" than was actually specified.
 *
 * @param[in] base   Address of start of range to be cleaned
 * @param[in] length Number of bytes in the range to be cleaned
 *
 * @func_req_id 8191682
 *
 * @return none
 *
 * @deprecated dcache_clean() should be used instead.
 */
void cache_clean(const void * const base, size_t length);

#endif
