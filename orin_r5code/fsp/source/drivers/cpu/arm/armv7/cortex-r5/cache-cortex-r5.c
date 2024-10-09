/*
 * Copyright (c) 2016-2021, NVIDIA CORPORATION. All rights reserved.
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
#include <stdint.h>                        // for uint32_t
#include <string.h>                        // for size_t

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <cpu/type-conversion.h>           // for fsp_c_v_ptr_to_uptr, FSP...
#include <misc/attributes.h>               // for UNUSED_NONCONST_PTR
#include <misc/bitops.h>                   // for BIT, EXTRACT, FSP__MISC__B...
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <cpu/armv7-cache.h>               // for bpiall, CACHE_LINE_SIZE
#include <cpu/armv7-regs.h>                // for rd_sctlr, wr_sctlr, rd_actlr
#include <cpu/barriers.h>                  // for barrier_cache_op_complete
#include <cpu/cache.h>                     // for FSP__CPU__CACHE_H
#include <cpu/sections-cache.h>            // Immune from CT_ASSERT protection

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_CACHE_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__BARRIERS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__CACHE_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief Convert address/length to number of cache lines
 *
 * This function will report the number of lines (including partial lines)
 * that are covered by the range described by addr and length.
 *
 * @param[in] addr:    Address of start of range
 * @param[in] length:  number of bytes in range
 *
 * @return number of cache lines covered by the range
 */
static inline uint32_t
cache_len_to_lines(const uint32_t addr, const size_t length)
{
    uint32_t    n_lines;

    /*
     * Compute how many cache lines are covered by the data.
     *
     * It takes into account where the data lies within the cache
     * line as the data may occupy an additional cache line based
     * upon where it starts in the cache line.
     */

    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    n_lines = ((addr & (~CACHE_LINE_MASK)) + length + CACHE_LINE_SIZE - 1UL)
              / CACHE_LINE_SIZE;

    return n_lines;
}

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
 * @return none
 */
SECTION_CACHE_TEXT void
dcache_clean(const void * const base, const size_t length)
{
    uint32_t            addr;
    uint32_t            nlines;

    addr = fsp_c_v_ptr_to_uptr(base);

    /*
     * Compute how many cache lines are covered by the data that
     * need to be cleaned.
     */
    nlines = cache_len_to_lines(addr, length);

    /*
     * Align the address to the start of a cache line.
     */
    addr &= CACHE_LINE_MASK;

    while (nlines != 0UL) {
        dccmvac(addr);
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        addr += CACHE_LINE_SIZE;
        nlines -= 1UL;
    }

    barrier_cache_op_complete();
}

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
 * @return none
 */
SECTION_CACHE_TEXT void
dcache_invalidate(void *const base, const size_t length)
{
    uint32_t            addr;
    uint32_t            nlines;

    UNUSED_NONCONST_PTR(base);
    addr = fsp_c_v_ptr_to_uptr(base);

    /*
     * Compute how many cache lines are covered by the data that
     * need to be invalidated.
     */
    nlines = cache_len_to_lines(addr, length);

    /*
     * Align the address to the start of a cache line
     */
    addr &= CACHE_LINE_MASK;

    while (nlines != 0UL) {
        dcimvac(addr);
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        addr += CACHE_LINE_SIZE;
        nlines -= 1UL;
    }

    barrier_cache_op_complete();
}

/**
 * @brief Invalidate entire dCache
 *
 * This function will cause the entire dCache to be invalidated
 *
 * @return none
 */
SECTION_CACHE_TEXT void
dcache_invalidate_all(void)
{
    dcivall();

    barrier_cache_op_complete();
}

/**
 * @brief Invalidate the entire iCache
 *
 * This function will cause the entire icache contents to be invalidated.
 *
 * @return none
 */
SECTION_CACHE_TEXT void
icache_invalidate_all(void)
{
    iciallu();
    barrier_cache_op_complete();
    bpiall();
}

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
 * @return none
 */
SECTION_CACHE_TEXT void
icache_invalidate(void * const base, const size_t length)
{
    uint32_t            addr;
    uint32_t            nlines;

    UNUSED_NONCONST_PTR(base);
    addr = fsp_c_v_ptr_to_uptr(base);

    /*
     * Compute how many cache lines are covered by the code that
     * need to be invalidated.
     */
    nlines = cache_len_to_lines(addr, length);

    /*
     * Align the address to the start of a cache line
     */
    addr &= CACHE_LINE_MASK;

    while (nlines != 0UL) {
        icimvau(addr);
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        addr += CACHE_LINE_SIZE;
        nlines -= 1UL;
    }

    barrier_cache_op_complete();
    bpiall();
}

/**
 * @brief Clean entire dCache
 *
 * This function will cause all of the dirty lines in the dCache to be
 * "cleaned".  That is, any dirty lines in the dCache will be written
 * back to DRAM.
 *
 * This will only clean the L1 dCache as the R5 only has L1 caches.
 *
 * @return none
 */
SECTION_CACHE_TEXT void
dcache_clean_all(void)
{
    uint32_t    ccsidr;
    uint32_t    num_ways;
    uint32_t    num_sets;
    uint32_t    way;
    uint32_t    set;
    uint32_t    dccsw_param;

    /*
     * Write CSSELR (Cache Size SELection Register)
     * Note that this implicitly selects the data cache
     * which is OK since we're cleaning the data cache.
     */
    wr_csselr(0UL);
    barrier_instruction_synchronization();

    /* Read CCSIDR (CaChe Size ID Register) */
    ccsidr = rd_ccsidr();

    /*
     * Note: num_ways *must* be less than or equal to 4 in order for
     * the shift in setting up the dccsw_param to work properly.
     * There cannot be a run-time check for this (e.g. an assert) because
     * that would introduce a cyclic dependency:
     *          assert-->abort-->dcache_clean_all-->assert-->...
     *
     * This shouldn't be a problem except if this code is moved to something
     * other than an R5.  The R5 will always have num_ways <= 4.
     */
    num_ways = EXTRACT(ccsidr, 12U, 3U, uint32_t) + 1UL;
    num_sets = EXTRACT(ccsidr, 27U, 13U, uint32_t) + 1UL;

    for (set = 0UL; set < num_sets; set += 1UL) {
        for (way = 0UL; way < num_ways; way += 1UL) {
            dccsw_param = way << 30;
            dccsw_param |= set << 5;

            dccsw(dccsw_param);

        }
    }

    barrier_cache_op_complete();
}

/**
 * @brief Disable all caches
 *
 * This function will disable the iCache and dCache.  It will also ensure
 * that any dirty lines in the dCache have been completely written to DRAM
 * before this function returns.
 *
 * @return none
 */
SECTION_CACHE_INIT_TEXT void
cache_disable(void)
{
    uint32_t    sctlr;

    sctlr = rd_sctlr();

    if ((sctlr & BIT(2)) != 0U) {
        dcache_clean_all();
    }

    sctlr &= ~BIT(12);          /* instruction cache disable */
    sctlr &= ~BIT(2);            /* data cache disable */

    barrier_memory_complete();
    wr_sctlr(sctlr);

    barrier_instruction_synchronization();
}

/**
 * @brief Enable all caches
 *
 * This function will enable the iCache and dCache.  It will ensure that
 * the contents of the caches are invalid (e.g. there is no data or instructions
 * in the cache) before enabling the caches.
 *
 * @return none
 */
SECTION_CACHE_INIT_TEXT void
cache_enable(void)
{
    uint32_t sctlr;

    sctlr = rd_sctlr();

    sctlr |= BIT(12);           /* instruction cache enable */
    sctlr |= BIT(2);            /* data cache enable */

    barrier_memory_complete();

    dcache_invalidate_all();
    icache_invalidate_all();
    bpiall();

    wr_sctlr(sctlr);
    barrier_instruction_synchronization();
}

/**
 * @brief Enable all caches with ECC enabled
 *
 * This function will enable the iCache and dCache with ECC enabled.  It
 * follows the Sequence Reference 8.5.5. of Cortex R5 TRM (Disabling or
 * enabling error checking).
 *
 * @return none
 */
SECTION_CACHE_INIT_TEXT void
cache_enable_ecc(void)
{
        uint32_t actlr;

        cache_disable();

        actlr = rd_actlr();
        actlr &= ~(0x7UL << 3);
        actlr |= 0x5UL << 3;    // do not generate abort on ECC errors,
                                // enable HW error recovery

        /*
         * WAR for ARM R5 Erratum 780125
         * See nvbugs 200322274 for details
         *
         * Dsiable write burst in the AXI master (DBWR: bit 14)
         */
        actlr |= BIT(14);
        wr_actlr(actlr);

        barrier_memory_complete();

        cache_enable();
}

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
 * @return none
 *
 * @deprecated dcache_clean() should be used instead.
 */
SECTION_CACHE_TEXT void
cache_clean(const void * const base, size_t length)
{
    dcache_clean(base, length);
}

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
 * @return none
 *
 * @deprecated dcache_invalidate() should be used instead.
 */
SECTION_CACHE_TEXT void
cache_invalidate(void * const base, size_t length)
{
    dcache_invalidate(base, length);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
