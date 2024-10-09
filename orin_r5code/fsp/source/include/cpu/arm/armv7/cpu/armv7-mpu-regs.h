/*
 * Copyright (c) 2015-2022, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CPU__ARMV7_MPU_REGS_H
#define CPU__ARMV7_MPU_REGS_H
#define FSP__CPU__ARMV7_MPU_REGS_H               1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/nvrm_drf.h>
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <cpu/armv7-regs.h>
#include <cpu/armv7-mpu.h>
#include <cpu/built-ins.h>

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
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__BUILT_INS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file armv7-mpu-regs.h
 * @brief Functions that provide access to MPU specific registers.
 */

/**
 * @brief Read MPU Region Number Register
 *
 * Returns the contents of the MPU Region number register.  This register
 * selects the particular MPU region registers will be accessed.
 *
 * @pre None
 *
 * @return MPU Region Number Register
 */
static inline uint32_t
r5mpu_rd_rgnr(void)
{
    uint32_t    rgnr;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mrc p15, 0, %0, c6, c2, 0\n\t" : "=r" (rgnr));

    return rgnr;
}

/**
 * @brief Write MPU Region Number Register
 *
 * Writes the contents of the MPU Region Number Register.  This register
 * selects which MPU region registers will be accessed.
 *
 * @pre None
 *
 * @param[in]   region  Region number to be written into the MPU Region
 *                      Number Register.
 *
 * @return None
 */
static inline void
r5mpu_wr_rgnr(uint32_t region)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mcr p15, 0, %0, c6, c2, 0\n\t" : : "r" (region));
}

/**
 * @brief Read MPU Region Base Address Register
 *
 * Reads the contents of the MPU Region Base Address Register.  This register
 * must have been previously selected by writing to the MPU Region Number
 * Register in order to select the correct region.
 *
 * @pre None
 *
 * @return Contents of the MPU Region Base Address Register
 */
static inline uint32_t
r5mpu_rd_bar(void)
{
    uint32_t    bar;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mrc p15, 0, %0, c6, c1, 0\n\t" : "=r" (bar));

    return bar;
}

/**
 * @brief Write MPU Region Base Address Register
 *
 * Writes the contents of the MPU Region Base Address Register.  This register
 * must have been previously selected by writing to the MPU Region Number
 * Register in order to select the correct region.
 *
 * @pre None
 *
 * @param[in]   bar  Base Address that is to be written into the prevously
 *                   selected MPU region.
 *
 * @return None
 */
static inline void
r5mpu_wr_bar(uint32_t bar)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mcr p15, 0, %0, c6, c1, 0\n\t" : : "r" (bar));
}

/**
 * @brief Read MPU Region Size and Enable Register
 *
 * Reads the contents of the MPU Region Size and Enable Register.  This register
 * must have been previously selected by writing to the MPU Region Number
 * Register in order to select the correct region.
 *
 * @pre None
 *
 * @return Contents of the MPU Region Size and Enable Register
 */
static inline uint32_t
r5mpu_rd_size(void)
{
    uint32_t    size;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mrc p15, 0, %0, c6, c1, 2\n\t" : "=r" (size));

    return size;
}

/**
 * @brief Write MPU Region Size and Enable Register
 *
 * Writes the contents of the MPU Region Size and Enable Register.  This register
 * must have been previously selected by writing to the MPU Region Number
 * Register in order to select the correct region.
 *
 * @pre None
 *
 * @param[in]   size  Size and enable that is to be written into the prevously
 *                    selected MPU region.
 *
 * @return None
 */
static inline void
r5mpu_wr_size(uint32_t size)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mcr p15, 0, %0, c6, c1, 2\n\t" : : "r" (size));
}

/**
 * @brief Read MPU Region Access Control Register
 *
 * Reads the contents of the MPU Region Access Control Register.  This register
 * must have been previously selected by writing to the MPU Region Number
 * Register in order to select the correct region.
 *
 * @pre None
 *
 * @return Contents of the MPU Region Access Control Register
 */
static inline uint32_t
r5mpu_rd_access(void)
{
    uint32_t    access;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mrc p15, 0, %0, c6, c1, 4\n\t" : "=r" (access));

    return access;
}

/**
 * @brief Write MPU Region Access Control Register
 *
 * Writes the contents of the MPU Region Access Control Register.  This register
 * must have been previously selected by writing to the MPU Region Number
 * Register in order to select the correct region.
 *
 * @pre None
 *
 * @param[in]   access  Access Control that is to be written to the previously
 *                      selected MPU region.
 *
 * @return None
 */
static inline void
r5mpu_wr_access(uint32_t access)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mcr p15, 0, %0, c6, c1, 4\n\t" : : "r" (access));
}

/**
 * @brief Configures a specific MPU region
 *
 * Sets the indicated MPU region's registers:
 *   - MPU Region Base Address
 *   - MPU Region Size and Enable
 *   - MPU Region Access Control
 *
 * The values passed as arguments to this function must be according to
 * the values specified in Section 4.3.21 of the ARM Cortex R5 Technical
 * Reference Manual.
 *
 * @pre MPU region being configured must have been disabled prior to using
 *      this function.
 *
 * @param[in]   region  MPU region number to be configured
 * @param[in]   base    base address of the region
 * @param[in]   size    encoded size of the region
 * @param[in]   access  access permissions for the region
 *
 * @return None
 */
static inline void
r5mpu_region_config(uint32_t region,
                    uint32_t base,
                    uint32_t size,
                    uint32_t access)
{
    r5mpu_wr_rgnr(region);
    r5mpu_wr_bar(base);
    r5mpu_wr_access(access);
    r5mpu_wr_size(size);
}

/**
 * @brief Disable a specific MPU region
 *
 * This function will disable the indicated MPU region.  To re-enable the
 * region, r5MPU_region_config() must be called specifying all of the values.
 *
 * @pre None
 *
 * @param[in]  region  MPU region number to be disabled.
 *
 * @return None
 */
static inline void
r5mpu_region_disable(uint32_t region)
{
    r5mpu_wr_rgnr(region);
    r5mpu_wr_size(0UL);
}

/**
 * @brief Enable the MPU
 *
 * This function will enable the MPU.  The various MPU regions must have
 * been setup prior to this function being called.
 *
 * @pre None
 *
 * @return None
 */
static inline void
r5mpu_enable(void)
{
    uint32_t sctlr;

    sctlr = rd_sctlr();
    sctlr = NV_FLD_SET_DRF_NUM(R5, SCTLR, BR, 0U, sctlr);
    sctlr = NV_FLD_SET_DRF_NUM(R5, SCTLR, M, 1U, sctlr);
    wr_sctlr(sctlr);
}

/**
 * @brief Disable the MPU
 *
 * This function will disable the MPU. This restores the MPU's default mapping.
 * Careful attention must be paid to code layout so that execution can
 * successfully continue after this call executes.
 *
 * @pre None
 *
 * @return None
 */
static inline void
r5mpu_disable(void)
{
    uint32_t sctlr;

    sctlr = rd_sctlr();
    sctlr = NV_FLD_SET_DRF_NUM(R5, SCTLR, M, 0U, sctlr);
    wr_sctlr(sctlr);
}

/**
 * @brief Convert a size to an MPU encoding
 *
 * Convert a region size in bytes to the MPU's register encoding of the region
 * size. The supplied number of bytes must have a single bit set, and be at
 * least 32. The intention is for this function call to result in a compile-
 * time constant where possible, hence its presence in the header file.
 *
 * @pre None
 *
 * @param[in]  size_bytes  The number of bytes to be covered by the region
 *
 * @return MPU encoded size value
 */
static inline uint32_t
r5mpu_size(uint32_t size_bytes)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    return 31UL - arm_builtin_clz(size_bytes - 1U);
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
