/*
 * Copyright (c) 2017-2022, NVIDIA CORPORATION. All rights reserved.
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

#ifndef CPU__ARMV7_REGS_H
#define CPU__ARMV7_REGS_H
#define FSP__CPU__ARMV7_REGS_H                   1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

#include <cpu/armv7-mpu.h>

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file armv7-regs.h
 * @brief Assembly functions that provide access to control registers
 */

/**
 * @brief Read Current Program Status Register
 *
 * Reads the contents of the CPSR register
 *
 * @return contents of CPSR register
 */
static inline uint32_t
rd_cpsr(void)
{
    uint32_t val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__ (
        "mrs %0, cpsr\n\t"
        : "=r" (val)
        );

    return val;
}

static inline void
wr_cpsr(uint32_t new_cpsr)
{
    __asm__ __volatile__ (
        "msr cpsr_c, %0\n\t"
        : /* no output */
        : "r"(new_cpsr)
        );
}

/**
 * @brief Read Coprocessor Access Control Register
 *
 * Reads the contents of the CPACR register.
 *
 * @return contents of CPACR register
 */
static inline uint32_t
rd_cpacr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c1, c0, 2\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Set the MPU Region Number Register
 *
 * Sets the MPU region number register so that a particular MPU region can
 * be manipulated.
 *
 * @param[in] region Region number of the MPU region to provide access to
 *
 * @return none
 */
static inline void
set_mpu_region(uint32_t region)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mcr p15, 0, %0, c6, c2, 0\n\t"
        : : "r" (region)
        );
}

/**
 * @brief Read the MPU Region Base Register
 *
 * Reads the contents of the MPU Region Base Register for the region number
 * that had been previously set.
 *
 * @return contents of the MPU Region Base Register
 */
static inline uint32_t
rd_mpu_region_base(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c6, c1, 0\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the MPU Region Access Control Register
 *
 * Reads the contents of the MPU Region Access Control Register for the region
 * number that had been previously set.
 *
 * @return contents of the MPU Region Access Control Register
 */
static inline uint32_t
rd_mpu_region_access(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c6, c1, 4\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the MPU Region Size and Enable Register
 *
 * Reads the contents of the MPU Region Size and Enable Register for the
 * region number that had been previously set.
 *
 * @return contents of the MPU Region Size and Enable Register
 */
static inline uint32_t
rd_mpu_region_size(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c6, c1, 2\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the BTCM Region Register
 *
 * Reads the contents of the BTCM Region Register which indicates the base
 * address and size of the BTCM.
 *
 * @return contents of the BTCM Regiion Register
 */
static inline uint32_t
rd_btcm_region(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c9, c1, 0\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the ATCM Region Register
 *
 * Reads the contents of the ATCM Region Register which indicates the base
 * address and size of the ATCM.
 *
 * @return contents of the ATCM Regiion Register
 */
static inline uint32_t
rd_atcm_region(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c9, c1, 1\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the Slave Port Control Register
 *
 * Reads the contents of the Slave Port Control Register.
 *
 * @return contents of the Slave Port Control Register
 */
static inline uint32_t
rd_slave_port_control(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c11, c0, 0\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the Normal AXI Peripheral Interface Region Register
 *
 * Reads the contents of the normal AXI Peripheral Interface Region Register
 *
 * @return contents of the normal AXI Peripheral Interface Region Register
 */
static inline uint32_t
rd_normal_llpp(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c15, c0, 1\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read the Virtual AXI Peripheral Interface Region Register
 *
 * Reads the contents of the virtual AXI Peripheral Interface Region Register
 *
 * @return contents of the virtual AXI Peripheral Interface Region Register
 */
static inline uint32_t
rd_virtual_llpp(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c15, c0, 2\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read AHB Peripheral Interface Register
 *
 * Reads the contents of the AHB Peripheral Interface Register
 *
 * @return contents of the AHB Peripheral Interface Register
 */
static inline uint32_t
rd_ahb_pirr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c15, c0, 3\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read Data Fault Status Register
 *
 * Reads the contents of the DFSR which contains status information
 * regarding the source of the last data abort.
 *
 * @return contents of the DFSR
 */
static inline uint32_t
rd_dfsr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c5, c0, 0\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read Data Fault Address Register
 *
 * Reads the contents of the DFAR which contains the address of the
 * fault when a synchronous abort occurs.
 *
 * @return contents of the DFAR
 */
static inline uint32_t
rd_dfar(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c6, c0, 0\n\t"
        : "=r" (val)
        );

    return val;
}


/**
 * @brief Read Instruction Fault Status Register
 *
 * Reads the contents of the IFSR which contains status information
 * regarding the source of the last prefetch abort.
 *
 * @return contents of the IFSR
 */
static inline uint32_t
rd_ifsr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c5, c0, 1\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read Instruction Fault Address Register
 *
 * Reads the contents of the IFAR which contains the address of the
 * instruction that caused a prefetch abort.
 *
 * @return contents of the IFAR
 */
static inline uint32_t
rd_ifar(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c6, c0, 2\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read Auxilary Data Fault Status Register
 *
 * Reads the contents of the ADFSR which contains information about
 * data parity, ECC and TCM errors.
 *
 * @return contents of the ADFSR
 */
static inline uint32_t
rd_adfsr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c5, c1, 0\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read Auxilary Instruction Fault Status Register
 *
 * Reads the contents of the AIFSR which contains information about
 * instruction parity, ECC and TCM errors.
 *
 * @return contents of AIFSR
 */
static inline uint32_t
rd_aifsr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c5, c1, 1\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Read Cache Level ID Register
 *
 * Reads the contents of the CLIDR register which indicates which cache
 * levels are implemented.
 *
 * @return contents of CLIDR
 */
static inline uint32_t
rd_clidr(void)
{
    uint32_t    clidr;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 1, %0, c0, c0, 1\n\t"
        : "=r" (clidr)
        );

    return clidr;
}

/**
 * @brief Read System Control Register
 *
 * Reads the contents of the SCTLR register which contains control and
 * configuration information for:
 *    memory alignment, endianness, protection and fault behavior
 *    MPU and cache enables and cache replacement strategy
 *    interrupts and the behvior of interrupt latency
 *    location of exception vectors
 *    program flow prediction
 *
 * @return contents of SCTLR
 */
static inline uint32_t
rd_sctlr(void)
{
    uint32_t    sctlr;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc    p15, 0, %0, c1, c0, 0\n\t"
        : "=r" (sctlr)
        );

    return sctlr;
}

/**
 * @brief Write System Control Register
 *
 * Writes the contents of the SCTLR register which contains control and
 * configuration information for:
 *    memory alignment, endianness, protection and fault behavior
 *    MPU and cache enables and cache replacement strategy
 *    interrupts and the behvior of interrupt latency
 *    location of exception vectors
 *    program flow prediction
 *
 * @param[in] sctrl value to be written to SCTLR register
 *
 * @return none
 */
static inline void
wr_sctlr(uint32_t sctlr)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c1, c0, 0\n\t"
        : : "r" (sctlr)
        );
}

/**
 * @brief Read Auxiliary Control Register
 *
 * Reads the contents of the ACTLR register which controls:
 *    branch prediction
 *    performance features
 *    error and parity logic
 *
 * @return contents of ACLTR
 */
static inline uint32_t
rd_actlr(void)
{
    uint32_t   actlr;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc    p15, 0, %0, c1, c0, 1\n\t"
        : "=r" (actlr)
        );

    return actlr;
}

/**
 * @brief Write Auxiliary Control Register
 *
 * Writes the contents of the ACTLR register which controls:
 *    branch prediction
 *    performance features
 *    error and parity logic
 *
 * @param[in] actlr value to be written to ACTLR register
 *
 * @return none
 */
static inline void
wr_actlr(uint32_t actlr)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c1, c0, 1\n\t"
        : : "r" (actlr)
        );
}

/**
 * @brief Read Secondary Auxiliary Control Register
 *
 * Reads the contents of the secondary ACTLR register which controls:
 *    branch prediction
 *    performance features
 *    error and parity logic
 *
 * @return contents of the secondary ACLTR
 */
static inline uint32_t
rd_secondary_actlr(void)
{
    uint32_t    val;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc p15, 0, %0, c15, c0, 1\n\t"
        : "=r" (val)
        );

    return val;
}

/**
 * @brief Write Secondary Auxiliary Control Register
 *
 * Writes the contents of the secondary ACTLR register which controls:
 *    branch prediction
 *    performance features
 *    error and parity logic
 *
 * @param[in] actlr value to be written to the secondary ACTLR register
 *
 * @return none
 */
static inline void
wr_secondary_actlr(uint32_t actlr)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c15, c0, 1\n\t"
        : : "r" (actlr)
        );
}

/**
 * @brief Read Cache Size Selection Register
 *
 * Reads the contents of the CSSELR register which is used to select
 * which level of the cache is to be interrogated.
 *
 * @return contents of the CSSELR
 */
static inline uint32_t
rd_csselr(void)
{
    uint32_t    csselr;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc    p15, 2, %0, c0, c0, 0\n\t"
        : "=r" (csselr)
        );

    return csselr;
}

/**
 * @brief Write Cache Size Selection Register
 *
 * Writes the contents of the CSSELR register to allow for the different
 * caches and cache levels to be interrogated.
 *
 * @param[in] csselr Level of cache to be interrogated
 *
 * @return none
 */
static inline void
wr_csselr(uint32_t csselr)
{
    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mcr    p15, 2, %0, c0, c0, 0\n\t"
        : : "r" (csselr)
        );
}

/**
 * @brief Read Cache Size ID Register
 *
 * Reads the contents of the CCSIDR register.  Provides information about the
 * size and behvior of the instruction or data cache.
 *
 * @return contents of the CCSIDR register
 */
static inline uint32_t
rd_ccsidr(void)
{
    uint32_t    ccsidr;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__(
        "mrc    p15, 1, %0, c0, c0, 0\n\t"
        : "=r" (ccsidr)
        );

    return ccsidr;
}

#define R5_CPSR_IF_MASK        (R5_CPSR_IRQ_MASK | R5_CPSR_FIQ_MASK)

/**
 * disable_interrupts() Disable interrupts without using OS
 *
 * Code coverage cannot use OS services because OS services
 * are being instrumented.
 */
static inline uint32_t
disable_interrupts(void)
{
    uint32_t    cur_cpsr;

    cur_cpsr = rd_cpsr();

    wr_cpsr(cur_cpsr | R5_CPSR_IF_MASK);

    return cur_cpsr;
}

/**
 * restore_interrupts() Restore the interrupts to original mask
 * @saved_mask          value saved from previous disable_interrupts() call
 *
 * Restore interrupts is used instead of enable interrupts disable/restore
 * may be called from within a OS provided critical section
 * (with interrupts disabled)
 */
static inline void
restore_interrupts(uint32_t saved_mask)
{
    uint32_t    new_cpsr;

    new_cpsr = rd_cpsr() & ~R5_CPSR_IF_MASK;
    new_cpsr |= saved_mask & R5_CPSR_IF_MASK;

    wr_cpsr(new_cpsr);
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
