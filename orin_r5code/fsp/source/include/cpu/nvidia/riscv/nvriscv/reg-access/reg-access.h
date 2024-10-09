/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2014-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef REG_ACCESS__REG_ACCESS_H
#define REG_ACCESS__REG_ACCESS_H
#define FSP__REG_ACCESS__REG_ACCESS_H                   1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <cpu/mmio-access.h>      // for ioread32, iowrite32, ...
#include <cpu/type-conversion.h>
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
CT_ASSERT(FSP__CPU__MMIO_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file reg-access.h
 * @brief functions that provide access to read and write the memory
 * mapped hardware registers.
 */

/**
 * @brief Read a 32-bit value from a memory mapped register
 *
 * @jama_func_req_id 8314409
 *
 * This function will read a 32-bit value from the memory mapped register
 * specified by the address parameter passed.
 *
 * @param[in] addr Address of the memory mapped register
 *
 * @return contents of the register
 */
static inline uint32_t readl(uint32_t addr)
{
    return ioread32(addr);
}

/**
 * @brief Read a 32-bit value from a memory mapped register address
 *
 * @jama_func_req_id 8314409
 *
 * This function will read a 32-bit value from the memory mapped register
 * specified by the address computed by the sum of the passed parameters
 * base and offset.
 *
 * @param[in] base Base address of the memory mapped register
 * @param[in] offset offset of the memory mapped register
 *
 * @return contents of the register
 */
static inline uint32_t readl_base_offset(uint32_t base, uint32_t offset)
{
    return ioread32_offset(base, offset);
}

/**
 * @brief Write a 32-bit value to a memory mapped register
 *
 * @jama_func_req_id 8314412
 *
 * This function will write a 32-bit value to the memory mapped register
 * specified by the address parameter passed.
 *
 * @param[in] data value to be written to the memory mapped register
 * @param[in] addr Address of the memory mapped register
 *
 * @return none
 */
static inline void writel(uint32_t data, uint32_t addr)
{
    iowrite32(addr, data);
}

/**
 * @brief Write a 32-bit value to a memory mapped register address
 *
 * @jama_func_req_id 8314412
 *
 * This function will write a 32-bit value to the memory mapped register
 * address computed by the sum of the passed parameters base and offset.
 *
 * @param[in] data value to be written to the memory mapped register
 * @param[in] base Base address of the memory mapped register
 * @param[in] offset offset of the memory mapped register; offset is immediate value
 *
 * @return none
 */
static inline void writel_base_offset(uint32_t data, uint32_t base, uint32_t offset)
{
    iowrite32_offset(base, offset, data);
}

/**
 * @brief Write a 32-bit value to a memory mapped register address
 *
 * @jama_func_req_id 8314412
 *
 * This function will write a 32-bit value to the memory mapped register
 * address computed by the sum of the passed parameters base and offset.
 *
 * @param[in] data value to be written to the memory mapped register
 * @param[in] base Base address of the memory mapped register
 * @param[in] offset offset of the memory mapped register; offset is register value
 *
 * @return none
 */
static inline void writel_base_regoffset(uint32_t data, uint32_t base, uint32_t offset)
{
    iowrite32_regoffset(base, offset, data);
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
