/*
 * cr52-diag - Diagnostics module for debugging FSP on NVRISCV.
 *
 */
#include <stdbool.h>

#include <stdint.h>
#include <stddef.h>
#include <misc/ct-assert.h>
#include <cpu/mmio-access.h>
#include <misc/macros.h>

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__MMIO_ACCESS_H, "Header file missing or invalid.")

#if 0
#pragma message "content of INT_MAX: " STR(INT_MAX)
#pragma message "content of INT32_MAX: " STR(INT32_MAX)
#pragma message "content of UINT_MAX: " STR(UINT_MAX)
#pragma message "content of UINT32_MAX: " STR(UINT32_MAX)
#pragma message "content of LONG_MAX: " STR(LONG_MAX)
#pragma message "content of size_t: " STR(__SIZE_TYPE__)
#pragma message "content of sizeof(uintptr_t): " STR(sizeof(uintptr_t))
#pragma message "content of sizeof(uint32_t): " STR(sizeof(uint32_t))
#pragma message "content of sizeof(unsigned long): " STR(sizeof(unsigned long))
#pragma message "content of __LONG_MAX__: " STR(__LONG_MAX__)
#pragma message "content of UINTPTR_MAX: " STR(UINTPTR_MAX)
#endif

#define BASE_ADDR32	0x12345678U
#define BASE_ADDR64	0x1234567890UL
#define OFFSET_32	0x75050505U

/* Prototypes */
void cr52_diag(void);
uint32_t r32(uintptr_t addr);
void     w32(uintptr_t addr, uint32_t data);
//uint32_t ra32(uint32_t addr);
//void     wa32(uint32_t data, uint32_t addr);
uint32_t r32o(uintptr_t addr, uint32_t offset);
void     w32o(uintptr_t addr, uint32_t offset, uint32_t data);
//uint32_t ra32o(uint32_t addr, uint32_t offset);
//void     wa32o(uint32_t data, uint32_t addr, uint32_t offset);

/* CR52-DIAG implementation */
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void cr52_diag(void)
{
    /* Verify that 32-bit MMIO functions are MISRA clean */
    uintptr_t addr32 = BASE_ADDR32;
    uint32_t  offs32 = OFFSET_32;
    uint32_t  data32;
    data32 = r32(addr32);
    w32(addr32, data32);
//    data32 = ra32(addr32);
//    wa32(data32, addr32);
    data32 = r32o(addr32, offs32);
    w32o(addr32, offs32, data32);
//    data32 = ra32o(addr32, offs32);
//    wa32o(data32, addr32, offs32);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint32_t r32(uintptr_t addr)
{
    return ioread32(addr);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void w32(uintptr_t addr, uint32_t data)
{
    iowrite32(addr, data);
}

#if 0
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint32_t ra32(uint32_t addr)
{
    return readl(addr);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void wa32(uint32_t data, uint32_t addr)
{
    writel(addr, data);
}
#endif

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint32_t r32o(uintptr_t addr, uint32_t offset)
{
    return ioread32_offset(addr, offset);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void w32o(uintptr_t addr, uint32_t offset, uint32_t data)
{
    iowrite32_offset(addr, offset, data);
}

#if 0
INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
uint32_t ra32o(uint32_t addr, uint32_t offset)
{
    return readl_base_offset(addr, offset);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "")
void wa32o(uint32_t data, uint32_t addr, uint32_t offset)
{
    writel_base_offset(data, addr, offset);
}
#endif

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
