/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef VIC__VIC_LD_H
#define VIC__VIC_LD_H
#define FSP__VIC__VIC_LD_H                              1

/* Compiler headers */
#ifndef __ASSEMBLER__
#include <stdint.h>               // for uint32_t
#endif

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/linker-symbols.h>  // for FSP__MISC__LINKER_SYMBOLS_H, IMAGE_...
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <processor/irqs-hw.h>    // for FSP__PROCESSOR__IRQS_HW_H, MAX_VIC_...

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
CT_ASSERT(FSP__MISC__LINKER_SYMBOLS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#ifndef __ASSEMBLER__

typedef uint32_t        irq_vect_t;

/**
 * @brief VIC layer ISR map structure.
 *
 * Structure used to map ISR to the tegra interrupt vector.
 *
 * @vector      Tegra interrupt vector [0 - MAX_VIC_INTERRUPT_VECTOR]
 * @isr_handler Pointer to the ISR for mapped tegra interrupt vector.
 *              Must not be NULL
 */
struct vic_isr_map_s {
    irq_vect_t          vector;
    void                (*isr_handler)(void);
};

extern struct vic_isr_map_s    IMAGE_SYM_BASE(VIC_ISR_MAP);
extern struct vic_isr_map_s    IMAGE_SYM_LIMIT(VIC_ISR_MAP);
const extern uint32_t vic_base_addr[MAX_VIC_CONTROLLER];

#endif // __ASSEMBLER__
#endif // VIC__VIC_LD_H
