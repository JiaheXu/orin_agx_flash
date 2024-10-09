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

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>                     // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                        // for U32_C, FSP_MISRA_BLOC...
#include <misc/generate-asm-hdr.h>              // for ASM_OFFSET macros...

/* Module-specific FSP headers */
#include <cpu/barriers.h>                       // for barrier_compiler, bar...
#include <cpu/sections-armv7-mpu.h>             // Immune from CT_ASSERT protection
#include <ospl/rtos-port.h>                     // for struct mpuParameters...

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
CT_ASSERT(FSP__MISC__GENERATE_ASM_HDR_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__BARRIERS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")

/**
 * @file armv7-mpu-asm-hdr.c
 * @brief Generates the MPU asm/C header defines.
 */

SECTION_MPU_INIT_TEXT
void mpuParameters_asm_hdr(void);

void mpuParameters_asm_hdr(void)
{
    ASM_OFFSET(MPU_INDEX_OFFSET, struct mpuParameters, index);
    ASM_OFFSET(MPU_BASE_OFFSET, struct mpuParameters, base);
    ASM_SIZEOF(MPU_PARM_LENGTH, struct mpuParameters);
    barrier_compiler();
}

