/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION. All rights reserved.
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
#ifndef WATCHDOG__SECTIONS_WATCHDOG_H
#define WATCHDOG__SECTIONS_WATCHDOG_H

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>               // for SECTION
#include <misc/macros.h>                   // for STR

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/*
 * Define the section name string.  The ARM and GNU tools optimize sections
 *  differently based on the section naming conventions.  If the correct section
 *  naming convention isn't used, the linker will fail to remove unused sections.
 *
 * When compiling with ARM tools, section name should be: NAME.TYPE
 * When compiling with GNU tools, section name should be: TYPE.NAME
 */
/*                                              SECTION_BUILDER(NAME                    TYPE)   */
#define SECTION_NAME_WATCHDOG_DATA              SECTION_BUILDER(watchdog,               data)
#define SECTION_NAME_WATCHDOG_TEXT              SECTION_BUILDER(watchdog,               text)
#define SECTION_NAME_WATCHDOG_ERROR_DATA        SECTION_BUILDER(error.watchdog,         data)
#define SECTION_NAME_WATCHDOG_ERROR_TEXT        SECTION_BUILDER(error.watchdog,         text)
#define SECTION_NAME_WATCHDOG_INIT_DATA         SECTION_BUILDER(init.watchdog,          data)
#define SECTION_NAME_WATCHDOG_INIT_RODATA       SECTION_BUILDER(init.watchdog,          rodata)
#define SECTION_NAME_WATCHDOG_INIT_TEXT         SECTION_BUILDER(init.watchdog,          text)
#define SECTION_NAME_WATCHDOG_PRIV_STACKS       SECTION_BUILDER(stacks.priv.watchdog,   bss)

#define SECTION_STR_WATCHDOG_TEXT               STR(SECTION_NAME_WATCHDOG_TEXT)
#define SECTION_STR_WATCHDOG_DATA               STR(SECTION_NAME_WATCHDOG_DATA)
#define SECTION_STR_WATCHDOG_INIT_TEXT          STR(SECTION_NAME_WATCHDOG_INIT_TEXT)
#define SECTION_STR_WATCHDOG_INIT_DATA          STR(SECTION_NAME_WATCHDOG_INIT_DATA)
#define SECTION_STR_WATCHDOG_INIT_RODATA        STR(SECTION_NAME_WATCHDOG_INIT_RODATA)
#define SECTION_STR_WATCHDOG_ERROR_TEXT         STR(SECTION_NAME_WATCHDOG_ERROR_TEXT)
#define SECTION_STR_WATCHDOG_ERROR_DATA         STR(SECTION_NAME_WATCHDOG_ERROR_DATA)
#define SECTION_STR_WATCHDOG_PRIV_STACKS        STR(SECTION_NAME_WATCHDOG_PRIV_STACKS)

#define SECTION_WATCHDOG_TEXT                   SECTION(SECTION_STR_WATCHDOG_TEXT)
#define SECTION_WATCHDOG_DATA                   SECTION(SECTION_STR_WATCHDOG_DATA)
#define SECTION_WATCHDOG_INIT_TEXT              SECTION(SECTION_STR_WATCHDOG_INIT_TEXT)
#define SECTION_WATCHDOG_INIT_DATA              SECTION(SECTION_STR_WATCHDOG_INIT_DATA)
#define SECTION_WATCHDOG_INIT_RODATA            SECTION(SECTION_STR_WATCHDOG_INIT_RODATA)
#define SECTION_WATCHDOG_ERROR_TEXT             SECTION(SECTION_STR_WATCHDOG_ERROR_TEXT)
#define SECTION_WATCHDOG_ERROR_DATA             SECTION(SECTION_STR_WATCHDOG_ERROR_DATA)
#define SECTION_WATCHDOG_PRIV_STACKS            SECTION(SECTION_STR_WATCHDOG_PRIV_STACKS)
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")


#endif