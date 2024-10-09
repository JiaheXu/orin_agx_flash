/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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

#ifndef ADCC__SECTIONS_ADCC_H
#define ADCC__SECTIONS_ADCC_H

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
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")

/*
 * Define the section name string.  The ARM and GNU tools optimize sections
 *  differently based on the section naming conventions.  If the correct section
 *  naming convention isn't used, the linker will fail to remove unused sections.
 *
 * When compiling with ARM tools, section name should be: NAME.TYPE
 * When compiling with GNU tools, section name should be: TYPE.NAME
 */
/*                                      SECTION_BUILDER(NAME                    TYPE)   */
#define SECTION_NAME_ADCC_DATA           SECTION_BUILDER(adcc,                    data)
#define SECTION_NAME_ADCC_RODATA         SECTION_BUILDER(adcc,                    rodata)
#define SECTION_NAME_ADCC_TEXT           SECTION_BUILDER(adcc,                    text)
#define SECTION_NAME_ADCC_INIT_DATA      SECTION_BUILDER(init.adcc,               data)
#define SECTION_NAME_ADCC_INIT_TEXT      SECTION_BUILDER(init.adcc,               text)

#define SECTION_STR_ADCC_TEXT            STR(SECTION_NAME_ADCC_TEXT)
#define SECTION_STR_ADCC_RODATA          STR(SECTION_NAME_ADCC_RODATA)
#define SECTION_STR_ADCC_DATA            STR(SECTION_NAME_ADCC_DATA)
#define SECTION_STR_ADCC_INIT_TEXT       STR(SECTION_NAME_ADCC_INIT_TEXT)
#define SECTION_STR_ADCC_INIT_DATA       STR(SECTION_NAME_ADCC_INIT_DATA)

#define SECTION_ADCC_TEXT                SECTION(SECTION_STR_ADCC_TEXT)
#define SECTION_ADCC_RODATA              SECTION(SECTION_STR_ADCC_RODATA)
#define SECTION_ADCC_DATA                SECTION(SECTION_STR_ADCC_DATA)
#define SECTION_ADCC_INIT_TEXT           SECTION(SECTION_STR_ADCC_INIT_TEXT)
#define SECTION_ADCC_INIT_DATA           SECTION(SECTION_STR_ADCC_INIT_DATA)

#endif
