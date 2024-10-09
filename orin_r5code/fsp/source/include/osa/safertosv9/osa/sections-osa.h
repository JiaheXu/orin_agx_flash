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
#ifndef OSA__SECTIONS_OSA_H
#define OSA__SECTIONS_OSA_H
#define FSP__OSA__SECTIONS_OSA_H                        0

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
#define SECTION_NAME_OSA_DATA           SECTION_BUILDER(osa,                    data)
#define SECTION_NAME_OSA_RODATA         SECTION_BUILDER(osa,                    rodata)
#define SECTION_NAME_OSA_TEXT           SECTION_BUILDER(osa,                    text)
#define SECTION_NAME_OSA_CREATE_TEXT    SECTION_BUILDER(create.osa,             text)
#define SECTION_NAME_OSA_INIT_DATA      SECTION_BUILDER(init.osa,               data)
#define SECTION_NAME_OSA_INIT_TEXT      SECTION_BUILDER(init.osa,               text)

#define SECTION_STR_OSA_TEXT            STR(SECTION_NAME_OSA_TEXT)
#define SECTION_STR_OSA_RODATA          STR(SECTION_NAME_OSA_RODATA)
#define SECTION_STR_OSA_DATA            STR(SECTION_NAME_OSA_DATA)
#define SECTION_STR_OSA_INIT_TEXT       STR(SECTION_NAME_OSA_INIT_TEXT)
#define SECTION_STR_OSA_INIT_DATA       STR(SECTION_NAME_OSA_INIT_DATA)
#define SECTION_STR_OSA_CREATE_TEXT     STR(SECTION_NAME_OSA_CREATE_TEXT)

#define SECTION_OSA_TEXT                SECTION(SECTION_STR_OSA_TEXT)
#define SECTION_OSA_RODATA              SECTION(SECTION_STR_OSA_RODATA)
#define SECTION_OSA_DATA                SECTION(SECTION_STR_OSA_DATA)
#define SECTION_OSA_INIT_TEXT           SECTION(SECTION_STR_OSA_INIT_TEXT)
#define SECTION_OSA_INIT_DATA           SECTION(SECTION_STR_OSA_INIT_DATA)
#define SECTION_OSA_CREATE_TEXT         SECTION(SECTION_STR_OSA_CREATE_TEXT)

#endif

