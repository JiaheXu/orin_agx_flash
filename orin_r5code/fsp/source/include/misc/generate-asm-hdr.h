/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * This file provides the interfaces for the SafeRTOS MPU functions for
 * ARMv7 CPUs (notably the Cortex R5)
 */
#ifndef MISC__GENERATE_ASM_HDR_H
#define MISC__GENERATE_ASM_HDR_H
#define FSP__MISC__GENERATE_ASM_HDR_H   1

/* Compiler headers */
#include <stddef.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

/**
 * @file generate-asm-hdr.h
 * @brief Generate C/asm data structure member offset defines.
 *
 * This file provides macros for generating data structure member
 * offset defines that are used in both C source and assembly for
 * the purpose of maintaining the offsets of the members.
 */

#define ASM_DEFINE(_s_, _v_)                                                  \
    __asm__ __volatile__ ("##define " #_s_ " %0 \n" : : "i" (_v_))

#define ASM_OFFSET(_sym_, _struct_, _member_)                                 \
    ASM_DEFINE(_sym_, offsetof(_struct_, _member_))

#define ASM_SIZEOF(_sym_, _struct_)                                           \
    ASM_DEFINE(_sym_, sizeof(_struct_))

#endif /* MISC__GENERATE_ASM_HDR_H */
