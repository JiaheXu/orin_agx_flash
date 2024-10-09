/**
 * @file asm_helpers.h
 *
 * @brief Portable NVRISC-V assembly macros
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS`` AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPU__ASM_HELPERS_H
#define CPU__ASM_HELPERS_H
#define FSP__CPU__ASM_HELPERS_H    1

/*
 * @file asm-helpers.h
 * Macros to improve the portability of assembly code accross NVRISC-V
 * architectures and improve the writing of assembly functions.
 */

/*
 * @brief Macros to improve portability of assembly code
 *
 * @macro-title Portable Assembly Code
 *
 * @REGBYTES        Describes how many bytes per register
 * @SREG            Instruction to store word from register
 * @LREG            Instruction to load word into register
 * @ZREG            Instruction to write zero into register
 */
#if (__riscv_xlen == 32U)
#define REGBYTES    4U
#define SREG        sw
#define LREG        lw
#else
#define REGBYTES    8U
#define SREG        sd
#define LREG        ld
#endif // (__riscv_xlen == 32)
#define ZREG(rd)    mv rd, zero

/**
 * @brief Assembly function setup
 *
 * Set up an assembly function with a specified name.
 *
 * @param[in] name      Name of the assembly function to set up
 */
.macro FUNC name
   .global \name
   \name :
   .func \name, \name
    .type \name, STT_FUNC
.endm

/**
 * @brief Assembly function teardown
 *
 * Tear down an assembly function with a specified name.
 *
 * @param[in] name      Name of the assembly function to tear down
 */
.macro EFUNC name
    .endfunc
    .size \name, .-\name
.endm

#endif /* CPU__ASM_HELPERS_H */
/** end of file **/
