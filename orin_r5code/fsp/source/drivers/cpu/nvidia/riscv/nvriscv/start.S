/**
 * @file start.S
 *
 * @brief Default assembly startup code
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

#include <cpu/csr.h>
#include <cpu/asm-helpers.h>

.section ".text.startup","ax",@progbits
.balign 4
FUNC startup
    j    do_reset

do_reset:
    ZREG(x1)
    ZREG(x2)
    ZREG(x3)
    ZREG(x4)
    ZREG(x5)
    ZREG(x6)
    ZREG(x7)
    ZREG(x8)
    ZREG(x9)
    ZREG(x10)
    ZREG(x11)
    ZREG(x12)
    ZREG(x13)
    ZREG(x14)
    ZREG(x15)
    ZREG(x16)
    ZREG(x17)
    ZREG(x18)
    ZREG(x19)
    ZREG(x20)
    ZREG(x21)
    ZREG(x22)
    ZREG(x23)
    ZREG(x24)
    ZREG(x25)
    ZREG(x26)
    ZREG(x27)
    ZREG(x28)
    ZREG(x29)
    ZREG(x30)
    ZREG(x31)

    // write mtvec and make sure it sticks
#if NVRISCV_FEATURE_EXCEPTION == 1
    la    t0, exception_entry
#else
    la    t0, riscv_panic_trampoline
#endif

#if (NVRISCV_CONFIG_CPU_MODE==3)
    csrw  mtvec, t0
#endif
#if (NVRISCV_HAS_S_MODE == 1)
    csrw  stvec, t0
#endif

    // write mrsp on specific chips due to HW bug
    // See https://nvbugs/3342254
#if ((NVRISCV_IS_CHIP_ls10 == 1) || (NVRISCV_IS_CHIP_gh100==1)) && (NVRISCV_CONFIG_CPU_MODE==3)
    csrw NV_RISCV_CSR_MRSP, zero
#endif

    la    sp, _stack_bottom
    LREG  sp, 0(sp)
    addi  sp, sp, -0x10
    SREG  zero, 0(sp)
    SREG  zero, 8(sp)
    jal   main
    // halt the core one way or another
    jal   riscv_shutdown
_end:
    j _end
EFUNC startup

#if NVRISCV_FEATURE_EXCEPTION == 0
.balign 4
FUNC riscv_panic_trampoline
    jal riscv_panic
EFUNC riscv_panic_trampoline
#endif

// We need that dereference to handle chips where dmem is far away from imem
  .balign 8
_stack_bottom:
  .dword _libnvriscv_stack_bottom
