/**
 * @file perfctr.h
 *
 * @brief Wrappers for riscv performance counter CSRs.
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
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


#ifndef NVRISCV_PERFCTR_H
#define NVRISCV_PERFCTR_H
#include <stdint.h>
#include <cpu/csr.h>

/**
 * @brief Atomically reads timer csr.
 * @return Contents of time CSR.
 *
 * This register contains value of PTIMER (also available via PTIMER0/PTIMER1
 * localIO registers).
 *
 * \warning This function requires permissions to execute.
 * Invalid instruction exception will be thrown if they're not granted.
 * - For M mode - it can always execute
 * - For S mode, mcounteren.tm must be set
 * - For U mode, mcounteren.tm and scounteren.tm must be set
 */
#define rdtime() csr_read(NV_RISCV_CSR_HPMCOUNTER_TIME)

/**
 * @brief Atomically reads cycle counter csr.
 * @return Number of cpu cycles from arbitrary moment in time (RISC-V start)
 *
 * \warning This function requires permissions to execute.
 * Invalid instruction exception will be thrown if they're not granted.
 * - For M mode - it can always execute
 * - For S mode, mcounteren.cy must be set
 * - For U mode, mcounteren.cy and scounteren.cy must be set
 */
#define rdcycle() csr_read(NV_RISCV_CSR_HPMCOUNTER_CYCLE)

/**
 * @brief Atomically reads instruction retired counter csr.
 * @return Number of instructions retired from arbitrary moment in time (RISC-V start)
 *
 * \warning This function requires permissions to execute.
 * Invalid instruction exception will be thrown if they're not granted.
 * - For M mode - it can always execute
 * - For S mode, mcounteren.ir must be set
 * - For U mode, mcounteren.ir and scounteren.ir must be set
 */
#define rdinstret() csr_read(NV_RISCV_CSR_HPMCOUNTER_INSTRET)

#endif // NVRISCV_PERFCTR_H
