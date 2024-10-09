/**
 * @file ptimer.h
 *
 * @brief Chip-specific ptimer (main systick) read wrapper.
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

#ifndef LIBNVRISCV_PTIMER_H
#define LIBNVRISCV_PTIMER_H
#include <stdint.h>
#include <cpu/perfctr.h>

#if NVRISCV_HAS_BINARY_PTIMER
#define PTIMER_SHIFT 0
#else // NVRISCV_HAS_BINARY_PTIMER
#define PTIMER_SHIFT 5
#endif // NVRISCV_HAS_BINARY_PTIMER

/*!
 * \brief Read 64-bit PTIMER
 * \return PTIMER value (1ns resolution)
 *
 * This function reads PTIMER, avoiding non-trivial reads from two 32-bit
 * localIO registers (PTIMER0/PTIMER1).
 *
 * \warning mcounteren.tm (for S- and U-mode) and scounteren.tm (for U-mode)
 * must be set or this function will cause invalid instruction exception.
 */
static inline uint64_t ptimer_read(void)
{
    return rdtime() << PTIMER_SHIFT;
}

#endif // LIBNVRISCV_PTIMER_H
