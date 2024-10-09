/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2020-2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef CPU__FENCE_H
#define CPU__FENCE_H
#define FSP__CPU__FENCE_H                       1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <cpu/csr.h>

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include NVRISCV64_MANUAL_CSR

// Lightweight fence, may not be coherent with rest of the system

/**
 * @brief Lightweight io fence wrapper
 *
 * @jama_func_req_id <ID>
 *
 * @return none
 */
static inline void riscvLwfenceIO(void)
{
    __asm__ volatile ("csrrw zero, %0, zero" : : "i"(NV_RISCV_CSR_LWFENCEIO));
}

/**
 * @brief Lightweight memory fence wrapper
 *
 * @jama_func_req_id <ID>
 *
 * @return none
 */
static inline void riscvLwfenceRW(void)
{
    __asm__ volatile ("csrrw zero, %0, zero" : : "i"(NV_RISCV_CSR_LWFENCEMEM));
}

/**
 * @brief Lightweight fence wrapper for io and memory
 *
 * @jama_func_req_id <ID>
 *
 * @return none
 */
static inline void riscvLwfenceRWIO(void)
{
    __asm__ volatile ("csrrw zero, %0, zero" : : "i"(NV_RISCV_CSR_LWFENCEALL));
}

// Heavy-weight fence, mem fence may not work on dGPU with FB locked

/**
 * @brief Heavy-weight IO fence wrapper
 *
 * @jama_func_req_id <ID>
 *
 * @return none
 */
static inline void riscvFenceIO(void)
{
    __asm__ volatile ("fence io,io");
}

/**
 * @brief Heavy-weight memory fence wrapper
 *
 * @jama_func_req_id <ID>
 *
 * @return none
 */
static inline void riscvFenceRW(void)
{
    __asm__ volatile ("fence rw,rw");
}

/**
 * @brief Heavy-weight fence wrapper for io and memory
 *
 * @jama_func_req_id <ID>
 *
 * @return none
 */
static inline void riscvFenceRWIO(void)
{
    __asm__ volatile ("fence iorw,iorw");
}

/**
 * @brief Heavy-weight supervisor fence wrapper
 *
 * @jama_func_req_id <ID>
 *
 * @param[in] asid:
 * @param[in] vaddr:
 *
 * @return none
 */
static inline void riscvSfenceVMA(uint8_t asid, uint8_t vaddr)
{
    __asm__ volatile ("sfence.vma %0, %1" :: "r" (asid), "r" (vaddr));
}

#endif /* FSP__CPU__FENCE_H */
