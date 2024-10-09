/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */

#ifndef CPU__BARRIER_H
#define CPU__BARRIER_H
#define FSP__CPU__BARRIER_H                       1

/* Compiler headers */

/* Early FSP headers */

/* Late FSP headers */
#include <cpu/fence.h>

/* NVRISCV conifgured headers */

/**
 * @file barriers.h
 * @brief Definitions for various synchronization barriers
 */

/**
 * @brief A general description of the function
 *
 * @jama_func_req_id <ID>
 *
 * A more detailed description of the function
 *
 * @return none
 */
static inline void barrier_cache_op_complete(void)
{
    riscvLwfenceRW();
}

/**
 * @brief A general description of the function
 *
 * @jama_func_req_id <ID>
 *
 * A more detailed description of the function
 *
 * @return none
 */
static inline void barrier_compiler(void) {}

/**
 * @brief A general description of the function
 *
 * @jama_func_req_id <ID>
 *
 * A more detailed description of the function
 *
 * @return none
 */
static inline void barrier_memory_order(void)
{
    riscvLwfenceRW();
}

/**
 * @brief A general description of the function
 *
 * @jama_func_req_id <ID>
 *
 * A more detailed description of the function
 *
 * @return none
 */
static inline void barrier_memory_complete(void)
{
    riscvFenceRW();
}

/**
 * @brief A general description of the function
 *
 * @jama_func_req_id <ID>
 *
 * A more detailed description of the function
 *
 * @return none
 */
static inline void barrier_instruction_synchronization(void)
{
    __asm__ volatile("fence.i");
}

#endif // CPU__BARRIER_H
