/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef CPU__ARMV7_MPU_SAFERTOS_H
#define CPU__ARMV7_MPU_SAFERTOS_H
#define FSP__CPU__ARMV7_MPU_SAFERTOS_H           1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Hardware headers */

/* Late FSP headers */
#include <misc/nvrm_drf.h>

/* Module-specific FSP headers */
#include <armv7-mpu.h>                                  /* Immune from CT_ASSERT protection */
#include <cpu/armv7-mpu.h>

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @file armv7-mpu-safertos.h
 * @brief Definitions and interfaces to use the MPU under SafeRTOS
 *
 * This file provides definitions, data structures and interfaces that
 * are necessary for an application to use the MPU driver.
 */

/**
 * @brief Type for representing an MPU Region number
 */
typedef uint32_t        r5mpu_region_num_t;

/**
 * @brief Type for representing MPU Check Access for a region
 *
 * The values R5MPU_CHECK_* values (below) can be logical OR'd together
 * to create the desired access for a region.
 */
typedef uint8_t         r5mpu_check_access_t;

/**
 * @brief No Privileged Access for a region.
 */
#define R5MPU_CHECK_PRIV_NONE   ((r5mpu_check_access_t)0x00U)

/**
 * Privileged Read Access for a region.
 */
#define R5MPU_CHECK_PRIV_READ   ((r5mpu_check_access_t)0x01U)

/**
 * @brief Privileged Write Access for a region.
 */
#define R5MPU_CHECK_PRIV_WRITE  ((r5mpu_check_access_t)0x02U)

/**
 * @brief No Unprivileged Access for a region.
 */
#define R5MPU_CHECK_USER_NONE   ((r5mpu_check_access_t)0x00U)

/**
 * @brief Unprivileged Read Access for a region.
 */
#define R5MPU_CHECK_USER_READ   ((r5mpu_check_access_t)0x04U)

/**
 * @brief Unprivileged Write Access for a region.
 */
#define R5MPU_CHECK_USER_WRITE  ((r5mpu_check_access_t)0x08U)

/**
 * @brief Privileged Read and Write Access for a region.
 */
#define R5MPU_CHECK_PRIV_RW     (R5MPU_CHECK_PRIV_READ | R5MPU_CHECK_PRIV_WRITE)

/**
 * @brief Unprivileged Read and Write Access for a region.
 */
#define R5MPU_CHECK_USER_RW     (R5MPU_CHECK_USER_READ | R5MPU_CHECK_USER_WRITE)

/**
 * @brief Type representing the flags in mpuParameters structure
 */
typedef uint8_t         r5mpu_flags_t;

/**
 * @brief Region is valid
 */
#define R5MPU_FL_VALID          ((r5mpu_flags_t)0x01U)

/**
 * @brief The size field in mpuParameters structure contains a size.
 */
#define R5MPU_FL_SIZE           ((r5mpu_flags_t)0x02U)

/**
 * The size field in mpuParameters structure contains an ending address.
 * During r5mpu_init() and r5mpu_task_region_init(), these fields will
 * be converted to a size by subtracting the base field from the size
 * field.
 */
#define R5MPU_FL_END            ((r5mpu_flags_t)0x04U)

/**
 * Indicate that the structure has been initialized.
 */
#define R5MPU_FL_INIT           ((r5mpu_flags_t)0x80U)

/**
 * @brief MPU parameters structure
 *
 * This structure contains the information that is necessary for
 * initializing the MPU and managing per-task MPU regions.  This
 * structure is generally used in arrays of either MPU_NGLOBAL_REGIONS
 * (for mpu_global_regions) or MPU_NTASK_REGIONS for the per-task
 * MPU regions.
 * @flags         The flags that indicate the state of the structure.
 * @permissions   The permissions that will be used for validating if a
 *                range is allowed to access a region.
 * @index         The MPU region number that the region corresponds to.
 * @size          The size of the region if R5MPU_FL_SIZE is set in flags.
 *                The end address of the region if R5MPU_FL_END is set in flags.
 * @base          The base address of the region.
 * @access        The access and memory type of the region.  See armv7-mpu.h
 *                for the acceptable values in this field.
 * @encoded_size  The encoded size of the MPU region.  It is of a form that
 *                is directly usable by the MPU hardware.  This field will
 *                be filled in/overwritten by calls to r5mpu_init() or
 *                r5mpu_task_region_init().
 */
typedef struct mpuParameters {
    r5mpu_flags_t       flags;          // flags on how to interprete fields
    r5mpu_check_access_t permissions;   // access for checking permissions
    uint8_t             pad[2];         // padding to align next field
    r5mpu_region_num_t  index;          // region index
    uint32_t            size;           // size (in bytes) of region
    // the following fields must be kept in this order to allow for
    // fast programming of the MPU during a context switch.  They
    // must also be at the end of the structure.
    uint32_t            base;           // base address for region
    uint32_t            access;         // access permissions
    uint32_t            encoded_size;   // encoded size - direct set of MPU
} mpuParameters_t;

/**
 * @brief Global MPU regions
 *
 * The global MPU regions that are common across all tasks.  It must
 * be initialized (usually staticlly) prior to calling r5mpu_init().
 */
extern mpuParameters_t  mpu_global_regions[MPU_NGLOBAL_REGIONS];

/**
 * @brief Initialize a Task's MPU region
 *
 * This function will initialize the supplied task's MPU region so that
 * it is useable by the context switch and copy check code.
 *
 * @pre r5mpu_init must have already been called to set up the global
 *      regions.
 *
 * @param[in]  task_regions   A pointer to an array of mpuParameters structures
 *                            that contains MPU_NTASK_REGIONS elements, that
 *                            represent the MPU regions for a specific task
 *
 * @return None
 */
void r5mpu_task_region_init(mpuParameters_t *task_mpu_regions);

/**
 * @brief Initialize Global MPU state
 *
 * This function will initialize the MPU for use.  It will perform the
 * following operations:
 * - finish initializing the global MPU regions contained in mpu_global_regions.
 * - initialize all of the MPU regions in the hardware according to the
 *   contents of mpu_global_regions and disabling any per-task regions.
 * - enabling the MPU hardware
 *
 * @pre mpu_global_regions must have at least one region defined that is
 *      not disabled.
 *
 * @return None
 */
void r5mpu_init(void);

/**
 * @brief Initialize an MPU region structure
 *
 * This function assumes that most of the structure has already been
 * initialized (more than likely statically).  This fills in any remaining
 * fields in the data structure from the information already present.
 *
 * @pre If set_mpu is true, then the R5's MPU must be disabled.  If set_mpu
 *      is false, then there is no requirement on the state of the R5's MPU.
 *
 * @param[in]  region    pointer to an mpuParameters structure
 * @param[in]  set_mpu   boolean that indicates if the actual MPU region
 *                       in the R5 should be set or if this is only used
 *                       to initialize the mpuParameters structure.
 *
 * @return None
 */
void r5mpu_region_init(mpuParameters_t *const region,
                       bool set_mpu);

/**
 * @brief Return the number of MPU regions
 *
 * Returns the number of MPU regions that are supported by the R5 CPU.
 *
 * @return number of MPU regions supported by the R5 CPU.
 */
static inline uint32_t
r5mpu_region_count(void)
{
    uint32_t        mpuir;

    INLINE_RFD(MISRA, FP, Directive_4_3, "Approval: JIRA TID-1940, DR: SWE-FSP-065-SWSADR.docx");
    __asm__ __volatile__("mrc p15, 0, %0, c0, c0, 4\n\t" : "=r" (mpuir));

    return NV_DRF_VAL(R5, MPUIR, DREGION, mpuir);
}

/**
 * @brief R5 MPU not initialized hook
 *
 * This is a default hook that can be overridden by the application
 * to handle cases when one of the MPU functions is called when the
 * MPU has not been initialized.
 *
 * @return None
 */
void r5mpu_not_initialized_hook(void);

/**
 * @brief R5 MPU re-initialization attempted
 *
 * This is a default hook that can be overridden by the application
 * to handle cases when re-initialization of the MPU is attempted.
 *
 * @param[in]  region  pointer to region that was already initialized.
 *                     NULL indicates that r5mpu_init was called again.
 *
 * @return None
 */
void r5mpu_reinitialize_hook(const mpuParameters_t * const region);

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif
