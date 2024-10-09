/*
 * Copyright (c) 2015-2022, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <armv7-mpu.h>                                  /* Immune from CT_ASSERT protection */
#include <cpu/sections-armv7-mpu.h>
#if defined(MPU_DYNAMIC_HEADERS) && (MPU_DYNAMIC_HEADERS == 1)
#include "armv7-mpu-asm-dynamic-hdr.h"
#else
#include <cpu/armv7-mpu-asm-static-hdr.h>
#endif

/*
 * ARMV7_MPU_RESTORE_REGION
 *
 * Macro that restores a single MPU region
 *
 * Inputs:
 *      R0:     pointer to mpuParameters_t structure
 *      R7:     0
 *
 * Outputs:
 *      R0:     pointer to next mpuParameters_t structure
 *
 * Registers affected:
 *      R1, R2, R3
 */
        .macro  ARMV7_MPU_RESTORE_REGION
             LDR        R1, [R0, MPU_INDEX_OFFSET] // get the region index
             MCR        p15, #0, R1, c6, c2, #0 // set region number
             MCR        p15, #0, R7, c6, c1, #2 // disable region
             ADD        R0, R0, MPU_BASE_OFFSET // point to values to be loaded
             LDMIA      R0!, {R1-R3}            // R0 will point to next struct
             MCR        p15, #0, R1, c6, c1, #0 // setup base address
             MCR        p15, #0, R2, c6, c1, #4 // setup access permissions
             MCR        p15, #0, R3, c6, c1, #2 // setup size
        .endm

/*
 * ARMV7_MPU_RESTORE_REGIONS
 *
 * Macro to restore a Task's MPU regions.  It is a wrapper that will
 * invoke ARMV7_MPU_RESTORE_REGION "n" times.
 *
 * Inputs:
 *      R0:     pointer to first element in an array of mpuParameters_t
 *              structures that represent the MPU regions to be restored
 *      R7:     0
 *
 * Outputs:
 *      R0:     pointer past the last mpuParameters_t structure of the MPU
 *              regions being restored.
 *
 * Registers affected:
 *      Registers affected by ARMV7_MPU_RESTORE_REGION
 */
        .macro  ARMV7_MPU_RESTORE_REGIONS n
        .if \n
        ARMV7_MPU_RESTORE_REGION
        ARMV7_MPU_RESTORE_REGIONS "(\n-1)"
        .endif
        .endm


	.section	SECTION_STR_MPU_TEXT, "ax"
	.arm
	.syntax		unified
/*
 * armv7_task_mpu_restore()	- restores a task's MPU context
 *
 * @task_mpu:	pointer to a task_mpu_regions_t structure that represents
 *		the task's MPU state that is to be restored.  It is expected
 *		that this pointer is in R0.
 *
 * This function will restore the task's MPU context so that the task will
 * be able to be dispatched and function correctly after a context switch.
 *
 * Return Values:
 *	none
 */
	.global		armv7_task_mpu_restore
	.type		armv7_task_mpu_restore, %function
	.balign		4
armv7_task_mpu_restore:
	PUSH		{ R0, R1, R2, R3, R7, LR }
	MOV		R7, #0
	ARMV7_MPU_RESTORE_REGIONS	MPU_NTASK_REGIONS
	POP		{ R0, R1, R2, R3, R7, PC }

/*
 * ARMV7_MPU_DISABLE_REGION
 *
 * Macro that disables a single MPU region
 *
 * Inputs:
 *      R0:     pointer to mpuParameters_t structure
 *      R7:     0
 *
 * Outputs:
 *      R0:     pointer to next mpuParameters_t structure
 *
 * Registers affected:
 *      R1
 */
        .macro  ARMV7_MPU_DISABLE_REGION	n
             LDR        R1, [R0, MPU_INDEX_OFFSET] // get the region index
             MCR        p15, #0, R1, c6, c2, #0 // set region number
             MCR        p15, #0, R7, c6, c1, #2 // disable region
	     ADD        R0, R0, MPU_PARM_LENGTH // move to next region
        .endm

/*
 * ARMV7_MPU_DISABLE_REGIONS
 *
 * Macro to disable a Task's MPU regions.  It is a wrapper that will
 * invoke ARMV7_MPU_DISABLE_REGION "n" times.
 *
 * Inputs:
 *      R0:     pointer to mpuParameters_t structure
 *      R7:     0
 *
 * Outputs:
 *	none
 *
 * Registers affected:
 *      Registers affected by ARMV7_MPU_DISABLE_REGION
 */
        .macro  ARMV7_MPU_DISABLE_REGIONS n
        .if \n
        ARMV7_MPU_DISABLE_REGION  \n
        ARMV7_MPU_DISABLE_REGIONS "(\n-1)"
        .endif
        .endm


	.section	SECTION_STR_MPU_TEXT, "ax"
	.arm
	.syntax		unified
/*
 * armv7_task_mpu_disable()	- disables a task's MPU context
 *
 * This function will disable a task's MPU context.
 *
 * Inputs:
 *	task_regions	A pointer to an array of mpuParameter structures
 *			that contains exactly MPU_NTASK_REGIONS.  It is
 *                      expected that this value is in R0.
 *
 * Return Values:
 *	none
 */
	.global		armv7_task_mpu_disable
	.type		armv7_task_mpu_disable, %function
	.balign		4
armv7_task_mpu_disable:
	PUSH		{ R0, R1, R7, LR }
	MOV		R7, #0
	ARMV7_MPU_DISABLE_REGIONS	MPU_NTASK_REGIONS
	POP		{ R0, R1, R7, PC }
