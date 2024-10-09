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
#include <stdbool.h>                            // for bool, true, false
#include <stdint.h>                             // for uint32_t, uintptr_t
#include <string.h>                             // for NULL, memcpy, size_t

/* Early FSP headers */
#include <misc/ct-assert.h>                     // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <ospl/rtos-port.h>                     // for portBaseType, portUnsignedB...
#include <cpu/type-conversion.h>                // for fsp_c_v_ptr_to_uptr
#include <debug/assert.h>                       // for ASSERT, FSP__DEBUG__A...
#include <misc/attributes.h>                    // for WEAK, UNUSED, FSP__MI...
#include <misc/bitops.h>                        // for BIT, bit_number, BIT32
#include <misc/macros.h>                        // for U32_C, FSP_MISRA_BLOC...
#include <misc/nvrm_drf.h>                      // for NV_DRF_NUM, FSP__MISC...

/* Module-specific FSP headers */
#include <armv7-mpu.h>                          // for MPU_MAX_REGIONS, MPU_...
#include <cpu/armv7-mpu.h>                      // for R5_DRSR_0_EN_LSB, R5_...
#include <cpu/armv7-mpu-regs.h>                 // for r5mpu_region_disable
#include <cpu/armv7-mpu-safertos-priv.h>        // for vPortTaskHandleToMPUP...
#include <cpu/armv7-mpu-safertos.h>             // for mpuParameters_t, r5mp...
#include <cpu/sections-armv7-mpu.h>             // Immune from CT_ASSERT protection
#include <cpu/barriers.h>                       // for barrier_compiler, bar...
#include <cpu/built-ins.h>                      // for arm_builtin_clz, FSP_...

#include <irq/safe-irqs.h>

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__OSPL__RTOS_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_REGS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_SAFERTOS_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARMV7_MPU_SAFERTOS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_9, "Approval: Bug 200532001, DR: SWE-FSP-016-SWSADR.docx")
/**
 * @file armv7-mpu.c
 * @brief Implements the various MPU interfaces
 *
 * The implementation of the various function that are exposed to the
 * application firmware and those functions that are needed by SafeRTOS.
 */

/**
 * @brief MPU Initialized
 *
 * Indicates if the various MPU structures (and the MPU HW itself) has
 * been initialized.
 *
 * This is intended to be a catch if the various callbacks provided by
 * this code to SafeRTOS are called without the MPU structures being
 * initialized.
 */
static bool                     r5mpu_initialized       SECTION_MPU_DATA = false;

/**
 * @brief MPU Global Regions
 *
 * This is here as a "weak" definition in case the application firmware
 * doesn't define it.
 */
mpuParameters_t                 mpu_global_regions[MPU_NGLOBAL_REGIONS] WEAK
                                                                SECTION_MPU_DATA;

/**
 * @brief Task regions array for the xApplicationCopy* functions.
 *
 * This is an array of pointers to mpuParameters structures that are ordered
 * based upon the index of the pointed to mpuParameters structures.  The
 * array itself represents the ordered regions that apply to the current
 * task.
 *
 * It is permissable to have NULL pointers in this array.
 *
 * This array  will be updated when one of the xApplicationCopy* functions
 * is called and the task handle is different from the previous task handle.
 */
static mpuParameters_t          *task_regions[MPU_MAX_REGIONS] SECTION_MPU_DATA;

/**
 * @brief  Kernel regions array for the xApplicationCopy* functions.
 *
 * This is an array of pointers. Each valid pointer points to an
 * mpuParameters structure. Each mpuParameters structure
 * contains information to manage per-task MPU regions. Each
 * region has a unique index value defined within the
 * mpuParameters structure for that region. This array of
 * pointers is ordered according to those region index values
 * and represents the ordered regions that are always accessible
 * to the kernel or any code that is not running in task context
 * (e.g. ISRs).
 *
 * It is permissable to have NULL pointers in this array.
 */
static mpuParameters_t          *kernel_regions[MPU_MAX_REGIONS] SECTION_MPU_DATA;

/**
 * @brief Previous Task Handle
 *
 * This contains the task handle of the previous task that performed
 * any xApplicationCopy* functions.  It is used to determine if the
 * task_regions array needs to be updated to reflect the regions of
 * a new task.
 */
static portTaskHandleType       prev_check_task SECTION_MPU_DATA = NULL;

/**
 * @brief Array to map R5 MPU permissions to r5mpu_check_access_t
 *
 * This maps the access permissions contained in the access field in
 * the mpuParameters structure to a value suitable for the permissions
 * field.
 *
 * This array is only accessed by r5mpu_init() and r5mpu_task_region_init()
 * and is not accessed again.
 */
static uint8_t                  mpu_access_check_map[8] SECTION_MPU_INIT_DATA = {
    [0] = R5MPU_CHECK_PRIV_NONE | R5MPU_CHECK_USER_NONE,
    [1] = R5MPU_CHECK_PRIV_RW   | R5MPU_CHECK_USER_NONE,
    [2] = R5MPU_CHECK_PRIV_RW   | R5MPU_CHECK_USER_READ,
    [3] = R5MPU_CHECK_PRIV_RW   | R5MPU_CHECK_USER_RW,
    [4] = R5MPU_CHECK_PRIV_NONE | R5MPU_CHECK_USER_NONE,
    [5] = R5MPU_CHECK_PRIV_READ | R5MPU_CHECK_USER_NONE,
    [6] = R5MPU_CHECK_PRIV_READ | R5MPU_CHECK_USER_READ,
    [7] = R5MPU_CHECK_PRIV_NONE | R5MPU_CHECK_USER_NONE,
};

/**
 * @brief R5 MPU not initialized hook
 *
 * This is a default hook that can be overridden by the application
 * to handle cases when one of the MPU functions is called when the
 * MPU has not been initialized.
 *
 * @return None
 */
SECTION_MPU_TEXT WEAK void
r5mpu_not_initialized_hook(void)
{
    /*
     * This function does nothing as it is the default hook.
     * The application firmware can override this if it wants
     * to perform some other operation (like abort).
     */
    barrier_compiler();
}

/**
 * @brief R5 MPU re-initialization attempted
 *
 * This is a default hook that can be overridden by the application
 * to handle cases when re-initialization of the MPU is attempted.
 *
 * @return None
 */
SECTION_MPU_TEXT WEAK void
r5mpu_reinitialize_hook(const mpuParameters_t * const region)
{
    /*
     * This function does nothing as it is the default hook.
     * The application firmware can override this if it wants
     * to perform some other operation (like abort).
     */
    UNUSED(region);
    barrier_compiler();
}

/**
 * @brief Initialize task_regions array for current task
 *
 * This function will initialize the task's check region array for use
 * by the other check range functions.
 *
 * @pre r5mpu_init() must have been called.
 * @pre r5mpu_task_region_init() must have been called for the task.
 * @pre the supplied task handle must represent a valid task.
 *
 * @param[in]  task     task handle of the task who's ranges are to be
 *                      initialized
 * @param[in]  regions  pointer to the array containing the task's
 *                      mpuParameters structures
 *
 * @return None
 */
SECTION_MPU_TEXT static void
vApplicationCheckTaskRangeInit(portTaskHandleType task,
                               mpuParameters_t * regions)
{
    uint32_t            i;
    mpuParameters_t     const *prev_region;

    /*
     * See if the task check array is still valid
     */
    if ((prev_check_task != NULL)
        && (prev_check_task == task)) {
        goto out;
    }

    /*
     * Clear out the entries in the task check array from the previous
     * task
     */
    if (prev_check_task != NULL) {
        prev_region = vPortTaskHandleToMPUPointer(prev_check_task);

        if (prev_region != NULL) {
            for (i = 0UL; i < U32_C(MPU_NTASK_REGIONS); i += 1UL) {
                task_regions[prev_region[i].index] = NULL;
            }
        }
    }

    /*
     * Set up the task check array for the new task
     */
    if (regions != NULL) {
        for (i = 0UL; i < U32_C(MPU_NTASK_REGIONS); i += 1UL) {
            /*
             * Fill the task_regions[i] with the current
             * task's regions.
             */
            task_regions[regions[i].index] = &regions[i];
        }
    }

    prev_check_task = task;

  out:
    return;
}

/**
 * @brief Check a range to determine if it is covered by the region
 *
 * This function will check a single mpuParameters structure to determine
 * if the specified range falls within it and has the appropriate access
 * permissions.  It will update the start and length if any part of the
 * supplied range overlaps the region and has the correct access permissions.
 *
 * @pre None
 *
 * @param[in]     region   pointer to a single mpuParameters_t structure
 * @param[in,out] start    pointer to starting address the of range.  The value
 *                         pointed to will potentially be updated if it the
 *                         value is within the region
 * @param[in,out]  length  pointer to the length of therange in bytes.  The value
 *                         pointed to will potentially be updated if the range
 *                         exists within the region
 *
 * @retval true  Part of the range falls within the region
 * @retval false No part of the range falls within the region
 */
SECTION_MPU_TEXT static bool
bApplicationCheckRange(mpuParameters_t const * region,
                       uintptr_t *start,
                       uint32_t *length)
{
    uintptr_t           begin           = *start;
    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    uintptr_t           end             = *start + *length - 1UL;
    uintptr_t           region_end;
    bool                matched         = false;

    /*
     * Determine if any part of the supplied range is within the region
     */
    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    region_end = region->base + region->size - 1UL;

    /*
     * Determine if there is any part of the range that is covered
     * by the region
     */
    if ((begin >= region->base)
        && (end <= region_end)) {
        /*
         * The range is completely contained within the region
         * so there will be nothing left to check
         */
        *start = 0UL;
        *length = 0UL;

        matched = true;
    } else if ((end >= region->base)
               && (end <= region_end)) {
        /*
         * The end of the range overlaps the region so adjust
         * the length but leave the start alone.
         */
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
        *length -= end - region->base + 1UL;
        matched = true;
    } else if ((begin >= region->base)
               && (begin <= region_end)) {
        /*
         * The start of the range overlaps the region so adjust
         * both the start and length.
         */
        *start = region_end + 1UL;
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
        *length = end - *start + 1UL;
        matched = true;
    } else {
        /*
         * Nothing to do in this case.
         */
    }

    return matched;
}

/**
 * @brief Check that range has access
 *
 * This function will check that the address range and required permissions
 * are allowed.
 *
 * @pre None
 *
 * @param[in]  regions  pointer to an array of ordered regions to check
 * @param[in]  access   access permissions required
 * @param[in]  addr     address in task
 * @param[in]  length   length of range
 *
 * @retval  true   range is valid against the supplied ordered list of regions
 *                 and has the requested permissions.
 * @retval  false  range is not valid against the supplied ordered list of
 *                 regions or does not have the requested permissions.
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_MPU_TEXT static bool bApplicationCheckAddrRange(mpuParameters_t *regions[],
                           r5mpu_check_access_t access,
                           const void * const addr,
                           uint32_t length)
{
    bool                valid           = false;
    uint32_t            i;
    uintptr_t           start;
    uint32_t            remaining       = length;
    mpuParameters_t     const *cur_region;
    r5mpu_check_access_t cur_access     = access;

    /*
     * If we get a 0 length then pass it.
     */
    if (length == 0UL) {
        valid = true;
        goto out;
    }

    /*
     * A request to copy from NULL with a non-zero length
     * will fail.
     */
    if (addr == NULL) {
        goto out;
    }

    start = fsp_c_v_ptr_to_uptr(addr);

    /*
     * Make sure that the address/length combination makes sense,
     * that is that addr + length < UINT32_MAX
     */
    if (start > (UINTPTR_MAX - length)) {
            goto out;
    }

    /*
     * Scan through the task regions and see if access is allowed
     */
    for (i = 0UL; i < U32_C(MPU_MAX_REGIONS); i += 1UL) {
        cur_region = regions[U32_C(MPU_MAX_REGIONS) - i - 1UL];

        /*
         * Skip the region if it isn't valid
         */
        if ((cur_region == NULL)
            || ((cur_region->flags & R5MPU_FL_VALID) == 0U)) {
            continue;
        }

        if (bApplicationCheckRange(cur_region, &start, &remaining)) {
            INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-616, DR: SWE-FSP-050-SWSADR.docx");
            cur_access &= access & cur_region->permissions;
            if (remaining == 0UL) {
                if (cur_access == access) {
                    valid = true;
                }
                break;
            }
        }
    }

  out:
    return valid;
}

/**
 * @brief Check that a task's address range is valid
 *
 * This function will check that a task's address range is valid for
 * the requested permissions.
 *
 * @param[in]  taskAddr  Address in the task's address space to be checked
 * @param[in]  uxLength  Length of range to be checked
 * @param[in]  access    permissions to be used for the check
 * @param[in]  priv      indicates if the task is privileged (true) or
 *                       unprivileged (false)
 *
 * @retval  true   task's region is valid for the permissions
 * @retval  false  task's region is not valid for the permissions
 */
SECTION_MPU_TEXT static bool
bCheckTaskAddrRange(const void *taskAddr,
                    portUnsignedBaseType uxLength,
                    r5mpu_check_access_t access,
                    bool priv)
{
    bool                rc              = true;

#if ((defined(MPU_COPY_PRIV_CHECK)) && (MPU_COPY_PRIV_CHECK == 1))
    UNUSED(priv);
#else
    if (!priv)
#endif
    {
        rc = bApplicationCheckAddrRange(task_regions,
                                        access, taskAddr,
                                        (uint32_t)uxLength);
     }

    return rc;
}

/**
 * @brief Check to see if a copy is valid
 *
 * This function will determine if the requested copy can be performed.
 *
 * @param[in]  xCurrentTaskHandle  handle to the current task whose context
 *                                 the copy will be performed under.  If NULL
 *                                 assume the context is privileged, otherwise
 *                                 use the task's privilege
 * @param[in]  pvDestAddr          destination address (e.g. will be written to)
 * @param[in]  pvSourceAddr        source address (e.g. will be read from)
 * @param[in]  uxLength            number of bytes in the copy
 * @param[in]  direction           indicates the direction of the copy
 *                                 - true  = task to kernel
 *                                 - false = kernel to task
 *
 * @retval  true   copy can proceed
 * @retval  false  copy cannot proceed
 */
SECTION_MPU_TEXT static bool
bApplicationCheckCopyValid(portTaskHandleType xCurrentTaskHandle,
                           const void *pvDestAddr,
                           const void *pvSourceAddr,
                           portUnsignedBaseType uxLength,
                           bool direction)
{
    bool                        rc              = true;

    /*
     * Allow the copy to proceed if the length is 0 because it
     * won't do anything anyway.
     */
    if (uxLength == 0UL) {
        goto out;
    }

    if ((pvDestAddr == NULL)
        || (pvSourceAddr == NULL)) {
        rc = false;
        goto out;
    }

#if (defined(MPU_COPY_CHECK) && (MPU_COPY_CHECK==1))
    bool                        priv;
    const void                  *kernelAddr;
    const void                  *taskAddr;
    r5mpu_check_access_t        kernAccess;
    r5mpu_check_access_t        taskAccess;
    mpuParameters_t             *region;

    if (!r5mpu_initialized) {
        r5mpu_not_initialized_hook();
        goto out;
    }

    /*
     * Determine if the task is privileged or not.
     */
    priv = (xCurrentTaskHandle == NULL) ? true
           : (uxPortTaskSystemModeSetting(xCurrentTaskHandle) == R5_PSR_MODE_SYSTEM);

    if (direction) {
        kernelAddr = pvDestAddr;
        kernAccess = R5MPU_CHECK_PRIV_WRITE;
        taskAddr   = pvSourceAddr;
        taskAccess = (priv) ? R5MPU_CHECK_PRIV_READ : R5MPU_CHECK_USER_READ;
    } else {
        kernelAddr = pvSourceAddr;
        kernAccess = R5MPU_CHECK_PRIV_READ;
        taskAddr   = pvDestAddr;
        taskAccess = (priv) ? R5MPU_CHECK_PRIV_WRITE : R5MPU_CHECK_USER_WRITE;
    }

    /*
     * Setup the task's region check array.
     */
    region = vPortTaskHandleToMPUPointer(xCurrentTaskHandle);

    vApplicationCheckTaskRangeInit(xCurrentTaskHandle, region);

    /*
     * Make sure that the kernel address falls within the
     * expected ranges and has the correct permissions.
     */
    if (!bCheckTaskAddrRange(kernelAddr, uxLength, kernAccess, priv)) {
        rc = false;
        goto out;
    }

    /*
     * Make sure that the task's address falls within the
     * expected ranges and has the correct permissions.
     */
    if (!bCheckTaskAddrRange(taskAddr, uxLength, taskAccess, priv)) {
        rc = false;
        goto out;
    }


#endif

  out:

    return rc;
}

/**
 * @brief Check to see if a copy in an ISR is valid
 *
 * This function will determine if the requested copy can be performed
 * while in an ISR
 *
 * @param[in]  pvDestinationAddr   destination address (e.g. will be written to)
 * @param[in]  pvSourceAddr        source address (e.g. will be read from)
 * @param[in]  uxLength            number of bytes in the copy
 *
 * @retval  true   copy can proceed
 * @retval  false  copy cannot proceed
 */
SECTION_MPU_TEXT static bool
bApplicationCheckISRAddrRange(const void *pvDestinationAddr,
                              const void *pvSourceAddr,
                              portUnsignedBaseType uxLength)
{
    bool        rc      = true;

    /*
     * Allow the copy to proceed if the length is 0 because it
     * won't do anything anyway.
     */
    if (uxLength == 0UL) {
        goto out;
    }

    if ((pvDestinationAddr == NULL)
        || (pvSourceAddr == NULL)) {
        rc = false;
        goto out;
    }

#if (defined(MPU_COPY_CHECK) && (MPU_COPY_CHECK==1))

    if (!r5mpu_initialized) {
        r5mpu_not_initialized_hook();
    } else {

#if (defined(MPU_COPY_KERNEL_CHECK) && (MPU_COPY_KERNEL_CHECK==1))

        if ((!bApplicationCheckAddrRange(kernel_regions,
                                         R5MPU_CHECK_PRIV_WRITE,
                                         pvDestinationAddr,
                                         (uint32_t)uxLength))
            || (!bApplicationCheckAddrRange(kernel_regions,
                                            R5MPU_CHECK_PRIV_READ,
                                            pvSourceAddr,
                                            (uint32_t)uxLength))) {
            rc = false;
        }

#endif

    }

#endif

  out:
    return rc;
}

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
SECTION_MPU_INIT_TEXT void
r5mpu_region_init(mpuParameters_t *const region,
                  bool set_mpu)
{
    uint32_t            access;
    uint32_t            size;
    uint32_t            first_bit;

    ASSERT(region != NULL);
    ASSERT(region->index < U32_C(MPU_MAX_REGIONS));

    /*
     * Skip over regions that have already been initialized
     * calling the reinitialize hook as necessary.
     */
    if ((region->flags & R5MPU_FL_INIT) != 0U) {
        r5mpu_reinitialize_hook(region);
        goto out;
    }

    if ((region->flags & R5MPU_FL_VALID) != 0U) {
        /*
         * Deal with regions that couldn't specify the size at compile time
         */
        if ((region->flags & R5MPU_FL_END) != 0U) {
            ASSERT(region->size > region->base);

            size = region->size - region->base;
            region->size = size;
            INLINE_RFD(CERTC, FP, INT31_C, "Approval: JIRA TID-616, DR: SWE-FSP-050-SWSADR.docx");
            region->flags &= ~R5MPU_FL_END;
            region->flags |= R5MPU_FL_SIZE;
        }

        ASSERT((region->flags & (uint32_t)R5MPU_FL_SIZE) != 0U);

        access = (uint32_t)(region->access & R5_DRACR_AP_MASK) >> R5_DRACR_AP_SHIFT;
        region->permissions = mpu_access_check_map[access];

        /*
         * The minimum size of a region is 32 bytes, so force the size
         * to be at least 32 bytes.
         */
        size = region->size;
        if (size < 32UL) {
            size = 32UL;
        }

        /*
         * Make sure that the size is a power-of-2.  Adjust the
         * resultant size to be the next larger power-of-2 as
         * necessary.
         */
        first_bit = 31UL - arm_builtin_clz(size);

        if (BIT(first_bit) < size) {
            first_bit += 1UL;
            size = BIT(first_bit);
        }

        /*
         * Clear the bits of base that correspond to the size
         * That is, we know that size is now a power-of-2, so
         * clear the low order bits of the base address so that
         * it too is aligned to the same power-of-2.
         *
         * NOTE: it is OK to do this, because the R5 MPU would
         * have done it, so nothing is really changed with this
         * in terms of correct/incorrect behavior.
         */
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        region->base &= ~(size - 1UL);
        region->size = size;

        region->encoded_size = NV_DRF_NUM(R5, DRSR, RSIZE,
                                          r5mpu_size(size))
                               | NV_DRF_NUM(R5, DRSR, EN, 1UL);
        if (set_mpu) {
            r5mpu_region_config(region->index,
                                region->base,
                                region->encoded_size,
                                region->access);
        }
    } else {
        region->encoded_size = 0UL;
        if (set_mpu) {
            r5mpu_region_disable(region->index);
        }
    }

  out:
    return;
}

/**
 * @brief Initialize a task's regions
 *
 * This function will initialize the mpuParameters structures in the passed
 * array (that contains exactly MPU_NTASK_REGIONS structures).  It does *not*
 * program the MPU with those mappings.
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
SECTION_MPU_INIT_TEXT void
r5mpu_task_region_init(mpuParameters_t *task_mpu_regions)
{
    uint32_t            i;
    uint32_t            used;
    uint32_t            unused_index;
    uint32_t            bit;
    uint32_t            max_regions_mask;

    if (!r5mpu_initialized) {
        r5mpu_not_initialized_hook();
        goto out;
    }

    used = 0UL;

    if (task_mpu_regions != NULL) {

        max_regions_mask = BIT(MPU_MAX_REGIONS) - 1UL;

        /*
         * Go through and see which are the used indecies
         * first go through the global regions.
         *
         * Assumption here is that the indecies for all of
         * the global regions are defined regardless of the
         * region being enabled or not.
         */
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        for (i = 0UL; i < (uint32_t)MPU_NGLOBAL_REGIONS; i += 1UL) {
            bit = BIT32_FN(mpu_global_regions[i].index);
            ASSERT((used & bit) == 0UL);
            used |= bit;
        }

        /*
         * Now go through and figure out which indecies
         * are used by the task.
         */
        for (i = 0UL; i < U32_C(MPU_NTASK_REGIONS); i += 1UL) {
            if ((task_mpu_regions[i].flags & R5MPU_FL_VALID) != 0U) {
                bit = BIT32_FN(task_mpu_regions[i].index);
                ASSERT((used & bit) == 0UL);
                used |= bit;
            }
        }

        /*
         * Sanity check to make sure used does not have invalid indices
         */
        ASSERT((used & (~max_regions_mask)) == 0UL);

        /*
         * By now we should have all the used indices. Now go through
         * and assign unused MPU indices to the disabled per-task regions
         */
        for (i = 0UL; i < U32_C(MPU_NTASK_REGIONS); i += 1UL) {
            if ((task_mpu_regions[i].flags & R5MPU_FL_VALID) == 0U) {
                /*
                 * find the bit position of the first zero
                 */
                unused_index = bit_number(~used);
                task_mpu_regions[i].index = unused_index;
                used |= BIT32_FN(unused_index);
            }
        }

        /*
         * Initialize the task's MPU region structures
         */
        for (i = 0UL; i < U32_C(MPU_NTASK_REGIONS); i += 1UL) {
            r5mpu_region_init(&task_mpu_regions[i], false);
        }
    }

  out:
    return;
}

/**
 * @brief Iinitialize the MPU
 *
 * This function will initialize the MPU initializing the global array
 * of mpuParameters structures, writing the appropriate values contained
 * in those structures into the MPU, disabling any unused (including task
 * related MPU entries) and doing the overall enabling of the MPU.
 *
 * @pre None
 *
 * @return None
 */
SECTION_MPU_INIT_TEXT void
r5mpu_init(void)
{
    uint32_t            i;
    uint32_t            n_regions;
    mpuParameters_t     *region;

    ASSERT((U32_C(MPU_MAX_REGIONS)) <= r5mpu_region_count());

    /*
     * Prevent re-initializing the MPU data structures and MPU HW
     * once they've already been initializaed.
     */
    if (r5mpu_initialized) {
        r5mpu_reinitialize_hook(NULL);

    }

    n_regions = r5mpu_region_count();

    r5mpu_disable();

    /*
     * Disable all regions
     */
    for (i = 0UL; i < n_regions; i += 1UL) {
        r5mpu_region_disable(i);
    }

    for (i = 0UL; i < U32_C(MPU_MAX_REGIONS); i += 1UL) {
        task_regions[i] = NULL;
        kernel_regions[i] = NULL;
    }

    /*
     * Initialize the MPU with the global regions
     */
    INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
    for (i = 0UL; i < (uint32_t)MPU_NGLOBAL_REGIONS; i += 1UL) {
        region = &mpu_global_regions[i];

        r5mpu_region_init(region, true);

        /*
         * Place the region based upon it's priority
         */
        if ((region->flags & R5MPU_FL_VALID) != 0U) {
            task_regions[region->index] = region;
            kernel_regions[region->index] = region;
        }
    }

    r5mpu_enable();
    barrier_memory_complete();

    r5mpu_initialized = true;

    return;
}

/**
 * @brief Application specific task switch handling
 *
 * This function is called by the context switch code in SafeRTOS.
 * It will perform the necessary low level operations necessary to
 * tear down the MPU mappings from the current task and set them up
 * for the next task.
 *
 * @pre MPU must be enabled via r5mpu_init().
 * @pre The task's region arrays must be properly initialized by
 *      calling r5mpu_task_region_init().
 * @pre The handle specified by currentTask must represent a valid task
 * @pre The handle specified by nextTask must represent a valid task
 *
 * @param[in] currentTask  A valid task handle to the current task
 *                         or NULL.
 * @param[in] nextTask     A valid task handle to the next task
 *                         or NULL.
 *
 * @return None
 */
SECTION_MPU_TEXT void
vApplicationTaskSwitch(const xTCB *pxTCBOfTaskSwitchedOut,
                       const xTCB *pxTCBOfTaskSwitchedIn)
{
    mpuParameters_t     *cur_region;
    mpuParameters_t     *next_region;

    if (!r5mpu_initialized) {
        r5mpu_not_initialized_hook();
        goto out;
    }

    next_region = (pxTCBOfTaskSwitchedIn == NULL) ? NULL :
                    pxTCBOfTaskSwitchedIn->pxMPUParameters;
    cur_region = (pxTCBOfTaskSwitchedOut == NULL) ? NULL :
                    pxTCBOfTaskSwitchedOut->pxMPUParameters;

    /*
     * Only restore the MPU task mappings if:
     * - there are actually task mappings (by the build)
     * - the current task and next task are different
     * - the MPU region pointers are different
     */
    if ((U32_C(MPU_NTASK_REGIONS) > 0UL)
        && (pxTCBOfTaskSwitchedOut != pxTCBOfTaskSwitchedIn)
        && (cur_region != next_region)) {

        /*
         * If the next task has per-task mappings, then set them up
         * otherwise disable the per-task mappings.
         */
        if (next_region != NULL) {
            armv7_task_mpu_restore(next_region);
        } else {
            /*
             * Next task has no per task regions, so disable
             * the per task MPU entries to avoid erroneous mappings.
             *
             * If current task has no per task regions either,
             * nothing needs to be done as the per-task regions have
             * already been disabled.
             */
            if (cur_region != NULL) {
                armv7_task_mpu_disable(cur_region);
            }
        }
    }

  out:
    return;
}

/**
 * @brief Copy data from a task to kernel
 *
 * This function will copy data from the specified task to kernel and will
 * check to make sure that:
 * -# the address specified for the task is readable by the task
 * -# the address specified for the kernel is writable
 *
 * This function must be called from a non-interrupt context.
 *
 * This function is only callable from privileged mode.  It will return an
 * error if called from user mode.
 *
 * @pre r5mpu_init() must have been called to set up the global regions
 * @pre r5mpu_task_region_init() must have been called to setup the task's
 *      regions if there are any for the task.
 * @pre the task handle specified by fromTask must represent a valid task.
 *
 * @param[in]  fromTask     task being copied from
 * @param[in]  kernelAddr   address in the kernel being copied to (e.g.
 *                          destination)
 * @param[in]  taskAddr     address in the task being copied from (e.g. source)
 * @param[in]  length       number of bytes being copied
 *
 * @retval   pdPASS   Copy occurred
 * @retval   pdFAIL   Copy could not proceed
 */
SECTION_MPU_TEXT portBaseType
xApplicationCopyDataFromTask(portTaskHandleType xCurrentTaskHandle,
                             void *pvKernelDestinationAddress,
                             const void *pvApplicationSourceAddress,
                             portUnsignedBaseType uxLength)
{
    portBaseType                rc              = pdFAIL;

    enter_critical();

    if (bApplicationCheckCopyValid(xCurrentTaskHandle,
                                   (const void *)pvKernelDestinationAddress,
                                   pvApplicationSourceAddress,
                                   uxLength, true)) {

        /*
         * It is possible to have NULL values for the addresses in certain
         * cases.  It will only happen when uxLength is 0.  However, the
         * operation of memcpy() is undefined if either of the pointers is NULL
         * even if the length is 0.  So to avoid the undefined behavior we skip
         * calling memcpy() in those cases.
         *
         * bApplicationCheckCopyValid will eliminate all other cases where the
         * pointers are NULL, so it is only necessary to check for NULL and not
         * call memcpy() in those cases (the copy would have had a 0 length and
         * not done anything anyway).
         */
        if ((pvKernelDestinationAddress != NULL) && (pvApplicationSourceAddress != NULL)) {
            (void)memcpy(pvKernelDestinationAddress,
                         pvApplicationSourceAddress,
                         (size_t)uxLength);
        }
        rc = pdPASS;
    }

    exit_critical();

    return rc;
}

/**
 * @brief Copy data to a task from the kernel
 *
 * This function will copy data from the kernel to the specified task and will
 * check to make sure that:
 * -# the address specified for the task is writable by the task
 * -# the address specified for the kernel is readable
 *
 * This function must be called from a non-interrupt context.
 *
 * This function is only callable from privileged mode.  It will return an
 * error if called from user mode.
 *
 * @pre r5mpu_init() must have been called to set up the global regions
 * @pre r5mpu_task_region_init() must have been called to setup the task's
 *      regions if there are any for the task.
 * @pre the task handle specified by toTask must represent a valid task.
 *
 * @param[in]   toTask      task being copied from
 * @param[in]   taskAddr    address in the task being copied to (e.g.
 *                          destination)
 * @param[in[   kernelAddr  address in the kernel being copied from (e.g. source)
 * @param[in]   length      number of bytes being copied
 *
 * @retval  pdPASS   copy occurred
 * @retval  pdFAIL   copy could not proceed
 */
SECTION_MPU_TEXT portBaseType
xApplicationCopyDataToTask(portTaskHandleType xCurrentTaskHandle,
                void *pvApplicationDestinationAddress,
                const void *pvKernelSourceAddress,
                portUnsignedBaseType uxLength)
{
    portBaseType                rc              = pdFAIL;

    enter_critical();

    if (bApplicationCheckCopyValid(xCurrentTaskHandle,
                                   (const void *)pvApplicationDestinationAddress,
                                   pvKernelSourceAddress,
                                   uxLength, false)) {

        /*
         * It is possible to have NULL values for the addresses in certain
         * cases.  It will only happen when uxLength is 0.  However, the
         * operation of memcpy() is undefined if either of the pointers is NULL
         * even if the length is 0.  So to avoid the undefined behavior we skip
         * calling memcpy() in those cases.
         *
         * bApplicationCheckCopyValid will eliminate all other cases where the
         * pointers are NULL, so it is only necessary to check for NULL and not
         * call memcpy() in those cases (the copy would have had a 0 length and
         * not done anything anyway).
         */
        if ((pvApplicationDestinationAddress != NULL) && (pvKernelSourceAddress != NULL)) {
            (void)memcpy(pvApplicationDestinationAddress,
                         pvKernelSourceAddress,
                         (size_t)uxLength);
        }
        rc = pdPASS;
    }

    exit_critical();

    return rc;
}

/**
 * @brief Copy data from ISR to the kernel
 *
 * This function will copy data from the ISR into the kernel.  Since the ISR
 * will have the same access mappings as the kernel, it will only perform
 * kernel access checks.  That is, it cannot depend upon what the current
 * task has mapped.
 *
 * This function is only callable from an ISR.  It will return an
 * error if called when not in an ISR.
 *
 * @pre r5mpu_init() must have been called to set up the global regions.
 *
 * @param[in]   taskAddr    address in the task being copied from (e.g.
 *                          destination)
 * @param[in]   kernelAddr  address in the kernel being copied to (e.g. source)
 * @param[in]   mlength     number of bytes being copied
 *
 * @retval pdPASS   the copy occurred
 * @retval pdFAIL   the copy could not proceed
 */
SECTION_MPU_TEXT portBaseType
xApplicationCopyDataFromISR(void *pvKernelDestinationAddress,
                            const void *pvApplicationSourceAddress,
                            portUnsignedBaseType uxLength)
{
    portBaseType                rc              = pdFAIL;

    if (bApplicationCheckISRAddrRange(pvKernelDestinationAddress,
                                      pvApplicationSourceAddress,
                                      uxLength)) {

        /*
         * It is possible to have NULL values for the addresses in certain
         * cases.  It will only happen when uxLength is 0.  However, the
         * operation of memcpy() is undefined if either of the pointers is NULL
         * even if the length is 0.  So to avoid the undefined behavior we skip
         * calling memcpy() in those cases.
         *
         * bApplicationCheckCopyValid will eliminate all other cases where the
         * pointers are NULL, so it is only necessary to check for NULL and not
         * call memcpy() in those cases (the copy would have had a 0 length and
         * not done anything anyway).
         */
        if ((pvKernelDestinationAddress != NULL) && (pvApplicationSourceAddress != NULL)) {
            (void)memcpy(pvKernelDestinationAddress,
                         pvApplicationSourceAddress,
                         (size_t)uxLength);
        }
        rc = pdPASS;
    }

    return rc;
}

/**
 * @brief Copy data from task to ISR
 *
 * This function will copy data from the kernel into the ISR.  Since the ISR
 * will have the same access mappings as the kernel, it will only perform
 * kernel access checks.  That is, it cannot depend upon what the current
 * task has mapped.
 *
 * @pre r5mpu_init() must have been called to set up the global regions.
 *
 * This function is only callable from an ISR.  It will return an
 * error if called from user mode.
 *
 * @param[in]  kernelAddr  address in the kernel being copied from (e.g.
 *                         destination)
 * @param[in]  taskAddr    address in the task being copied to (e.g. source)
 * @param[in]  length      number of bytes being copied
 *
 * @retval pdPASS   the copy occurred
 * @retval pdFAIL   the copy could not proceed
 */
SECTION_MPU_TEXT portBaseType
xApplicationCopyDataToISR(void *pvApplicationDestinationAddress,
                          const void *pvKernelSourceAddress,
                          portUnsignedBaseType uxLength)
{
    portBaseType                rc              = pdFAIL;

    if (bApplicationCheckISRAddrRange((const void *)pvApplicationDestinationAddress,
                                      pvKernelSourceAddress,
                                      uxLength)) {

        /*
         * It is possible to have NULL values for the addresses in certain
         * cases.  It will only happen when uxLength is 0.  However, the
         * operation of memcpy() is undefined if either of the pointers is NULL
         * even if the length is 0.  So to avoid the undefined behavior we skip
         * calling memcpy() in those cases.
         *
         * bApplicationCheckCopyValid will eliminate all other cases where the
         * pointers are NULL, so it is only necessary to check for NULL and not
         * call memcpy() in those cases (the copy would have had a 0 length and
         * not done anything anyway).
         */
        if ((pvApplicationDestinationAddress != NULL) && (pvKernelSourceAddress != NULL)) {
            (void)memcpy(pvApplicationDestinationAddress,
                         pvKernelSourceAddress,
                         (size_t)uxLength);
        }
        rc = pdPASS;
    }

    return rc;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_9, "Approval: Bug 200532001, DR: SWE-FSP-016-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
