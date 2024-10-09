/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <soc-common/hw-const.h>                        /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>

/* Late FSP headers */

/* Module-specific FSP header files */
#include <processor/irqs-hw.h>

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")

const uint32_t vic_base_addr[MAX_VIC_CONTROLLER] ={
        NV_ADDRESS_MAP_RCE_VIC_0_BASE,
        NV_ADDRESS_MAP_RCE_VIC_1_BASE
};
