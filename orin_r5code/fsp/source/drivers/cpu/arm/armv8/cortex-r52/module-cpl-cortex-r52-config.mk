#
# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

#
# Print makefile diagnostic message
ifeq ($(VERBOSE), 1)
$(info Including:  module-cpl-cortex-r52-config.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
#   TOC:  Declare the include file directories needed by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-cpl_*.mk' files.
MODULE_CPL_CONFIG                   := 1
MODULE_CPL_CORTEX_R52_CONFIG        := 1

#
# Define ARCH variables
ARCH                                := arm
ISA                                 := armv8
CPU                                 := cortex-r52

#
# Location of source code files
CPL_SOC_IDS                          = $(FSP_COMMON_REPO)/soc/$(SOC)/ids/$(CLUSTER)
CPL_SOC_IDS_CMN                      = $(FSP_COMMON_REPO)/soc/$(SOC)/ids/soc-common
CPL_SOC_PORT                         = $(FSP_COMMON_REPO)/soc/$(SOC)/port/$(CLUSTER)
CPL_ARCH_DIR                         = $(FSP_COMMON_REPO)/drivers/cpu/$(ARCH)
CPL_ISA_DIR                          = $(FSP_COMMON_REPO)/drivers/cpu/$(ARCH)/$(ISA)
MODULE_CPL_CORTEX_R52_SOURCE_DIR     =
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_SOC_IDS)
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_SOC_IDS_CMN)
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_SOC_PORT)
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_VIC_DIR)
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_ISA_DIR)/$(CPU)
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_ARCH_DIR)/common
MODULE_CPL_CORTEX_R52_SOURCE_DIR    += $(CPL_ISA_DIR)

#
# Common includes '-I <INCLUDEDIR>'
MODULE_CPL_CORTEX_R52_INCLUDES       = -I $(FSP_COMMON_REPO)/include/osa/$(OSA)/$(ARCH)/$(ISA)/$(CPU)
MODULE_CPL_CORTEX_R52_INCLUDES      += -I $(FSP_COMMON_REPO)/include/cpu/$(ARCH)/$(ISA)
MODULE_CPL_CORTEX_R52_INCLUDES      += -I $(FSP_COMMON_REPO)/include/cpu/$(ARCH)/$(ISA)/$(CPU)
MODULE_CPL_CORTEX_R52_INCLUDES      += -I $(FSP_COMMON_REPO)/include/cpu/$(ARCH)/common
