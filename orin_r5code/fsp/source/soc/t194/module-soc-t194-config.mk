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
$(info Including:  module-soc-t194-config.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
#   TOC:  Declare the include file directories needed by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_SOC_CONFIG               := 1
MODULE_SOC_T194_CONFIG          := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
MODULE_SOC_DEPENDS              :=
$(foreach _,$(MODULE_SOC_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# Define SOC variables
HW_PLATFORM                      = t19x
SOC                              = t194

#
# Include <hwinc-*> directory rules
ifndef NV_HWINC_T19X_CL
include $(TEGRA_TOP)/hwinc-$(HW_PLATFORM)/rules.mk
endif

#
# Location of source code files
MODULE_SOC_T194_SOURCE_DIR       =

#
# Common includes '-I <INCLUDEDIR>'
MODULE_SOC_T194_INCLUDES         = -I $(TEGRA_TOP)/hwinc-$(HW_PLATFORM)/$(NV_HWINC_T19X_CL)
MODULE_SOC_T194_INCLUDES        += -I $(TEGRA_TOP)/hwinc-private/$(HW_PLATFORM)/$(NV_HWINC_T19X_CL)
MODULE_SOC_T194_INCLUDES        += -I $(FSP_COMMON_REPO)/include/soc/$(SOC)
