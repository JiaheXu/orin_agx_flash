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
$(info Including:  module-lic.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to
#   VPATH
#   TOC:  Declare the include file directories needed by this module
#   TOC:  Declare the C language files
#   TOC:  Declare the Assembly language files
#   TOC:  Declare C-Flags used by this module
#   TOC:  Declare ASM-Flags used by this module

#
# Define module name:
#   Once the module name is defined, it will be added to the MODULE_NAMES
#   list.  This can be done manually (like it is here), or automatically
#   on a per-application basis.
MODULE_LIC_NAME                 := LIC

#
# Define a something similar to header guard protection...for makefiles
# instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_LIC_DEFINED              := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
CONFIG_LIC_DEPENDS              := LIC SOC CPL
$(foreach _,$(CONFIG_LIC_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# C Source
MODULE_LIC_C_SRC                 =
MODULE_LIC_C_SRC                += $(MODULE_LIC_DRIVERS_DIR)/lic-tegra.c
MODULE_LIC_C_SRC                += $(CPL_SOC_IDS)/lic-tegra-ids.c

#
# Assembly source
MODULE_LIC_ASM_SRC               = $(CPL_SOC_PORT)/lic-irq.S

#
# Special C Flags
MODULE_LIC_C_FLAGS               =

#
# Special ASM Flags
MODULE_LIC_ASM_FLAGS             =
