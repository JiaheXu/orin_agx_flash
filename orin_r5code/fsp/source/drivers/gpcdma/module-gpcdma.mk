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
$(info Including:  module-gpcdma.mk)
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
MODULE_GPCDMA_NAME                  := GPCDMA

#
# Define a something similar to header guard protection...for makefiles
# instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_GPCDMA_DEFINED               := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
CONFIG_GPCDMA_DEPENDS               := GPCDMA SOC CPL
$(foreach _,$(CONFIG_GPCDMA_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# C Source
MODULE_GPCDMA_C_SRC                  =
MODULE_GPCDMA_C_SRC                 += $(MODULE_GPCDMA_DRIVERS_DIR)/gpcdma.c
MODULE_GPCDMA_C_SRC                 += $(CPL_SOC_IDS)/gpcdma-ids.c

#
# If the client module does not need 'gpcdma-port.c' use this MAKEFILE flag to
# ignore it.
MODULE_GPCDMA_C_SRC                 += $(CPL_SOC_PORT)/gpcdma-port.c

#
# Assembly source
MODULE_GPCDMA_ASM_SRC                =
MODULE_GPCDMA_ASM_SRC               += $(CPL_SOC_PORT)/gpcdma-irq.S

#
# Special C Flags
MODULE_GPCDMA_C_FLAGS                =

#
# Special ASM Flags
MODULE_GPCDMA_ASM_FLAGS              =