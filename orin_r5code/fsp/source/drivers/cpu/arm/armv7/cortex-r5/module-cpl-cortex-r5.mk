#
# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
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
$(info Including:  module-cpl-cortex-r5.mk)
endif

#
# Table of Contents:
#   TOC:  Define the module name
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
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
MODULE_CPL_CORTEX_R5_NAME       := CPL_CORTEX_R5

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_CPL_DEFINED              := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
CONFIG_CPU_DEPENDS              := CPL SOC OSA DEBUG_NOSAFETY CPL_CORTEX_R5
$(foreach _,$(CONFIG_CPU_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# C Source
MODULE_CPL_CORTEX_R5_C_SRC       =
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_VIC_DIR)/irqapi-vic.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_VIC_DIR)/vic-init.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_SOC_IDS)/irqapi-vic-ids.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_ARCH_DIR)/common/arm-vic.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_ARCH_DIR)/$(CPU)/cache-cortex-r5.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_ISA_DIR)/armv7-exception.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_ISA_DIR)/armv7-mpu.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_ARCH_DIR)/common/irqapi-arm.c
MODULE_CPL_CORTEX_R5_C_SRC      += $(CPL_ARCH_DIR)/$(CPU)/irqapi-cortex-r5.c

#
# Assembly source
MODULE_CPL_CORTEX_R5_ASM_SRC     =
MODULE_CPL_CORTEX_R5_ASM_SRC    += $(CPL_VIC_DIR)/vic-asm.S
MODULE_CPL_CORTEX_R5_ASM_SRC    += $(CPL_ISA_DIR)/armv7-exceptions.S
MODULE_CPL_CORTEX_R5_ASM_SRC    += $(CPL_ISA_DIR)/armv7-mpu-asm.S

#
# Special C Flags
MODULE_CPL_CORTEX_R5_C_FLAGS     =

#
# Special ASM Flags
MODULE_CPL_CORTEX_R5_ASM_FLAGS   =

#
# Instructs the makefile system that there is a module-specific dependency.
# A few pieces are needed to make this work:
#   MODULE_$(MODULE_NAME)_TARGET_ENABLED             = 1 Tells the FSP makefile system that this module has a custom target
#   MODULE_$(MODULE_NAME)_TARGET_NAMES               = List of one or more target names (can be more than one)
#                                                    =   This recipe will be dynamically created in the master makefile;
#                                                        no need to create it here.
#   MODULE_CPL_CORTEX_R5_TARGET_RECIPE               = List of recipe/dependencies to be built (made) by $(MODULE_$(MODULE_NAME)_TARGET_NAMES)
#   $(MODULE_$(MODULE_NAME)_TARGET_NAMES)_AFLAGS     = Compiler ASM-FLAGS used to compile this module
#   $(MODULE_$(MODULE_NAME)_TARGET_NAMES)_CFLAGS     = Compiler C-FLAGS used to compile this module
MODULE_CPL_CORTEX_R5_TARGET_ENABLED                  = 1
MODULE_CPL_CORTEX_R5_TARGET_NAMES                    = MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS
MODULE_CPL_CORTEX_R5_TARGET_RECIPE                   = armv7-mpu-asm-dynamic-hdr.h
MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_AFLAGS          =
MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_CFLAGS          = -MT $(DYNAMIC_SRC_DIR)/$(addsuffix .s,$(notdir $(basename $@)-interim))
MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_CFLAGS         += -MMD
MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_CFLAGS         += -MP
MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_CFLAGS         += -MF $(DYNAMIC_DEP_DIR)/$(notdir $(basename $@)-interim).d

armv7-mpu-asm-dynamic-hdr.h: armv7-mpu-asm-dynamic-hdr.c | GLOBAL_dirs
	@echo  [$(SOC)][C-\>ASM] $(notdir $(subst $(TEGRA_TOP)/,,$<))
	@$(CC) -c -S $(MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_AFLAGS) \
                    $(MODULE_CPL_CORTEX_R5_DYNAMIC_HEADERS_CFLAGS) \
                    $< -o $(DYNAMIC_SRC_DIR)/$(addsuffix .s,$(notdir $(basename $@)-interim))
	@echo  [$(SOC)][ASM-\>HDR] $(notdir $(subst .c,.h,$(subst $(TEGRA_TOP)/,,$<)))
	@$(GREP) -o '#define[ \t][ \t]*[^( |\t)]*[ \t][ \t]*[^( |\t|\$)]*' $(DYNAMIC_SRC_DIR)/$(addsuffix .s,$(notdir $(basename $@)-interim)) > $(DYNAMIC_HDR_DIR)/$@

