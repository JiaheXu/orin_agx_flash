#
# Copyright (c) 2021 NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

#
# Print makefile utilities message
ifeq ($(VERBOSE), 1)
$(info Including:  module-libc-lite-config.mk)
endif

#
# Table of Contents:
#   TOC:  Define the module name
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to
#   VPATH
#   TOC:  Declare the include file directories needed by this module
#   TOC:  Declare the C language files
#   TOC:  Declare the Assembly language files
#   TOC:  Declare C-Flags used by this module
#   TOC:  Declare ASM-Flags used by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_LIBC_LITE_CONFIG                 := 1

#
# Define a something similar to header guard protection...for makefiles
# instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_LIBC_LITE_DEFINED                := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
MODULE_LIBC_LITE_DEPENDS                := CPL
$(foreach _,$(MODULE_LIBC_LITE_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# Location of source code files
MODULE_LIBC_LITE_DRIVERS_DIR             = $(FSP_COMMON_REPO)/drivers/libc-lite
MODULE_LIBC_LITE_SOURCE_DIR              = $(MODULE_LIBC_LITE_DRIVERS_DIR)

#
# Common includes '-I <INCLUDEDIR>'
MODULE_LIBC_LITE_INCLUDES                =