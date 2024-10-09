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
$(info Including:  module-ospl-config.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
#   TOC:  Declare the include file directories needed by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-cpl_*.mk' files.
MODULE_OSPL_CONFIG              := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
MODULE_OSPL_DEPENDS             := SOC CLUSTER CPL OSA FSP_COMMON
$(foreach _,$(MODULE_OSPL_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-osp*.mk' files.
MODULE_OSPL_DEFINED             := 1

#
# Location of source code files
OSPL_DIR                         = $(FSP_COMMON_REPO)/drivers/osa/$(OSA)/$(ARCH)/$(ISA)/$(CPU)
MODULE_OSPL_SOURCE_DIR           = $(OSPL_DIR)

#
# Common includes '-I <INCLUDEDIR>'
MODULE_OSPL_INCLUDES             =
