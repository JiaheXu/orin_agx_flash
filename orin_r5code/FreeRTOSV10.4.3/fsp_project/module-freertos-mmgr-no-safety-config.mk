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
# Print makefile diagnostic message
ifeq ($(VERBOSE), 1)
$(info Including:  module-freertos-mmgr-no-safety-config.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
#   TOC:  Declare the include file directories needed by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_FREERTOS_MMGR_NO_SAFETY_CONFIG               := 1

#
# Location of source code files
MODULE_FREERTOS_MMGR_NO_SAFETY_SOURCE_DIR            = $(FREERTOS_MMGR_DIR)

#
# Common includes '-I <INCLUDEDIR>'
MODULE_FREERTOS_MMGR_NO_SAFETY_INCLUDES              =
MODULE_FREERTOS_MMGR_NO_SAFETY_INCLUDES              = -I $(FREERTOS_DIR)/include
