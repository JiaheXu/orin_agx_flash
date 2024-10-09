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
$(info Including:  module-lib-print-nosafety-config.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
#   TOC:  Declare the include file directories needed by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_LIB_PRINT_NOSAFETY_CONFIG        := 1

#
# Location of source code files
LIB_PRINT_DIR                            = $(FSP_COMMON_REPO)/drivers/debug
MODULE_LIB_PRINT_NOSAFETY_SOURCE_DIR     = $(LIB_PRINT_DIR)

#
# Common includes '-I <INCLUDEDIR>'
MODULE_LIB_PRINT_NOSAFETY_INCLUDES       =
