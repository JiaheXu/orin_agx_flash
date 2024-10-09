#
# @file module-sha-nvriscv-config.mk
# 
# @brief SHA-NVRISCV config makefile for libFSP.
# 
# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of NVIDIA CORPORATION nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#
# Print makefile diagnostic message
ifeq ($(VERBOSE), 1)
$(info Including:  module-sha-config.mk)
endif

#
# Table of Contents:
#   TOC:  Declare the module dependencies and check them
#   TOC:  Declare the source directories, which automatically adds them to VPATH
#   TOC:  Declare the include file directories needed by this module

#
# Define a something similar to header guard protection...for makefiles instead.
#   This definition must exist for all 'module-soc_*.mk' files.
MODULE_SHA_NVRISCV_CONFIG               := 1

#
# Check for sub-make module dependencies.  For each of the module dependencies
#   defined below, they must exist in the build before this (current) sub-make
#   is called.
MODULE_SHA_NVRISCV_DEPENDS              := FSP_COMMON
MODULE_SHA_NVRISCV_DEPENDS              := SHA_NVRISCV
$(foreach _,$(MODULE_SHA_NVRISCV_DEPENDS),$(eval $(call CHECK_MAKEFILE_DEFINED,$_)))

#
# Location of source code files
MODULE_SHA_NVRISCV_DRIVERS_DIR           = $(FSP_COMMON_REPO)/drivers/sha-nvriscv
MODULE_SHA_NVRISCV_SOURCE_DIR            = $(MODULE_SHA_NVRISCV_DRIVERS_DIR)

#
# Common includes '-I <INCLUDEDIR>'
#MODULE_SHA_NVRISCV_INCLUDES              = $(FSP_COMMON_REPO)/include/sha
