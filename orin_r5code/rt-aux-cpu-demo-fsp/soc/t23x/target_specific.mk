# Copyright (c) 2021-2022, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

TARGET_INCLUDES := \
	$(SPE_TARGET_DIR)/include/ \
	$(SPE_TARGET_DIR)/config/ \
	$(FSP_SRC_DIR)/include/soc/t234/aon/ \
	$(FSP_SRC_DIR)/include/soc/t234/

TARGET_SRCS := \
	$(SPE_TARGET_DIR)/src/clk.c \
	$(SPE_TARGET_DIR)/src/init_padctrl.c \
	$(SPE_TARGET_DIR)/src/spe-clk.c \
	$(SPE_TARGET_DIR)/src/spe-pm-sequencer.c \
	$(SPE_TARGET_DIR)/src/late-init.c \
	$(SPE_TARGET_DIR)/src/relocate-dma.S \
	$(SPE_TARGET_DIR)/src/boot.S \
	$(SPE_TARGET_DIR)/src/lic-map.c \
	$(SPE_TARGET_DIR)/src/spe-lic.c \
	$(SPE_TARGET_DIR)/src/interrupt.c \
	$(SPE_TARGET_DIR)/src/tegra-lic.c \
	$(FSP_SRC_DIR)/drivers/lic/lic-tegra.c \
	$(SPE_TARGET_DIR)/src/uart-t234.c \
	$(SPE_TARGET_DIR)/src/tcu-ids.c \
	$(FSP_SRC_DIR)/drivers/hsp/hsp-tegra-sm-128.c  \
	$(FSP_SRC_DIR)/soc/t234/ids/aon/uart-tegra-ids.c \
	$(FSP_SRC_DIR)/soc/t234/ids/aon/hsp-tegra-ids.c \
	$(FSP_SRC_DIR)/soc/t234/ids/aon/tke-tegra-ids.c \
	$(FSP_SRC_DIR)/soc/t234/ids/aon/ast-tegra-ids.c \
	$(FSP_SRC_DIR)/soc/t234/ids/aon/irqapi-vic-ids.c \
	$(FSP_SRC_DIR)/soc/t234/ids/aon/lic-tegra-ids.c \
	$(FSP_SRC_DIR)/soc/t234/ids/soc-common/clk-tegra.c

# Enable = 1/Disable = 0 GPCDMA functionality
ENABLE_GPCDMA_FUNC := 0

# Enable = 1/Disable = 0 AODMIC sample app
ENABLE_AODMIC_APP := 0

ifeq ($(ENABLE_AODMIC_APP), 1)
	ENABLE_GPCDMA_FUNC := 1
	TARGET_SRCS += \
		$(FSP_SRC_DIR)/soc/t234/port/aon/aodmic-port.c \
		$(FSP_SRC_DIR)/soc/t234/ids/aon/aodmic-tegra-ids.c
endif

ifeq ($(ENABLE_GPCDMA_FUNC), 1)
	TARGET_SRCS += \
		$(FSP_SRC_DIR)/soc/t234/ids/aon/gpcdma-ids.c \
		$(FSP_SRC_DIR)/soc/t234/port/aon/gpcdma-port.c
endif

# Enable = 1/Disable = 0 timer sample app
ENABLE_TIMER_APP := 0

# Enable = 1/Disable = 0 GPIO sample app
ENABLE_GPIO_APP := 1

ifeq ($(ENABLE_GPIO_APP), 1)
	TARGET_SRCS += \
		$(FSP_SRC_DIR)/soc/t234/ids/aon/gpio-aon-tegra-ids.c \
		$(FSP_SRC_DIR)/soc/t234/ids/aon/gpio-main-tegra-ids.c \
		$(FSP_SRC_DIR)/soc/t234/ids/aon/gpio-tegra-ids.c
endif

# Enable = 1/Disable = 0 I2C sample app
ENABLE_I2C_APP := 0
ifeq ($(ENABLE_I2C_APP), 1)
	TARGET_SRCS += \
		$(FSP_SRC_DIR)/soc/t234/ids/aon/i2c-tegra-ids.c
endif

