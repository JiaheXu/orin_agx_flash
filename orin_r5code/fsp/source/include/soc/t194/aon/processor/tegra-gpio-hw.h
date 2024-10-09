/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PROCESSOR__GPIO_TEGRA_HW_H
#define PROCESSOR__GPIO_TEGRA_HW_H
#define FSP__PROCESSOR__GPIO_TEGRA_HW_H                  1

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Late FSP headers */
#include <soc-common/hw-const.h>        // for MK_U32_CONST

/* Module-specific FSP headers */
#include <gpio/tegra-gpio-priv.h>       // for struct tegra_gpio_id


/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */
CT_ASSERT(FSP__GPIO__TEGRA_GPIO_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")

/* GPIOs implemented by main GPIO controller */
#define TEGRA_GPIO_BANK_ID_A    MK_U32_CONST(0)
#define TEGRA_GPIO_BANK_ID_B    MK_U32_CONST(1)
#define TEGRA_GPIO_BANK_ID_C    MK_U32_CONST(2)
#define TEGRA_GPIO_BANK_ID_D    MK_U32_CONST(3)
#define TEGRA_GPIO_BANK_ID_E    MK_U32_CONST(4)
#define TEGRA_GPIO_BANK_ID_F    MK_U32_CONST(5)
#define TEGRA_GPIO_BANK_ID_G    MK_U32_CONST(6)
#define TEGRA_GPIO_BANK_ID_H    MK_U32_CONST(7)
#define TEGRA_GPIO_BANK_ID_I    MK_U32_CONST(8)
#define TEGRA_GPIO_BANK_ID_J    MK_U32_CONST(9)
#define TEGRA_GPIO_BANK_ID_K    MK_U32_CONST(10)
#define TEGRA_GPIO_BANK_ID_L    MK_U32_CONST(11)
#define TEGRA_GPIO_BANK_ID_M    MK_U32_CONST(12)
#define TEGRA_GPIO_BANK_ID_N    MK_U32_CONST(13)
#define TEGRA_GPIO_BANK_ID_O    MK_U32_CONST(14)
#define TEGRA_GPIO_BANK_ID_P    MK_U32_CONST(15)
#define TEGRA_GPIO_BANK_ID_Q    MK_U32_CONST(16)
#define TEGRA_GPIO_BANK_ID_R    MK_U32_CONST(17)
#define TEGRA_GPIO_BANK_ID_S    MK_U32_CONST(18)
#define TEGRA_GPIO_BANK_ID_T    MK_U32_CONST(19)
#define TEGRA_GPIO_BANK_ID_U    MK_U32_CONST(20)
#define TEGRA_GPIO_BANK_ID_V    MK_U32_CONST(21)
#define TEGRA_GPIO_BANK_ID_W    MK_U32_CONST(22)
#define TEGRA_GPIO_BANK_ID_X    MK_U32_CONST(23)
#define TEGRA_GPIO_BANK_ID_Y    MK_U32_CONST(24)
#define TEGRA_GPIO_BANK_ID_Z    MK_U32_CONST(25)
#define TEGRA_GPIO_BANK_ID_FF   MK_U32_CONST(26)
#define TEGRA_GPIO_BANK_ID_GG   MK_U32_CONST(27)

/* GPIOs implemented by AON GPIO controller */
#define TEGRA_GPIO_BANK_ID_AA   MK_U32_CONST(0)
#define TEGRA_GPIO_BANK_ID_BB   MK_U32_CONST(1)
#define TEGRA_GPIO_BANK_ID_CC   MK_U32_CONST(2)
#define TEGRA_GPIO_BANK_ID_DD   MK_U32_CONST(3)
#define TEGRA_GPIO_BANK_ID_EE   MK_U32_CONST(4)

#define TEGRA_GPIO(bank, offset) ((TEGRA_GPIO_BANK_ID_##bank * 8) + offset)

#endif
