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
#ifndef PROCESSOR__GPIO_TEGRA_HW_PARAMS_H
#define PROCESSOR__GPIO_TEGRA_HW_PARAMS_H
#define FSP__PROCESSOR__GPIO_TEGRA_HW_PARAMS_H          1

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT

/* Late FSP headers */
#include <processor/irqs-hw.h>          // for NV_AON_INTERRUPT_VIC1_BASE, ...
#include <processor/irqs-lic.h>         // for AON_LIC_IRQ_GPIO0

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
CT_ASSERT(FSP__PROCESSOR__IRQS_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__IRQS_LIC_H, "Header file missing or invalid.")

#define TEGRA_MAIN_GPIO_ID0_IRQ         (AON_LIC_IRQ_GPIO0)
#define TEGRA_MAIN_GPIO_ID1_IRQ         (AON_LIC_IRQ_GPIO1)
#define TEGRA_MAIN_GPIO_ID2_IRQ         (AON_LIC_IRQ_GPIO2)
#define TEGRA_MAIN_GPIO_ID3_IRQ         (AON_LIC_IRQ_GPIO3)
#define TEGRA_MAIN_GPIO_ID4_IRQ         (AON_LIC_IRQ_GPIO4)
#define TEGRA_MAIN_GPIO_ID5_IRQ         (AON_LIC_IRQ_GPIO5)
#define TEGRA_AON_GPIO_IRQ              (NV_AON_INTERRUPT_GPIO + \
                                         NV_AON_INTERRUPT_VIC1_BASE)
#endif
