/*
 * Copyright (c) 2019-2021 NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */
#include <hw-config/vic-irqs.h>        // defined by the application
#include <vic/vic-asm.h>

/* Module-specific FSP header files */
#include <soc-common/i2c-defs.h>
#include <i2c/sections-i2c.h>

    .section    SECTION_STR_I2C_TEXT, "ax"
    .arm
    .syntax     unified

    /*
     * I2C interrupt handlers
     */
    isr_value I2C_CTRL2_isr, TEGRA194_I2C2, tegra_i2c_irq_handler, \
                INTERRUPT_I2C2
    isr_value I2C_CTRL8_isr, TEGRA194_I2C8, tegra_i2c_irq_handler, \
                INTERRUPT_I2C3

    /*
     * Defines the portion of the ISR map array for I2C
     */
    .section    SECTION_STR_VIC_ISRMAP_DATA, "a"

    isr_map_entry    INTERRUPT_I2C2, I2C_CTRL2_isr
    isr_map_entry    INTERRUPT_I2C3, I2C_CTRL8_isr

    .end
