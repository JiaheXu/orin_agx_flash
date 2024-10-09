/*
 * Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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

#include <sections/sections.h>
#include <vic/vic-asm.h>
#include <hw-config/vic-irqs.h>
#include <processor/gpcdma-hw.h>
#include <gpcdma/sections-gpcdma.h>

    .section    SECTION_STR_GPCDMA_TEXT, "ax"
    .arm
    .syntax     unified

    /**
     * GPCDMA per channel interrupt handlers
     * GPCDMA_CHAN[0-7] should be defined by the soc includes
     */
    isr_addr DMA0_isr, GPCDMA_CHAN0, gpcdma_chan_irq, INTERRUPT_DMA0
    isr_addr DMA1_isr, GPCDMA_CHAN1, gpcdma_chan_irq, INTERRUPT_DMA1
    isr_addr DMA2_isr, GPCDMA_CHAN2, gpcdma_chan_irq, INTERRUPT_DMA2
    isr_addr DMA3_isr, GPCDMA_CHAN3, gpcdma_chan_irq, INTERRUPT_DMA3
    isr_addr DMA4_isr, GPCDMA_CHAN4, gpcdma_chan_irq, INTERRUPT_DMA4
    isr_addr DMA5_isr, GPCDMA_CHAN5, gpcdma_chan_irq, INTERRUPT_DMA5
    isr_addr DMA6_isr, GPCDMA_CHAN6, gpcdma_chan_irq, INTERRUPT_DMA6
    isr_addr DMA7_isr, GPCDMA_CHAN7, gpcdma_chan_irq, INTERRUPT_DMA7

    /**
     * Defines the portion of the ISR map array for GPCDMA
     */
    .section    SECTION_STR_VIC_ISRMAP_DATA, "a"

    isr_map_entry    INTERRUPT_DMA0, DMA0_isr
    isr_map_entry    INTERRUPT_DMA1, DMA1_isr
    isr_map_entry    INTERRUPT_DMA2, DMA2_isr
    isr_map_entry    INTERRUPT_DMA3, DMA3_isr
    isr_map_entry    INTERRUPT_DMA4, DMA4_isr
    isr_map_entry    INTERRUPT_DMA5, DMA5_isr
    isr_map_entry    INTERRUPT_DMA6, DMA6_isr
    isr_map_entry    INTERRUPT_DMA7, DMA7_isr

    .end
