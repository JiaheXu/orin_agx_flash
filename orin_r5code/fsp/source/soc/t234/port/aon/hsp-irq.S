/*
 * Copyright (c) 2019-2020 NVIDIA CORPORATION.  All rights reserved.
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
#include <hw-config/vic-irqs.h>
#include <processor/hsp-tegra-hw.h>
#include <vic/vic-asm.h>

/* Module-specific FSP headers */
#include <hsp/sections-hsp.h>

    .section    SECTION_STR_HSP_TEXT, "ax"
    .arm
    .syntax     unified

    /**
     * HSP interrupt handlers
     * TEGRA_HSP_CTX_MBOX/DB should be defined by the soc includes
     */
    isr_addr MBOX_isr, TEGRA_HSP_ID_MBOX, tegra_hsp_irq_handler, INTERRUPT_MBOX
    isr_addr TOP0_HSP_DB_isr, TEGRA_HSP_ID_DB, tegra_hsp_db_irq_handler, INTERRUPT_TOP0_HSP_DB

    /**
     * Defines the portion of the ISR map array for HSP
     */
    .section    SECTION_STR_VIC_ISRMAP_DATA, "a"

    isr_map_entry    INTERRUPT_MBOX, MBOX_isr
    isr_map_entry    INTERRUPT_TOP0_HSP_DB, TOP0_HSP_DB_isr

    .end
