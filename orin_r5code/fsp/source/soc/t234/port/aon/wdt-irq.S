/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <hw-config/vic-irqs.h>		// defined by the application, defines the interrupt map
#include <sections/sections.h>
#include <vic/vic-asm.h>

/* Module-specific FSP headers */
#include <watchdog/sections-wdt.h>

	.section	SECTION_STR_WDT_TEXT, "ax"
	.arm
	.syntax		unified

	/*
	 * Watchdog interrupt handlers
	 */
	isr_addr WDTFIQ_isr, tegra_wdt_instance, tegra_safe_wdt_fiq, INTERRUPT_WDTFIQ
	isr_addr WDTIRQ_isr, tegra_wdt_instance, tegra_safe_wdt_irq, INTERRUPT_WDTIRQ

	/*
	 * Defines the portion of the ISR map array for watchdog
	 */
	.section	SECTION_STR_VIC_ISRMAP_DATA, "a"

	isr_map_entry	INTERRUPT_WDTFIQ, WDTFIQ_isr
	isr_map_entry	INTERRUPT_WDTIRQ, WDTIRQ_isr

	.end
