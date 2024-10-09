/*
 * Copyright (c) 2017-2019 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef PROCESSOR__IRQS_LIC_H
#define PROCESSOR__IRQS_LIC_H
#define FSP__PROCESSOR__IRQS_LIC_H                      1

#define NR_VIC_IRQS 64UL

#define AON_LIC_IRQ_BASE     NR_VIC_IRQS
#define AON_LIC_IRQ_TOP_HSP1 (NR_VIC_IRQS)
#define AON_LIC_IRQ_SPI1     (NR_VIC_IRQS + 1UL)
#define AON_LIC_IRQ_SPI3     (NR_VIC_IRQS + 2UL)
#define AON_LIC_IRQ_GPIO0    (NR_VIC_IRQS + 3UL)
#define AON_LIC_IRQ_GPIO1    (NR_VIC_IRQS + 4UL)
#define AON_LIC_IRQ_GPIO2    (NR_VIC_IRQS + 5UL)
#define AON_LIC_IRQ_GPIO3    (NR_VIC_IRQS + 6UL)
#define AON_LIC_IRQ_GPIO4    (NR_VIC_IRQS + 7UL)
#define AON_LIC_IRQ_GPIO5    (NR_VIC_IRQS + 8UL)
#define AON_LIC_IRQ_MAX      (NR_VIC_IRQS + 9UL)

#define LIC0_IRQ_CONTEXT    lic0_irq_context
#define LIC1_IRQ_CONTEXT    lic1_irq_context
#define LIC2_IRQ_CONTEXT    lic2_irq_context
#define LIC3_IRQ_CONTEXT    lic3_irq_context

#ifndef __ASSEMBLER__

#include <lic/lic-tegra-priv.h>

extern struct tegra_lic_id tegra_lic_id_aon;
extern lic_irq_context_t lic0_irq_context;
extern lic_irq_context_t lic1_irq_context;
extern lic_irq_context_t lic2_irq_context;
extern lic_irq_context_t lic3_irq_context;

#endif

#endif /* PROCESSOR__IRQS_LIC_H */
