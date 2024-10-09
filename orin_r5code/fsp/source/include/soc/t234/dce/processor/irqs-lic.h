/*
 * Copyright (c) 2019-2020 NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef PROCESSOR__IRQS_LIC_H
#define PROCESSOR__IRQS_LIC_H
#define FSP__PROCESSOR__IRQS_LIC_H                      1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

#define NR_VIC_IRQS 64

#define LIC_IRQ_BASE        NR_VIC_IRQS
#define LIC0_IRQ_CONTEXT    lic0_irq_context
#define LIC1_IRQ_CONTEXT    lic1_irq_context
#define LIC2_IRQ_CONTEXT    lic2_irq_context
#define LIC3_IRQ_CONTEXT    lic3_irq_context

#ifndef __ASSEMBLER__

#include <lic/lic-tegra-priv.h>

extern struct tegra_lic_id tegra_lic_id_dce;
extern lic_irq_context_t lic0_irq_context;
extern lic_irq_context_t lic1_irq_context;
extern lic_irq_context_t lic2_irq_context;
extern lic_irq_context_t lic3_irq_context;

#endif

#endif /* PROCESSOR__IRQS_LIC_H */
