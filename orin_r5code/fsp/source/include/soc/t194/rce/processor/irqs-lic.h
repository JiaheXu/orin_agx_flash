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
#include <irq/tegra-lic-priv.h>

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
CT_ASSERT(FSP__IRQ__TEGRA_LIC_PRIV_H, "Header file missing or invalid.")

#define NR_VIC_IRQS 64

enum {
	RCE_LIC_IRQ_BASE = NR_VIC_IRQS,
	RCE_LIC_IRQ_ISP = LIC_IRQ_BASE,
	RCE_LIC_IRQ_NVCSI,
	RCE_LIC_IRQ_HOST1X_CAM0,
	RCE_LIC_IRQ_HOST1X_CAM1,
	RCE_LIC_IRQ_HOST1X_VM1,
	RCE_LIC_IRQ_HOST1X_VM2,
	RCE_LIC_IRQ_HOST1X_VM3,
	RCE_LIC_IRQ_HOST1X_VM4,
	RCE_LIC_IRQ_HOST1X_VM5,
	RCE_LIC_IRQ_HOST1X_VM6,
	RCE_LIC_IRQ_HOST1X_VM7,
	RCE_LIC_IRQ_HOST1X_VM8,
	RCE_LIC_IRQ_UARTA,
	RCE_LIC_IRQ_UARTG,
	RCE_LIC_IRQ_I2C0,
	RCE_LIC_IRQ_I2C2,
	RCE_LIC_IRQ_I2C4,
	RCE_LIC_IRQ_I2C5,
	RCE_LIC_IRQ_I2C6,
	RCE_LIC_IRQ_I2C7,
	RCE_LIC_IRQ_I2C9,
	RCE_LIC_IRQ_I2C10,
	RCE_LIC_IRQ_VI_VM0,
	RCE_LIC_IRQ_VI_VM1,
	RCE_LIC_IRQ_VI_VM2,
	RCE_LIC_IRQ_VI_VM3,
	RCE_LIC_IRQ_VI_VM4,
	RCE_LIC_IRQ_VI_VM5,
	RCE_LIC_IRQ_VI_VM6,
	RCE_LIC_IRQ_VI_VM7,
	RCE_LIC_IRQ_VI_VM8,
	RCE_LIC_IRQ_RCE_HSP1,
	RCE_LIC_IRQ_RCE_HSP2,
	RCE_LIC_IRQ_RCE_HSP3,
	RCE_LIC_IRQ_RCE_HSP4,
};

extern const struct tegra_lic_id tegra_lic_id_rce;

#endif /* PROCESSOR__IRQS_LIC_H */
