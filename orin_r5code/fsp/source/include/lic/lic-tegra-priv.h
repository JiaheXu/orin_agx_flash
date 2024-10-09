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

#ifndef LIC__LIC_TEGRA_PRIV_H
#define LIC__LIC_TEGRA_PRIV_H
#define FSP__LIC__LIC_TEGRA_PRIV_H                      1

/**
 * @file lic/lic-tegra-priv.h
 * @brief Structures that are internal to the LIC driver
 */

/* Compiler headers */
#include <stdbool.h>                // for bool
#include <stdint.h>                 // for uint8_t, uint32_t, uint16_t

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>            // for END_RFD_BLOCK, START_RFD_BLOCK

/* Module-specific FSP headers */
#include <lic/lic-tegra.h>          // for FSP__LIC__LIC_TEGRA_H, lic_irq_t

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
START_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
    CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
    CT_ASSERT(FSP__LIC__LIC_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @file lic/lic-tegra-priv.h
 * @brief Structures that are internal to the LIC driver
 */

/**
 * @brief LIC irq handler context
 *
 * Structure that describes the parameter to the LIC irq handler.
 *
 * @id            pointer to LIC context id structure
 * @chan          channel that triggered the irq
 * @irqs_set      irqs enabled on this channel
 * @is_vfiq       if the channel is FIQ/IRQ
 */
typedef struct {
    struct tegra_lic_id *id;
    uint8_t             chan;
    uint32_t            irqs_set;
    bool                is_vfiq;
} lic_irq_context_t;


/**
 * @brief LIC configuration structure
 *
 * Structure that describes the configuration of the LIC.
 *
 * @base_chan       base channel for the current R5 cluster
 * @num_chans       number of LIC channels supported
 * @num_slices      number of slices supported per channel
 * @local_irq_base  base VIC interrupt number for the LIC interrupt lines
 * @lic_irq_base    start of the LIC interrupts for the cluster
 */
struct tegra_lic_conf {
    uint8_t         base_chan;
    uint8_t         num_chans;
    uint8_t         num_slices;
    uint16_t        local_irq_base;
    uint16_t        lic_irq_base;
};

/**
 * @brief LIC context ID structure
 *
 * Structure that is used by the LIC driver to manage the LIC state
 * during run time.
 *
 * @conf                LIC configuration for the R5 cluster
 * @lic_map             pointer to the LIC irq mapping for the cluster
 * @irq_contexts        Pointer to the irq contexts for the LIC interrupts to
 *                      R5 throught the local VIC
 * @lic_map_size        size of the LIC irq map for the cluster
 * @num_lic_irq_lines   Number of irq lines for LIC to the local VIC
 */
struct tegra_lic_id {
    const struct tegra_lic_conf conf;
    const lic_irq_t             *lic_map;
    lic_irq_context_t           **irq_contexts;
    uint32_t                    lic_map_size;
    const uint32_t              num_lic_irq_lines;
};

#endif
