/*
 * Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef HSP__HSP_TEGRA_PRIV_H
#define HSP__HSP_TEGRA_PRIV_H
#define FSP__HSP__HSP_TEGRA_PRIV_H                      1

/**
 * @file hsp/hsp-tegra-priv.h
 * @brief Structures that are internal to the HSP driver
 */

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>        // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <hsp/hsp-tegra.h>

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__HSP__HSP_TEGRA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * @brief Maximum number of HSP shared mailboxes per instance
 *
 * Defines the maximum number of shared mailboxes that are available
 * per instance.
 */
#define MAX_TEGRA_HSP_SM  8U

/**
 * @brief HSP configuration structure
 *
 * Structure that describes the configuration of a HSP instance.
 *
 * @base_addr       base address of the HSP instance
 * @host_id         host identifier of the HSP instance
 * @db_irq          doorbell irq number, UINT32_MAX if none
 * @sh_irq          shared irq number, UINT32_MAX if none
 * @si_index        which shared IRQ is used, UINT8_MAX if none
 */
struct tegra_hsp_conf {
    uint32_t base_addr;
    uint32_t host;
    uint32_t db_irq;
    uint32_t sh_irq;
    uint8_t si_index;
};

/**
 * @brief HSP mailbox callback configuration structure
 *
 * Structure that describes the callback configuration of a shared
 * mailbox irq.
 *
 * @wide        32-bit or 128-bit
 * @callback    callback for the mailbox irq
 * @opaque      opaque data pointer for the callback
 */
struct hsp_sm {
    bool wide;
    tegra_hsp_sm_callback callback;
    void *opaque;
};

/**
 * @brief HSP context ID structure
 *
 * Structure that is used by the HSP driver to manage the HSP instance
 * during run time.
 *
 * @conf            HSP configuration of an instance
 * @db_callback     callback function on doorbell signal
 * @db_base         doorbell base address
 * @ss_base         shared semaphore base address
 * @sie_mask        shared interrupt enable mask
 * @sm              shared mailbox callback configuration array
 * @n_sm            number of shared mailboxes
 * @n_ss            number of shared semaphores
 * @n_as            number of arbitrated semaphores
 * @n_db            doorbell count
 * @n_si            shared interrupt count
 * @inited          initialization status
 */
struct tegra_hsp_id {
    struct tegra_hsp_conf const conf;
    tegra_hsp_db_callback db_callback;
    uint32_t db_base;
    uint32_t ss_base;
    uint32_t sie_mask;
    struct hsp_sm sm[MAX_TEGRA_HSP_SM];
    uint8_t n_sm;
    uint8_t n_ss;
    uint8_t n_as;
    uint8_t n_db;
    uint8_t n_si;
    bool    inited;
};

#endif
