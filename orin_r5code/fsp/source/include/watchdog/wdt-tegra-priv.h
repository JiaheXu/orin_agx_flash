/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef WATCHDOG__WDT_TEGRA_PRIV_H
#define WATCHDOG__WDT_TEGRA_PRIV_H
#define FSP__WATCHDOG__WDT_TEGRA_PRIV_H                 1

/**
 * @file watchdog/wdt-tegra-priv.h
 * @brief Structures that are internal to the WDT driver
 */

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/attributes.h>               // for FSP_ALIGNOF
#include <misc/macros.h>                   // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <watchdog/tegra-safe-wdt.h>

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
                MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__WATCHDOG__TEGRA_SAFE_WDT_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief Watchdog ID Structure
 *
 * Structure the describes the implementation of a particular
 * watchdog timer instance.
 */
struct tegra_wdt_id
{
    const char      *devname;           ///! Name of watchdog timer instance
    uint32_t        base_addr;          ///! Base address of watchdog timer
    uint32_t        irq;                ///! IRQ vector number for watchdog timer
    uint32_t        fiq;                ///! FIQ vector number for watchdog timer
};

/**
 * @brief Watchdog Timer Structure
 *
 * Structure that is used by the watchdog driver to manage the watchdog
 * timer instance during run-time.
 */
struct tegra_wdt
{
    struct tegra_wdt_id         id;                     ///! Watchdog ID
    tegra_wdt_callback          irq_callback;           ///! function to call
                                                        ///!  on IRQ
    void                        *irq_data;              ///! data to supply to
                                                        ///!  irq_callback
    tegra_wdt_callback          fiq_callback;           ///! function to call
                                                        ///!  on FIQ
    void                        *fiq_data;              ///! data to supply to
                                                        ///!  fiq_callback
    bool                        irq_ack_by_callback;    ///! ack IRQ in
                                                        ///!  irq_callback, else
                                                        ///!  ack in tegra_wdt_irq
    bool                        fiq_ack_by_callback;    ///! ack FIQ in
                                                        ///!  irq_callback, else
                                                        ///!  ack in tegra_wdt_fiq
    bool                        challenge_response;     ///! updates to watchdog
                                                        ///!  timer requires a
                                                        ///!  challenge /
                                                        ///!  response
    uint8_t                     pad;
};

/**
 * Converts const struct tegra_wdt_id * to uint32_t.
 *
 * @param[in] input const struct tegra_wdt_id pointer which will be coverted to uint32_t
 * @param[out] output uint32_t from const struct tegra_wdt_id pointer
 */
static inline uint32_t fsp_c_wdtid_ptr_to_u32(const struct tegra_wdt_id *input)
{
CT_ASSERT(FSP_ALIGNOF(const struct tegra_wdt_id *) >= FSP_ALIGNOF(uint32_t), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const struct tegra_wdt_id *) == sizeof(uint32_t), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_wdtid_ptr_to_u32 {
        const struct tegra_wdt_id *input;
        uint32_t output;
    };
    union cast_fsp_c_wdtid_ptr_to_u32 t;

    t.input = input;
    return t.output;
}

/**
 * Converts uint32_t to const struct tegra_wdt *.
 *
 * @param[in] input uint32_t which will be converted to struct tegra_wdt pointer
 * @param[out] output const struct tegra_wdt pointer from uint32_t
 */
static inline const struct tegra_wdt *fsp_u32_to_c_wdt_ptr(uint32_t input)
{
CT_ASSERT(FSP_ALIGNOF(uint32_t) >= FSP_ALIGNOF(const struct tegra_wdt *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(uint32_t) == sizeof(const struct tegra_wdt *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_u32_to_c_wdt_ptr {
        uint32_t input;
        const struct tegra_wdt *output;
    };
    union cast_fsp_u32_to_c_wdt_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts const struct tegra_wdt * to struct tegra_wdt.
 *
 * @param[in] input const struct tegra_wdt which will be converted to struct tegra_wdt pointer
 * @param[out] output struct tegra_wdt pointer from const struct tegra_wdt
 */
static inline struct tegra_wdt *fsp_c_wdt_ptr_to_wdt_ptr(const struct tegra_wdt *input)
{
CT_ASSERT(FSP_ALIGNOF(const struct tegra_wdt *) >= FSP_ALIGNOF(struct tegra_wdt *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const struct tegra_wdt *) == sizeof(struct tegra_wdt *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_wdt_ptr_to_wdt_ptr {
        const struct tegra_wdt *input;
        struct tegra_wdt *output;
    };
    union cast_fsp_c_wdt_ptr_to_wdt_ptr t;

    t.input = input;
    return t.output;
}

/**
 * Converts const struct tegra_wdt_id * to void *.
 *
 * @param[in] input const struct tegra_wdt_id which will be converted to void pointer
 * @param[out] output void pointer from const struct tegra_wdt_id
 */
static inline void *fsp_c_wdtid_ptr_to_v_ptr(const struct tegra_wdt_id *input)
{
CT_ASSERT(FSP_ALIGNOF(const struct tegra_wdt_id *) >= FSP_ALIGNOF(void *), "ASSERT due to data misalignment.")
    CT_ASSERT(sizeof(const struct tegra_wdt_id *) == sizeof(void *), "ASSERT due to incompatible pointer sizes.")
    union cast_fsp_c_wdtid_ptr_to_v_ptr {
        const struct tegra_wdt_id *input;
        void *output;
    };
    union cast_fsp_c_wdtid_ptr_to_v_ptr t;

    t.input = input;
    return t.output;
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_19_2, "Approval: Bug 200543136, DR: SWE-FSP-033-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval:  On-file, DR: SWE-FSP-012-SWSADR.docx")
#endif  /* FSP_WATCHDOG_WDT_TEGRA_PRIV_H */
