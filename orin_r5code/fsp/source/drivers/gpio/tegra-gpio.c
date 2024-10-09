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

/* Compiler headers */
#include <stdint.h>                        //for uint32_t
#include <stddef.h>                        //for NULL
#include <stdbool.h>                       //for bool

/* Early FSP headers */
#include <misc/ct-assert.h>                // for CT_ASSERT
#include <soc-common/hw-const.h>           // for MK_U32_CONSTANT

/* Hardware headers */
#include <argpio_sw.h>                     // for GPIO_N_ENABLE_CONFIG_00_0...

/* Late FSP headers */
#include <reg-access/reg-access.h>         // for readl_base_offset, writel_base_offset
#include <irq/safe-irqs.h>                      // for irq_safe_enable, irq_safe_set_handler, ...
#include <misc/bitops.h>                   // for BIT, BIT32, bit_number
#include <misc/macros.h>                   // for START_RFD_BLOCk, END_RFD_BLOCK, INLINE_RFD
#include <error/common-errors.h>           // for E_SUCCESS, error_t
#include <misc/nvrm_drf.h>                 // for NV_FLD_SET_DRF_NUM, NV_DRF_NUM

/* Module headers */
#include <gpio/gpio-error.h>               // For E_GPIO_INVALID_GPIO_IRQ,E_GPIO_INVALID_IRQ_HANDLER
#include <gpio/sections-gpio.h>            // Immune from CT_ASSERT protection
#include <gpio/tegra-gpio.h>               // For tegra_gpio_direction_in.....
#include <gpio/tegra-gpio-priv.h>          // For struct tegra_gpio_id....

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPIO__GPIO_ERROR_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPIO__TEGRA_GPIO_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPIO__TEGRA_GPIO_PRIV_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define TEGRA_GPIO_IRQ_TYPES    0x4U
#define TEGRA_GPIO_IRQ_LEVELS   0x2U

/* Static global variables saving copy of GPIO SoC data and
number of controllers registered */
static struct tegra_gpio_id **ctrls;
static uint32_t nctrls;

/* Default mask for bank gpio IRQ */
#define TEGRA_GPIO_IRQ_MASK    0xFFUL

/**
 * @brief Extract a GPIO controller ID from a global GPIO ID.
 *
 * No error-range-checking is performed on the parameters or returned value.
 *
 * @param[in] global_id    The global ID of the GPIO.
 *
 * @retval ctrl_id The GPIO controller ID.
 */
static inline uint32_t gpio_ctrl_id_of_global_id(uint32_t global_id)
{
    return global_id >> 16;
}

/**
 * @brief Extract a controller-relative GPIO ID from a global GPIO ID.
 *
 * No error-range-checking is performed on the parameters or returned value.
 *
 * @param[in] global_id    The global ID of the GPIO.
 *
 * @retval gpio_id The controller-relative GPIO ID of the GPIO.
 */
static inline uint32_t gpio_gpio_id_of_global_id(uint32_t global_id)
{
    return global_id & 0xffffUL;
}

static inline bool gpio_valid(const struct tegra_gpio_id *id,
                              uint32_t gpio)
{
    uint32_t bank = gpio >> MK_U32_CONST(3);

    return bank < id->bank_count;
}

static inline uint32_t gpio_reg(const struct tegra_gpio_id *id,
                           uint32_t bank_gpio,
                           uint32_t gpio_regnum)
{
    uint32_t bank = bank_gpio / GPIOS_PER_BANK;
    uint32_t gpio = bank_gpio % GPIOS_PER_BANK;
    uint32_t bank_base = id->bank_bases[bank];

    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, Example-1, Example-3, DR: SWE-FSP-045-SWSADR.docx");
    return id->base_addr + bank_base + (gpio_regnum - GPIO_N_ENABLE_CONFIG_00_0) +
        (gpio * (GPIO_N_ENABLE_CONFIG_01_0 - GPIO_N_ENABLE_CONFIG_00_0));
}

SECTION_GPIO_TEXT
error_t tegra_gpio_enable_irq(uint32_t gpio)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   gpio_bank;
    uint32_t                   gpio_pin;
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }
    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    gpio_bank = gpio_id / GPIOS_PER_BANK;
    gpio_pin = gpio_id % GPIOS_PER_BANK;

    if (id->isr_status && ((id->irqs == NULL) || (id->nirqs == 0U))) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_IRQ;
        goto out;
    }

    if (id->isr_status && (id->irq_handlers[gpio_id].handler == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_IRQ_HANDLER;
        goto out;
    }

    id->bank_irq_status[gpio_bank] |= (uint8_t)((1UL << gpio_pin) & TEGRA_GPIO_IRQ_MASK);

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val |= NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, INTERRUPT_FUNCTION, ENABLE);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_enable_timestamp(uint32_t gpio)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }
    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val |= NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, TIMESTAMPING_FUNCTION, ENABLE);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_disable_timestamp(uint32_t gpio)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t     val;
    const struct tegra_gpio_id *id;
    error_t      err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val &= ~NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, TIMESTAMPING_FUNCTION, ENABLE);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_disable_irq(uint32_t gpio)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   gpio_bank;
    uint32_t                   gpio_pin;
    uint32_t     val;
    const struct tegra_gpio_id *id;
    error_t      err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    gpio_bank = gpio_id / GPIOS_PER_BANK;
    gpio_pin = gpio_id % GPIOS_PER_BANK;

    if (id->isr_status && ((id->irqs == NULL) || (id->nirqs == 0U))) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_IRQ;
        goto out;
    }

    if (id->isr_status && (id->irq_handlers[gpio_id].handler == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_IRQ_HANDLER;
        goto out;
    }


    id->bank_irq_status[gpio_bank] &= (uint8_t)((uint32_t)(~(1UL << gpio_pin))
                                                & TEGRA_GPIO_IRQ_MASK);

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val &= ~NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, INTERRUPT_FUNCTION, ENABLE);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_set_irq_type(uint32_t gpio,
                                tegra_gpio_irq_type type,
                                tegra_gpio_irq_level level)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t     val;
    const struct tegra_gpio_id *id;
    error_t      err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    if (level >= TEGRA_GPIO_IRQ_LEVELS) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_IRQ_LEVEL;
        goto out;
    }

     if(type >= TEGRA_GPIO_IRQ_TYPES) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_IRQ_TYPE;
        goto out;
     }
    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val = NV_FLD_SET_DRF_NUM(GPIO, N_ENABLE_CONFIG_00, TRIGGER_LEVEL,
                             level, val);
    val = NV_FLD_SET_DRF_NUM(GPIO, N_ENABLE_CONFIG_00, TRIGGER_TYPE,
                             type, val);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
static void tegra_gpio_clear_irq(const struct tegra_gpio_id *id,
                                 uint32_t gpio)
{
    uint32_t val;

    val = NV_DRF_DEF(GPIO, N_INTERRUPT_CLEAR_00, GPIO_INTERRUPT_CLEAR, CLEAR);
    writel(val, gpio_reg(id, gpio, GPIO_N_INTERRUPT_CLEAR_00_0));
}

SECTION_GPIO_TEXT
error_t tegra_gpio_clear_irq_status(uint32_t gpio)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    const struct tegra_gpio_id *id;
    error_t      err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    tegra_gpio_clear_irq(id, gpio_id);

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_read_irq_status(uint32_t gpio, bool *irq_status)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   gpio_pin = gpio_id % GPIOS_PER_BANK;
    uint32_t                   gpio_bank = gpio_id / GPIOS_PER_BANK;
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t      err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    INLINE_RFD(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx", CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    val = readl(id->base_addr + id->bank_bases[gpio_bank] + id->irq_status_offset);
    *irq_status = ((val & (1UL << gpio_pin)) != 0U);

out:
    return err;
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_GPIO_TEXT void tegra_gpio_irq_handler(void *data)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    const struct tegra_gpio_id *id = (struct tegra_gpio_id const *)data;
    uint32_t             bank;
    uint32_t             gpio;
    uint32_t             val;

    if (id == NULL) {
        goto irq_exit;
    }

    for (bank = 0; bank < id->bank_count; bank++) {
        if (id->bank_irq_status[bank] == 0U) {
               continue;
        }
        INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, Example-1, Example-3, DR: SWE-FSP-045-SWSADR.docx");
        val = readl(id->base_addr + id->bank_bases[bank] + id->irq_status_offset);
        while (val != 0UL) {
            gpio = bit_number(val);
            val &= ~BIT32_FN(gpio);
            INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, Example-3, DR: SWE-FSP-045-SWSADR.docx");
            gpio += bank * GPIOS_PER_BANK;
            tegra_gpio_clear_irq(id, gpio);
            if (id->irq_handlers[gpio].handler != NULL) {
                id->irq_handlers[gpio].handler(
                        id->irq_handlers[gpio].data);
            }
        }
    }

irq_exit:
    return;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_set_irq_handler(uint32_t gpio,
                                          void (*handler)(void *hdata),
                                          void *data)
{
    uint32_t             ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t             gpio_id = gpio_gpio_id_of_global_id(gpio);
    const struct tegra_gpio_id *id;
    error_t              err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    if ((id->irqs == NULL) || (id->nirqs == 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_IRQ;
        goto out;
    }

    if (handler == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_IRQ_HANDLER;
        goto out;
    }
    enter_critical();
    id->irq_handlers[gpio_id].handler = handler;
    id->irq_handlers[gpio_id].data = data;
    exit_critical();

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_clear_irq_handler(uint32_t gpio)
{
    uint32_t             ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t             gpio_id = gpio_gpio_id_of_global_id(gpio);
    const struct tegra_gpio_id *id;
    uint32_t             val;
    uint32_t             irq_en;
    error_t              err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    if ((id->irqs == NULL) || (id->nirqs == 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_IRQ;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    irq_en = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, INTERRUPT_FUNCTION, val);
    if (irq_en != 0UL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_IRQ;
        goto out;
    }
    enter_critical();
    id->irq_handlers[gpio_id].handler = NULL;
    id->irq_handlers[gpio_id].data = NULL;
    exit_critical();

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_set_debounce(uint32_t gpio,
                                uint32_t debounce_ms)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val = 0UL;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    if (debounce_ms == 0UL) {
        val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
        val &= ~NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, DEBOUNCE_FUNCTION, ENABLE);
        writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
        goto out;
    }

    val |= NV_DRF_NUM(GPIO, N_DEBOUNCE_THRESHOLD_00, DEBOUNCE_THRESHOLD, debounce_ms);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_DEBOUNCE_THRESHOLD_00_0));
    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val |= NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, DEBOUNCE_FUNCTION, ENABLE);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_direction_in(uint32_t gpio)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val |=
        NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, GPIO_ENABLE, ENABLE) |
        NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, IN_OUT, IN);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_direction_out(uint32_t gpio,
                                 bool value)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = 1;//E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = 2;//E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = 3;//E_GPIO_INVALID_GPIO;
        goto out;
    }

    val =
        NV_DRF_NUM(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, value ? 1UL : 0UL);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_OUTPUT_VALUE_00_0));

    val =
        NV_DRF_DEF(GPIO, N_OUTPUT_CONTROL_00, GPIO_OUT_CONTROL, DRIVEN);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_OUTPUT_CONTROL_00_0));

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    val |=
        NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, GPIO_ENABLE, ENABLE) |
        NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, IN_OUT, OUT);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_get_direction(uint32_t gpio, uint32_t *gpio_dir)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    *gpio_dir = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, IN_OUT, val);

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_get_output_value(uint32_t gpio,
                                    bool *gpio_val)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    uint32_t                   gpio_cfg;
    const struct tegra_gpio_id *id;
    error_t                    ret = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    gpio_cfg = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, IN_OUT, val);
    if (gpio_cfg == 0UL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_GPIO;
    }
    val = readl(gpio_reg(id, gpio_id, GPIO_N_OUTPUT_VALUE_00_0));

    *gpio_val = NV_DRF_VAL(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, val) != 0U;

out:
    return ret;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_get_input_value(uint32_t gpio,
                                   bool     *gpio_val)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    ret = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_INPUT_00_0));

    *gpio_val = NV_DRF_VAL(GPIO, N_INPUT_00, GPIO_IN, val) != 0U;

out:
    return ret;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_get_value(uint32_t gpio,
                             bool *gpio_val)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    uint32_t                   gpio_cfg;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val = readl(gpio_reg(id, gpio_id, GPIO_N_ENABLE_CONFIG_00_0));
    gpio_cfg = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, IN_OUT, val);
    if (gpio_cfg == 0UL) {
        val = readl(gpio_reg(id, gpio_id, GPIO_N_INPUT_00_0));
        *gpio_val = NV_DRF_VAL(GPIO, N_INPUT_00, GPIO_IN, val) != 0U;
    } else {
        val = readl(gpio_reg(id, gpio_id, GPIO_N_OUTPUT_VALUE_00_0));
        *gpio_val = NV_DRF_VAL(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, val) != 0U;
    }

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_set_value(uint32_t gpio,
                             bool value)
{
    uint32_t                   ctrl_id = gpio_ctrl_id_of_global_id(gpio);
    uint32_t                   gpio_id = gpio_gpio_id_of_global_id(gpio);
    uint32_t                   val;
    const struct tegra_gpio_id *id;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!gpio_valid(id, gpio_id)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO;
        goto out;
    }

    val =
        NV_DRF_NUM(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, value ? 1UL : 0UL);
    writel(val, gpio_reg(id, gpio_id, GPIO_N_OUTPUT_VALUE_00_0));

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_suspend(uint32_t ctrl_id)
{
    struct tegra_gpio_id       const *id;
    uint32_t                   i;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];
    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if ((id->irqs == NULL) || (id->nirqs == 0U)) {
        goto out;
    }

    for (i = 0UL; i < id->nirqs; i++) {
        if (id->irqs[i] == UINT32_MAX) {
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            err = E_GPIO_INVALID_GPIO_IRQ;
        }
        if (err == E_SUCCESS) {
            err = irq_safe_disable(id->irqs[i]);
        }
        if (err != E_SUCCESS) {
            break;
        }
    }

out:
    return err;
}

SECTION_GPIO_TEXT
error_t tegra_gpio_resume(uint32_t ctrl_id)
{
    struct tegra_gpio_id       const *id;
    uint32_t                   i;
    error_t                    err = E_SUCCESS;

    if (ctrl_id >= nctrls) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_GPIO_CTRL_ID;
        goto out;
    }

    id = ctrls[ctrl_id];
    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if ((id->irqs == NULL) || (id->nirqs == 0U)) {
        goto out;
    }

    for (i = 0UL; i < id->nirqs; i++) {
        if (id->irqs[i] == UINT32_MAX) {
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            err = E_GPIO_INVALID_GPIO_IRQ;
        }
        if (err == E_SUCCESS) {
            err = irq_safe_disable(id->irqs[i]);
        }
        if (err != E_SUCCESS) {
            break;
        }
    }

out:
    return err;
}

SECTION_GPIO_INIT_TEXT
static error_t tegra_gpio_do_init(struct tegra_gpio_id *id)
{
    uint32_t             i;
    error_t              err = E_SUCCESS;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    if (!id->isr_status) {
        goto out;
    }

    if ((id->irqs == NULL) || (id->nirqs == 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    for (i = 0UL; i < id->nirqs; i++) {
        if (id->irqs[i] == UINT32_MAX) {
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            err = E_GPIO_INVALID_GPIO_IRQ;
        }
        if (err == E_SUCCESS) {
            err = irq_safe_set_handler(id->irqs[i], tegra_gpio_irq_handler, id);
        }
        if (err == E_SUCCESS) {
            err = irq_safe_enable(id->irqs[i]);
        }
        if (err != E_SUCCESS) {
            break;
        }
    }

out:
    return err;
}

SECTION_GPIO_INIT_TEXT
error_t tegra_gpio_init(struct tegra_gpio_id **ids, uint32_t num_ctrls)
{
    uint32_t             i;
    error_t              err = E_SUCCESS;

    if (ids == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_NULL_INPUT_PARAMETER;
        goto out;
    }

    if (nctrls > MAX_GPIO_CONTROLLERS) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_GPIO_INVALID_CTRL_DATA;
        goto out;
    }

    ctrls = ids;
    nctrls = num_ctrls;

    for (i = 0U; i < nctrls; i++) {
        err = tegra_gpio_do_init(ctrls[i]);
        if (err != E_SUCCESS) {
            break;
        }
    }

out:
    return err;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
