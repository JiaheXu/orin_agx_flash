/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stdint.h>                 // for uint32_t

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before hwinc files */

/* Hardware headers */
#include <araps_vic.h>              // for APS_VIC_VICVECTADDR0_0, APS_VIC_V...

/* Late FSP headers */
#include <cpu/type-conversion.h>    // for fsp_v_fn_ptr_to_uptr, FSP__MISC__T...
#include <debug/assert.h>           // for ASSERT, FSP__DEBUG__ASSERT_H
#include <misc/bitops.h>            // for BIT, FSP__MISC__BITOPS_H
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <reg-access/reg-access.h>  // for writel_base_offset, readl_base_of...

/* Module-specific FSP headers */
#include <cpu/arm-vic.h>            // for arm_vic_context, ARM_VIC_IRQ_COUNT

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
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__ARM_VIC_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

void arm_vic_enable(uint32_t vic_base, uint32_t irq)
{
        writel_base_offset(BIT32_FN(irq), vic_base, APS_VIC_VICINTENABLE_0);
}

void arm_vic_disable(uint32_t vic_base, uint32_t irq)
{
        writel_base_offset(BIT32_FN(irq), vic_base, APS_VIC_VICINTENCLEAR_0);
}

void arm_vic_disable_all(uint32_t vic_base)
{
        writel_base_offset(~0U, vic_base, APS_VIC_VICINTENCLEAR_0);
}

void arm_vic_gen_software_int(uint32_t vic_base, uint32_t irq)
{
        writel_base_offset(BIT32_FN(irq), vic_base, APS_VIC_VICSOFTINT_0);
}

void arm_vic_clear_software_int(uint32_t vic_base, uint32_t irq)
{
        writel_base_offset(BIT32_FN(irq), vic_base, APS_VIC_VICSOFTINTCLEAR_0);
}

void arm_vic_set_isr_vect_addr(uint32_t vic_base, uint32_t irq,
                                void (*isr_vect_addr)(void))
{
        ASSERT(irq < MAX_ARM_VIC_IRQS);

        writel_base_offset((uint32_t)fsp_v_fn_ptr_to_uptr(isr_vect_addr), vic_base,
                (uint32_t)APS_VIC_VICVECTADDR0_0 +
                (uint32_t)(irq * (APS_VIC_VICVECTADDR1_0 - APS_VIC_VICVECTADDR0_0)));
}

uint32_t arm_vic_read_irq_status(uint32_t vic_base)
{
        return readl_base_offset(vic_base, APS_VIC_VICIRQSTATUS_0);
}

uint32_t arm_vic_read_fiq_status(uint32_t vic_base)
{
        return readl_base_offset(vic_base, APS_VIC_VICFIQSTATUS_0);
}

uint32_t arm_vic_read_raw_int_status(uint32_t vic_base)
{
        return readl_base_offset(vic_base, APS_VIC_VICRAWINTR_0);
}

void arm_vic_write_intselect(uint32_t vic_base, uint32_t intselect)
{
        writel_base_offset(intselect, vic_base, APS_VIC_VICINTSELECT_0);
}

uint32_t arm_vic_read_intselect(uint32_t vic_base)
{
        return readl_base_offset(vic_base, APS_VIC_VICINTSELECT_0);
}

void arm_vic_write_intenable(uint32_t vic_base, uint32_t intenable)
{
        writel_base_offset(intenable, vic_base, APS_VIC_VICINTENABLE_0);
}

uint32_t arm_vic_read_intenable(uint32_t vic_base)
{
        return readl_base_offset(vic_base, APS_VIC_VICINTENABLE_0);
}

void arm_vic_save_state(uint32_t vic_base, struct arm_vic_context *ctx)
{
        uint32_t i;

        for (i = 0; i < ARM_VIC_IRQ_COUNT; i++) {
                ctx->vect_addr[i] = readl_base_offset(vic_base, (uint32_t)APS_VIC_VICVECTADDR0_0 +
                        (i * (uint32_t)(APS_VIC_VICVECTADDR1_0 - APS_VIC_VICVECTADDR0_0)));
        }
        ctx->intenable = readl_base_offset(vic_base, APS_VIC_VICINTENABLE_0);
        ctx->intselect = readl_base_offset(vic_base, APS_VIC_VICINTSELECT_0);
}

void arm_vic_restore_state(uint32_t vic_base, const struct arm_vic_context *ctx)
{
        uint32_t i;

        for (i = 0; i < ARM_VIC_IRQ_COUNT; i++) {
                writel_base_offset(ctx->vect_addr[i], vic_base, (uint32_t)APS_VIC_VICVECTADDR0_0 +
                        (i * (uint32_t)(APS_VIC_VICVECTADDR1_0 - APS_VIC_VICVECTADDR0_0)));
        }
        writel_base_offset(~(ctx->intenable), vic_base, APS_VIC_VICINTENCLEAR_0);
        writel_base_offset(ctx->intenable, vic_base, APS_VIC_VICINTENABLE_0);
        writel_base_offset(ctx->intselect, vic_base, APS_VIC_VICINTSELECT_0);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
