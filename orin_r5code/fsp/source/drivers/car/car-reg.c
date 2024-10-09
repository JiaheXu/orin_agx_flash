/* Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
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
#include <stddef.h>                     // for NULL
#include <stdbool.h>                    // for bool, false
#include <stdint.h>                     // for uint32_t, uint8_t, UINT8_MAX, UIN...

/* Early FSP headers */
#include <misc/ct-assert.h>             // for CT_ASSERT
#include <soc-common/hw-const.h>        /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>            // for NV_ADDRESS_MAP_CAR_BASE

/* Late FSP headers */
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD...
#include <reg-access/reg-access.h>      // for readl_base_offset, writel_base_of...
#include <misc/bitops.h>                // for BIT, bit_number, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */
#include <car/car-sections.h>           // for SECTION_CAR_TEXT, ...
#include <car/car-reg.h>                // for register access declarations...

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
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CAR__CAR_REG_H, "Header file missing or invalid.")


static inline uint32_t car_reg(uint32_t offset)
{
    return (NV_ADDRESS_MAP_CAR_BASE + offset);
}

SECTION_CAR_TEXT
static uint32_t car_reg_msk2shft(uint32_t mask) {
    uint32_t shift;

    if (mask != 0U) {
        shift = bit_number(mask);
    } else {
        shift = 0;
    }
    return shift;
}

SECTION_CAR_TEXT
uint32_t car_reg_rd(uint32_t offset)
{
    return readl(car_reg(offset));
}

SECTION_CAR_TEXT
void car_reg_wr(uint32_t offset, uint32_t data)
{
    writel(data, car_reg(offset));
    return;
}

SECTION_CAR_TEXT
uint32_t car_reg_rdwr(uint32_t offset, uint32_t data, uint32_t mask) {
    uint32_t data_rdwr;

    data_rdwr = readl(car_reg(offset));
    data_rdwr &= ~mask;
    data_rdwr |= data;
    writel(data_rdwr, car_reg(offset));
    return data_rdwr;
}

SECTION_CAR_TEXT
uint32_t car_reg_rd_val(uint32_t offset, uint32_t mask) {
    uint32_t val;

    val = readl(car_reg(offset));
    val &= mask;
    val >>= car_reg_msk2shft(mask);
    return val;
}

SECTION_CAR_TEXT
uint32_t car_reg_rdwr_val(uint32_t offset, uint32_t mask, uint32_t val) {
    uint32_t shift;

    if (mask != 0) {
        shift = car_reg_msk2shft(mask);
        if (val > (mask >> shift)) {
            val = mask >> shift;
        }
    } else {
        shift = 0;
    }

    return car_reg_rdwr(offset, val << shift, mask);
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

