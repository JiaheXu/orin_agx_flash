/* Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CAR__CAR_REG_H
#define CAR__CAR_REG_H
#define FSP__CAR__CAR_REG_H             1

/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

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

/**
 * @brief Read a CAR MMIO register.
 *
 * @param offset The register offset from the CAR MMIO base address.
 * @return Value read from register.
 */
uint32_t car_reg_rd(uint32_t offset);

/**
 * @brief Write a value to a CAR MMIO register.
 *
 * @param offset The register offset from the CAR MMIO base
 *               address.
 * @param data The value to write to the register.
 * @return Void.
 */
void car_reg_wr(uint32_t offset, uint32_t data);

/**
 * @brief Read, modify, write to a CAR MMIO register.
 *
 * @param offset The register offset from the CAR MMIO base
 *               address.
 * @param data The data in the mask field to write.
 * @param mask The mask of the field(s) to modify and write.
 * @return The value written to register.
 */
uint32_t car_reg_rdwr(uint32_t offset, uint32_t data, uint32_t mask);

/**
 * @brief Read a value from a CAR MMIO register field.
 *
 * The purpose of the function is to read the value from a
 * register field regardless of the fields bit alignment with
 * the register.
 * For example:
 * Register data  = 0x12345678
 * Mask           = 0x000FF000
 * Value returned = 0x00000045
 *
 * @param offset The register offset from the CAR MMIO base
 *               address.
 * @param mask The mask of the data field to read.
 * @return The value read from register field shifted to bit
 *         zero.
 */
uint32_t car_reg_rd_val(uint32_t offset, uint32_t mask);

/**
 * @brief Write a value to a CAR MMIO register field.
 *
 * The purpose of the function is to write a value to a register
 * field regardless of the fields bit alignment with the
 * register.
 * For example:
 * Register data = 0x00000000
 * Mask          = 0x000FF000
 * Value         = 0x00000045
 * Register data = 0x00045000
 *
 * @param offset The register offset from the CAR MMIO base
 *               address.
 * @param mask The mask of the data field to write.
 * @param val The value to write to the field.
 * @return The register data written to the register.
 */
uint32_t car_reg_rdwr_val(uint32_t offset, uint32_t mask, uint32_t val);

#endif /* CAR__CAR_REG_H */

