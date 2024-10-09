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

#ifndef AST__TEGRA_AST_H
#define AST__TEGRA_AST_H
#define FSP__AST__TEGRA_AST_H                           1

/* Compiler headers */
#include <stdint.h>               // for uint8_t, uint64_t, uint32_t
#include <stdbool.h>              // for bool

/* Early FSP headers */
#include <misc/ct-assert.h>       // for CT_ASSERT
#include <soc-common/hw-const.h>  // for MK_U32_CONST

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>          // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <error/common-errors.h>  // for error_t

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/**
 * @brief AST instance macro definitions
 *
 * @macro-title AST instance mask macros
 *
 * @MK_U32_CONST             Defines a u32 constant value
 * @MAX_AST_CONTROLLER       Defines maximum number of AST instances per cluster.
 * @AST_INSTANCE_R5          Mask to configure R5 instance.
 * @AST_INSTANCE_DMA         Mask to configure DMA instance.
 * @AST_INSTANCE_R5_AND_DMA  Mask to configure both R5 and DMA instances.
 * @MAX_REGION_INDEX         Max value of valid region index
 */
#define MAX_AST_CONTROLLER      MK_U32_CONST(2)
#define AST_INSTANCE_R5         (uint8_t)(1U << 0U)
#define AST_INSTANCE_DMA        (uint8_t)(1U << 1U)
#define AST_INSTANCE_R5_AND_DMA (AST_INSTANCE_R5 | AST_INSTANCE_DMA)
#define MAX_REGION_INDEX        MK_U32_CONST(7)

INLINE_RFD(MISRA, DEVIATE, Rule_8_6, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
const extern uint32_t ast_tegra_id[MAX_AST_CONTROLLER];

typedef uint64_t iova;

#define IOVA_NULL          ((iova)0U)

/**
 * @brief Structure defining the AST region map
 *
 * @master_base   Base address of the translated region in the system address
 *                space.
 * @slave_base    Base address of the translated region in the local address
 *                space
 * @size          Size of the translated region. Size should be defined as a
 *                power of 2 multiple of the align address and less than the
 *                size of the private address space
 * @streamid      StreamID used with the translated region. StreamID value for a
 *                VM is assigned by the hypervisor. This is don't care value for
 *                regions mapped with physical streamID.
 * @is_phy_region Indicator if the requested region is mapped using physical
 *                streamID
 */
struct tegra_ast_region_map {
	iova      master_base;
	uint64_t  slave_base;
	uint64_t  size;
	uint8_t   stream_id;
        bool      is_phy_region;
};

/**
 * @brief Enable an AST region for address translation
 * This API should be used to enable regions mapped with virtual
 * streamID.
 *
 * @param[in] region        Region number [0 - MAX_REGION_INDEX]
 * @param[in] stream_id     StreamID used with region as assigned by
 *                          the hypervisor.
 * @param[in] master        Base address in SoC address space
 * @param[in] slave         Base address in private address space
 * @param[in] size          Size of the region. Should be less than size
 *                          of the private address space
 * @param[in] instance_mask AST instance mask [AST_INSTANCE_R5,
 *                          AST_INSTANCE_DMA, AST_INSTANCE_R5_AND_DMA]
 *
 * @pre
 * - Region size must be programmed as power of 2 multiple of the align
 *   address.
 * - Regions may not overlap.
 * - Master and slave base addresses must be aligned to the region size.
 *
 * @func_req_id 10551222
 *
 * @retval 0                                For success
 * @retval E_AST_INVALID_VMINDEX            If streamID could not be set
 * @retval E_AST_INVALID_INPUT_PARAMETERS   For invalid region and instance_mask
 */
error_t tegra_ast_enable_region(uint8_t region, uint8_t stream_id,
                                iova master, uint64_t slave,
                                uint64_t size, uint8_t instance_mask);

/**
 * @brief Disable an AST region.
 *
 * Disables an AST region.
 *
 * @func_req_id 10551345
 *
 * @param[in] region         Region number [0 - MAX_REGION_INDEX]
 * @param[in] instance_mask  AST instance mask [AST_INSTANCE_R5,
 *                           AST_INSTANCE_DMA, AST_INSTANCE_R5_AND_DMA]
 *
 * @retval 0                                For success
 * @retval E_AST_INVALID_INPUT_PARAMETERS   For invalid region and instance_mask
 */
error_t tegra_ast_disable_region(uint8_t region, uint8_t instance_mask);

/**
 * @brief Get mapping for an AST region.
 *
 * @param[in]  region        Region number [0 - MAX_REGION_INDEX]
 * @param[in]  instance_mask AST instance mask [AST_INSTANCE_R5,
 *                                              AST_INSTANCE_DMA]
 * @param[out] region_map    Pointer to tegra_ast_region_map structure to store
 *                           region map
 *
 * @func_req_id 10551651
 *
 * @retval 0                                For success
 * @retval E_AST_REGION_DISABLED            If input AST region is disabled
 * @retval E_AST_INVALID_INPUT_PARAMETERS   For Null input region_map pointer
 * @retval E_AST_REGION_STREAMID_DISABLED   If region streamid is disabled
 */
error_t tegra_ast_get_region_mapping(uint8_t region,
		                     struct tegra_ast_region_map *return_region_map,
                                     uint8_t instance_mask);

/**
 * @brief Get the StreamID used with an AST region.
 *
 * @param[in] region        Region number [0 - MAX_REGION_INDEX]
 * @param[in] instance_mask AST instance mask [AST_INSTANCE_R5,
 *                                             AST_INSTANCE_DMA]
 * @param[out] streamID     Variable to capture stream ID
 *
 * @func_req_id 10551687
 *
 * @retval 0                                For success
 * @retval E_AST_REGION_DISABLED            If input AST region is disabled
 * @retval E_AST_REGION_STREAMID_DISABLED   If region streamid is disabled
 * @retval E_AST_INVALID_INPUT_PARAMETERS   For Null input region_map pointer
 * @retval E_AST_INVALID_PHY_REGION_REQUEST Request not applicable as the region
 *                                          is mapped with physical streamID
 */
error_t tegra_ast_get_region_stream_id(uint8_t region, uint8_t *streamID,
                                       uint8_t instance_mask);

/**
 * @brief Add and enable a StreamID.
 *
 * @param[in] stream_id     StreamID to add. StreamID for a VM is specified by
 *                          hypervisor
 * @param[in] instance_mask AST instance mask [AST_INSTANCE_R5,
 *                          AST_INSTANCE_DMA, AST_INSTANCE_R5_AND_DMA]
 *
 * @func_req_id 10551702
 *
 * @retval 0                        For success
 * @retval E_AST_INVALID_VMINDEX    If streamID could not be set
 */
error_t tegra_ast_add_streamid(uint8_t stream_id, uint8_t instance_mask);

/**
 * @brief Check if AST is globally locked.
 *
 * It is not possible to change the Physical StreamID or define new
 * regions with Physical StreamID if AST is globally locked.
 *
 * @func_req_id 10604979
 *
 * @retval true  if AST is globally locked
 * @retval false if AST is not globally locked
 */
bool tegra_ast_is_global_locked(void);

/**
 * @brief Map pointer to IOVA in ast region.
 *
 * Returns IOVA address for pointer in the slave address space
 *
 * @param[in] map     AST region mapping
 * @param[in] pointer Input pointer in the slave address space
 *
 * @func_req_id 10683207
 *
 * @retval IOVA_address If input slave address is valid and lies with the
 *                      AST region map
 * @retval IOVA_NULL    Otherwise
 */
iova tegra_ast_map_pointer_to_iova(const struct tegra_ast_region_map *map,
                                   const void *pointer);

/**
 * @brief Map IOVA to local pointer value in ast region.
 *
 * Returns pointer value in local address space corresponding to the input IOVA
 *
 * @param[in] map  AST region mapping
 * @param[in] iova 40-bit valid I/O virtual address
 *
 * @func_req_id 10684041
 *
 * @retval Slave_address_pointer_value  If input IOVA is valid and lies withing the
 *                                      AST region map
 * @retval 0                            Otherwise
 */
uint64_t tegra_ast_get_local_pointer_value(const struct tegra_ast_region_map *map,
                                           const iova iova_addr);

/**
 * @brief Map slave address to IOVA in ast region.
 *
 * Returns IOVA corresponding to the local address in slave address space.
 *
 * @param[in] map   AST region mapping
 * @param[in] slave Input slave address
 *
 * @func_req_id 10683153
 *
 * @retval IOVA      If input slave address is valid and lies within the AST
 *                   region map
 * @retval IOVA_NULL Otherwise
 */
iova tegra_ast_map_slave_to_iova(const struct tegra_ast_region_map *map,
                                 const uint64_t slave);

/**
 * @brief Map IOVA to slave address in ast region.
 *
 * Returns address in slave address space corresponding to the input IOVA
 *
 * @param[in] map  AST region mapping
 * @param[in] iova 40-bit valid I/O virtual address
 *
 * @func_req_id 10683234
 *
 * @retval slave_address If input IOVA is valid and lies within the AST region
 *                       map
 * @retval 0             Otherwise
 */
uint64_t tegra_ast_map_iova_to_slave(const struct tegra_ast_region_map *map,
                                     const iova iova_addr);

/**
 * @brief Set Default Stream ID
 * Enables physical stream ID or adds an entry to the streamID control table for default access
 *
 * @param[in] instance_mask AST instance mask [AST_INSTANCE_R5,
 *                          AST_INSTANCE_DMA, AST_INSTANCE_R5_AND_DMA]
 * @param[in] phy           Bool variable to set input stream ID as physical streamID
 * @param[in] stream_id     StreamID value to be set as specified by the hypervisor
 *
 * @func_req_id 13063249
 *
 * @retval 0                        For success
 * @retval E_AST_GLOBALLY_LOCKED    Physical stream ID could not be set as AST
 *                                  is globally locked
 * @retval E_AST_INVALID_VMINDEX    If streamID could not be set.
 */
error_t tegra_ast_set_default_stream_id(uint8_t stream_id, uint8_t instance_mask,
                                        bool phy);

/**
 * @brief Enable an AST region with physical streamID
 *
 * @param[in] region        Region number [0 - MAX_REGION_INDEX]
 * @param[in] master        Base address in SoC address space
 * @param[in] slave         Base address in private address space
 * @param[in] size          Size of the region. Should be less than size
 *                          of the private address space
 * @param[in] instance_mask AST instance mask [AST_INSTANCE_R5,
 *                          AST_INSTANCE_DMA, AST_INSTANCE_R5_AND_DMA]
 *
 * @pre
 * - Region size must be programmed as power of 2 multiple of the align
 *   address.
 * - Regions may not overlap.
 * - Master and slave base addresses must be aligned to the region size.
 *
 * @func_req_id 10551222
 *
 * @retval 0                                For success
 * @retval E_AST_GLOBALLY_LOCKED            Region could not be enabled as AST
 *                                          is globally locked
 * @retval E_AST_INVALID_INPUT_PARAMETERS   For invalid region index,
 *                                          instance_mask, master/slave address
 *                                          and size of the region.
 */
error_t tegra_ast_enable_phy_region(uint8_t region,
                                    iova master,
                                    uint64_t slave,
                                    uint64_t size,
                                    uint8_t instance_mask);
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif /* AST__TEGRA_AST_H */
