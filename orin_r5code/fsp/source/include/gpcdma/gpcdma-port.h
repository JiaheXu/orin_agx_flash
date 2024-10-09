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

#ifndef GPCDMA__GPCDMA_PORT_H
#define GPCDMA__GPCDMA_PORT_H
#define FSP__GPCDMA__GPCDMA_PORT_H                      1

/**
 * @file gpcdma/gpcdma-port.h
 * @brief APIs that the gpcdma controller driver depends on and needs to
 *        be implemented by the gpcdma port layer.
 */

/* Compiler headers */
#include <stdbool.h>                // for bool
#include <stdint.h>
                                    // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* LateFSP headers */
#include <error/common-errors.h>
#include <misc/macros.h>            // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

// IWYU pragma: no_forward_declare gpcdma_id
struct gpcdma_id;

/**
 * @brief gpcdma channel setup hook
 *
 * @jama_func_req_id 17662895
 *
 * This function is used to perform the port specific per-channel setup.
 *
 * @note called from function gpcdma_init()

 * @param[in] chan_sem DMA channel id
 *
 * @retval E_SUCESS  on success
 */
error_t gpcdma_port_chan_setup(uint32_t chan_num);

/**
 * @brief gpcdma channel sync hook
 *
 * @jama_func_req_id 17663510
 *
 * This function is used to block/wait until the synchronous
 * transfer is finished.
 *
 * @pre the function gpcdma_init() has been called
 * @note called from gpcdma_transfer() for synchronous transfers
 *
 * @param[in] chan_sem DMA channel id
 * @param[in] timeout Timeout to wait before bailing out
 *
 * @retval E_SUCESS                     on success
 * @retval E_GPCDMA_INVALID_PARAM       invalid DMA channel id
 * @retval E_GPCDMA_PORT_SYNC_TIMEOUT   timed out for synchronous transfers
 */
error_t gpcdma_port_chan_sync(uint32_t chan_num,
                              uint32_t timeout);

/**
 * @brief gpcdma channel sync complete hook
 *
 * @jama_func_req_id 17671472
 *
 * This function is used to notify/unblock the channel that is blocked on
 * a synchronous transfer.
 *
 * @pre the function gpcdma_init() has been called
 * @note called from gpcdma_chan_irq() for synchronous transfers
 *
 * @param[in] chan_num  DMA channel id of the controller
 *
 * @retval E_SUCESS                 on success
 * @retval E_GPCDMA_INVALID_PARAM   invalid DMA channel id
 */
error_t gpcdma_port_chan_sync_end(uint32_t chan_num);

/**
 * @brief gpcdma port specific controller init hook
 *
 * @jama_func_req_id 17662889
 *
 * @note called from function gpcdma_init()
 *
 * This function is used to perform some port specific init such
 * as resetting the controller after the common init.
 *
 * @param[in] id dma controller context
 *
 * @retval E_SUCESS                 on success
 * @retval E_GPCDMA_PORT_INIT_FAIL  port specific init failed
 */
error_t gpcdma_port_init(struct gpcdma_id *id);

/**
 * @brief gpcdma port specific queue creation and init function
 *
 * @note Only used during continuous mode
 *
 * This function is used to create queue to store the list of the
 * buffers (src and dst) in cyclic/continuous mode.
 *
 * @param[in] chan_num dma channel number
 * @param[in] nelems   number of elements to create queue for
 * @param[in] el_size  size of the elements
 *
 * @retval E_SUCESS                 on success
 * @retval E_GPCDMA_INVALID_PARAM   invalid parameters
 * @retval E_GPCDMA_QUEUE_OP_FAIL   queue creation failed
 */
error_t gpcdma_port_init_queue(uint32_t chan_num, uint32_t nelems,
                               uint32_t el_size);

/**
 * @brief gpcdma port specific queue delete/deinit function
 *
 * @note Only used during continuous mode
 *
 * This function is used to deinit or delete queue.
 *
 * @param[in] chan_num dma channel number
 */
void gpcdma_port_delete_queue(uint32_t chan_num);

/**
 * @brief gpcdma port specific get element function
 *
 * @note Only used during continuous mode
 *
 * This function is used to retrieve the element or descriptor from the queue.
 *
 * @param[in]  chan_num   dma channel number
 * @param[out] ptr        store de-queued element
 * @param[in]  timeout    timeout to wait dequeuing element
 * @param[in]  from_isr   indicates if API is being called from ISR
 *
 * @retval E_SUCESS                 on success
 * @retval E_GPCDMA_INVALID_PARAM   on invalid parameters
 * @retval E_GPCDMA_QUEUE_OP_FAIL   on failure to retrieve the descriptor.
 */
error_t gpcdma_port_get_desc(uint32_t chan_num, void *ptr, uint32_t timeout,
                             bool from_isr);

/**
 * @brief gpcdma port specific store element at the back function
 *
 * @note Only used during continuous mode
 *
 * This function is used to send or store the element or descriptor
 * at the back of the queue.
 *
 * @param[in] chan_num dma channel number
 * @param[in] ptr      element to store
 * @param[in] timeout  timeout to wait dequeuing element
 * @param[in] from_isr indicates if API is being called from ISR
 *
 * @retval E_SUCESS                 on success
 * @retval E_GPCDMA_INVALID_PARAM   on invalid parameters
 * @retval E_GPCDMA_QUEUE_OP_FAIL   on failure to retrieve the descriptor.
 */
error_t gpcdma_port_send_desc_to_back(uint32_t chan_num, void *ptr,
                                      uint32_t timeout, bool from_isr);
#endif
