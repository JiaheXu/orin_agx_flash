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

#ifndef GPCDMA__GPCDMA_H
#define GPCDMA__GPCDMA_H
#define FSP__GPCDMA__GPCDMA_H                      1

/**
 * @file gpcdma/gpcdma-tegra.h
 * @brief functions for performing DMA operations.
 */

/* Compiler headers */
#include <stdbool.h>
#include <stdint.h>
                                        // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>        // for error_t, FSP__ERROR__COMMON...
#include <misc/macros.h>                // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

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
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/*
 * The completion status of a DMA transfer.
 *
 * These values will be returned by DMA driver function which initiated DMA
 * transfers. For asynchronous transfers, the value returned represents the
 * status of submitting the request to the hardware. For synchronous
 * transfers, the value returned represents the overall status of the
 * transaction.
 *
 * For asynchronous transfers, these values will also be passed to the caller-
 * supplied completion callback.
 *
 *                Ret async    Ret sync    Async callback
 * NOT_INITIATED: Y            Y           N
 * EXECUTING:     Y            N           N
 * COMPLETE:      N            Y           Y
 * ABORTED:       N            N           Y
 * TIMEOUT:       N            Y           N
 */
typedef uint32_t dma_status;

#define DMA_STATUS_NOT_INITIATED    0x0UL
#define DMA_STATUS_EXECUTING        0x1UL
#define DMA_STATUS_COMPLETE         0x2UL
#define DMA_STATUS_ABORTED          0x3UL
#define DMA_STATUS_TIMEOUT          0x4UL

/**
 * @brief dma callback function
 *
 * The function is called whenever a DMA transfer has completed.
 *
 * @param[in] callback_param A copy of a value passed to the DMA driver function
 *                           that initiated the transfer. Clients may used this
 *                           to pass context to the DMA completion callback.
 * @param[in ]status Status of the transfer.
 *
 *@ return None
 */
typedef void dma_callback(void *callback_param, dma_status status);

/* Supported DMA transfer combinations */
typedef uint32_t gpcdma_transfer_direction;

#define GPCDMA_XFER_DIR_MEM_TO_IO   0x0UL
#define GPCDMA_XFER_DIR_IO_TO_MEM   0x1UL
#define GPCDMA_XFER_DIR_MEM_TO_MEM  0x2UL

/* Supported IO bus widths for DMA transfers */
typedef uint32_t gpcdma_bus_width;

#define GPCDMA_IO_BUS_WIDTH_8       0x0UL
#define GPCDMA_IO_BUS_WIDTH_16      0x1UL
#define GPCDMA_IO_BUS_WIDTH_32      0x2UL
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

/**
 * direction:       A valid gpcdma_transfer_direction value.
 * bus_width:       IO buswidth. A valid gpcdma_bus_width value.
 * burst_size:      The number of words to transfer in each burst. Valid for
 *                  both tx/rx transfers.
 * src_addr:        The source address to read from.
 * src_addr_wrap:   The number of word transfers to make before wrapping
 *                  the src address. 0 represents no wrap.
 *                  Legal values for io addr wrap are 2^n with n from
 *                  0 through 6.
 *                  Legal values for mc addr wrap are 2^n with n from
 *                  5 through 11.
 * dst_addr:        The target address to write to.
 * dst_addr_wrap:   The number of word transfers to make before wrapping
 *                  the mc address. 0 represents no wrap.
 *                  Legal values for io addr wrap are 2^n with n from
 *                  0 through 6.
 *                  Legal values for mc addr wrap are 2^n with n from
 *                  5 through 11.
 * period_len:      Used in continuous mode, callback will be called at
 *                  every period_len byte transfers. It has to be word
 *                  aligned and value must be in bytes.
 * xfer_count:      The number of bytes to be transferred. The value must
 *                  be word aligned. If continuous mode is used, this value
 *                  must be multiple of period_len parameter.
 * timeout:         Timeout for synchronous transfers.
 * callback:        A function to execute once the transfer is done.
 *                  Callback is valid only for asynchronous transfers.
 * callback_param:  Data to be passed as context to the callback function.
 * slave_req:       HW specific slave request select value. Used for both
 *                  rx/tx transfers.
 * src_sid:         Source stream id.
 * dst_sid:         Destination stream id.
 * en_flow_ctrl:    Use rx/tx slave request or not.
 * synchronous:     Execute a synchronous transfer or not.
 *                  If true, wait till DMA transfer is done and return
 *                  status.
 *                  If false, return after starting DMA transfer and execute
 *                  the provided callback upon transfer completion.
 * continuous:      Set the DMA continuous transfer mode. This option is
 *                  mutually exclusive with synchronous option above.
 */
struct gpcdma_xfer {
    gpcdma_transfer_direction  direction;
    gpcdma_bus_width           bus_width;
    uint32_t                   burst_size;
    uint64_t                   src_addr;
    uint32_t                   src_addr_wrap;
    uint64_t                   dst_addr;
    uint32_t                   dst_addr_wrap;
    uint32_t                   period_len;
    uint32_t                   xfer_count;
    uint32_t                   timeout;
    dma_callback               *callback;
    void                       *callback_param;
    uint8_t                    slave_req;
    uint8_t                    src_sid;
    uint8_t                    dst_sid;
    bool                       en_flow_ctrl;
    bool                       synchronous;
    bool                       continuous;
};

/* The identity of GPCDMA controller instance */
// IWYU pragma: no_forward_declare gpcdma_id
struct gpcdma_id;

/**
 * @brief global initialization of the GPCDMA controller.
 *
 * @jama_func_req_id 17644910
 *
 * @pre None
 *
 * @param[in] id        DMA controller context
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_GPCDMA_NULL_PTR                invalid id paramter passed
 * @retval E_GPCDMA_INVALID_PARAM           invalid controller base address
 * @retval E_GPCDMA_PORT_INIT_FAIL          port specific init failed
 * @retval E_GPCDMA_PORT_CHAN_SETUP_FAIL    port specific channel setup failed
 */
error_t gpcdma_init(struct gpcdma_id *id);

/**
 * @brief Execute DMA transfer on the specified channel.
 *
 * @jama_func_req_id 17645012
 *
 * For synchronous transfers, transfer status would be returned.
 * For asynchronous transfers, provided callback would be executed upon
 * transfer completion.
 *
 * @pre the function gpcdma_init() has been called
 *
 * @param[in] id        DMA controller context
 * @param[in] chan_num  DMA channel id of the controller
 * @param[in] xfer      DMA transfer details
 *
 * @retval E_SUCCESS                     indicates success
 * @retval E_GPCDMA_NULL_PTR             invalid id paramter passed
 * @retval E_GPCDMA_INVALID_PARAM        invalid DMA channel id
 * @retval E_GPCDMA_CHAN_BUSY            DMA channel is busy
 * @retval E_GPCDMA_INVALID_XFER_DIR     invalid transfer direction
 * @retval E_GPCDMA_NO_CALLBACK_ASYNC    No callback provided for asynchrous
 *                                       transfer
 * @retval E_GPCDMA_INVALID_XFER_SIZE    invalid transfer size
 * @retval E_GPCDMA_INVALID_XFER_SIZE_IO invalid transfer size for IO<->MEM
 * @retval E_GPCDMA_UNALIGNED_SRC        unaligned source address
 * @retval E_GPCDMA_UNALIGNED_DST        unaligned destination address
 * @retval E_GPCDMA_INVALID_FC_REQ_ID    invalid flow control slave request id
 * @retval E_GPCDMA_INVALID_ADDR_WRAP    invalid source/dest address wrap
 * @retval E_GPCDMA_PORT_SYNC_TIMEOUT    timed out for synchronous transfers
 */
error_t gpcdma_transfer(const struct gpcdma_id *id,
                        uint32_t chan_num,
                        struct gpcdma_xfer const *xfer);

/**
 * @brief get transferred size of the transfer in progress
 *
 * @jama_func_req_id 17649722
 *
 * This function is used to fetch the number of bytes transferred
 * successfully using DMA.
 *
 * @pre the function gpcdma_init() has been called
 *
 * @param[in]  id         DMA controller context
 * @param[in]  chan_num   DMA channel id of the controller
 * @param[out] xfer_count Total bytes transferred so far
 *
 * @retval E_SUCCESS indicates success
 * @retval E_GPCDMA_NULL_PTR invalid id paramter passed
 * @retval E_GPCDMA_INVALID_PARAM invalid DMA channel id
 */
error_t gpcdma_get_bytes_xferred(const struct gpcdma_id *id,
                                 uint32_t chan_num,
                                 uint64_t *xfer_count);

/**
 * @brief Abort the DMA transfer in progress.
 *
 * @jama_func_req_id 17649740
 *
 * This function is used to abort the ongoing transfer on a dma channel.
 *
 * @pre the function gpcdma_init() has been called
 *
 * @param[in] id        DMA controller context
 * @param[in] chan_num  DMA channel id of the controller
 *
 * @retval E_SUCCESS                        indicates success
 * @retval E_GPCDMA_NULL_PTR                invalid id paramter passed
 * @retval E_GPCDMA_INVALID_PARAM           invalid controller base address
 */
error_t gpcdma_abort(const struct gpcdma_id *id,
                     uint32_t chan_num);

/**
 * @brief GPCDMA channel irq handler.
 *
 * @jama_func_req_id 17656319
 *
 * This function gets called whenever a DMA transfer is completed.
 *
 * @pre the function gpcdma_init() has been called
 *
 * @param[in] data pointer for GPCDMA channel context.
 *
 * @return None
 */
void gpcdma_chan_irq(void *data);

#endif
