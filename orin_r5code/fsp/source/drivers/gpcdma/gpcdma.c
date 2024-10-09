_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_1_2 \"Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx\") \
         (deviate MISRA_C_2012_Directive_4_8 \"Approval: Bug 200534384, DR: SWE-FSP-011-SWSADR.docx\")")
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
#include <stdbool.h>                   // for bool
#include <stdint.h>                    // for uint32_t, uint8_t, UINT32_...
#include <string.h>                    // for NULL
                                       // IWYU pragma: no_include <errno.h>

/* Early FSP headers */
#include <misc/ct-assert.h>            // for CT_ASSERT
#include <soc-common/hw-const.h>       // IWYU pragma: keep
#include <soc-common/argpcdma-defs.h>  // GPCDMA_CHANNEL_CH...

/* Hardware headers */
#include <misc/nvrm_drf.h>             // NV_DRF_* macros

/* Late FSP headers */
#include <cpu/type-conversion.h>       // for fsp_c_v_ptr_to_u32
#include <error/common-errors.h>       // for error_t, E_SUCCESS
#include <irq/safe-irqs.h>             // for in_interrupt, irq_safe_enable, irq_safe_set...
#include <misc/attributes.h>           // for UNUSED
#include <misc/bitops.h>               // for BIT, bit_number, FSP__MISC__BITOPS_H
#include <misc/macros.h>               // for ARRAY_SIZE
#include <reg-access/reg-access.h>     // for readl_base_offset, writel_base_of...
#include <debug/assert.h>              // for ASSERT, ...
#include <delay/delay.h>               // for udelay
#include <lock/smplock-up.h>           // for smp_unlock_irqrestore, smp_loc...

/* Module specific headers */
#include <gpcdma/gpcdma-errors.h>      // for E_GPCDMA_NULL_PTR, E_GPCDMA_...
#include <gpcdma/sections-gpcdma.h>    // Immune from CT_ASSERT protection
#include <gpcdma/gpcdma-priv.h>        // for struct gpcdma_id, gpcdma_channel...
#include <gpcdma/gpcdma-port.h>        // for gpcdma_port_...
#include <gpcdma/gpcdma.h>             // for gpcdma_xfer and gpcdma_* apis

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
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__ARGPCDMA_DEFS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CPU__TYPE_CONVERSION_H, "Header file missing or invalid.")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__IRQ__SAFE_IRQS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ASSERT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DELAY__DELAY_H, "Header file missing or invalid.")
CT_ASSERT(FSP__LOCK__SMPLOCK_UP_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_PRIV_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_PORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__GPCDMA__GPCDMA_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

#define GPCDMA_ADDR_MASK     0xFFFFFFFFUL
#define GPCDMA_HI_ADDR_MASK  0xFFU
#define GPCDMA_HI_ADDR_SHIFT 32UL

SECTION_GPCDMA_TEXT static inline uint32_t
gpcdma_chan_readl(const struct gpcdma_channel *dma_chan,
                  uint32_t reg)
{
    return readl_base_offset(dma_chan->chan_base, reg);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_chan_writel(const struct gpcdma_channel *dma_chan,
                   uint32_t val,
                   uint32_t reg)
{
    writel_base_regoffset(val, dma_chan->chan_base, reg);
}

SECTION_GPCDMA_TEXT static inline bool
gpcdma_chan_is_valid(const struct gpcdma_id *id,
                     uint32_t chan_num)
{
    return (chan_num < ARRAY_SIZE(id->channels));
}

SECTION_GPCDMA_TEXT static inline struct
gpcdma_channel *gpcdma_chan(const struct gpcdma_id *id,
                            uint32_t chan_num)
{
    return id->channels[chan_num];
}

SECTION_GPCDMA_TEXT static inline uint32_t
gpcdma_chan_base(const struct gpcdma_id *id,
                 uint32_t chan)
{
    const uint32_t channel_stride = GPCDMA_CHANNEL_CH1_CSR_0 -
                                        GPCDMA_CHANNEL_CH0_CSR_0;

    INLINE_RFD(CERTC, DEVIATE, INT30_C,"Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    return id->conf.base_addr + (chan * channel_stride);
}

#if defined(HW_BUG_200667678) && (HW_BUG_200667678 == 1)
SECTION_GPCDMA_TEXT static inline void
gpcdma_disable_virtualization(const struct gpcdma_id *id)
{
    writel_base_regoffset(0UL, id->conf.base_addr,
                          GPCDMA_COMMON_DMA_CHAN_VIRTUALIZATION_ENABLE_0);
}
#endif

/* Create list of buffers each equal to period_len for the cyclic mode */
SECTION_GPCDMA_TEXT static error_t
gpcdma_create_list(struct gpcdma_xfer const *transfer, uint32_t id)
{
    struct gpcdma_buf_desc desc;
    uint32_t el_cnt, period_len, rem_len;
    uint64_t src_addr, dst_addr, cont_buf, const_buf;
    error_t ret = E_SUCCESS;

    dst_addr = transfer->dst_addr;
    src_addr = transfer->src_addr;
    rem_len = transfer->xfer_count;
    period_len = transfer->period_len;
    el_cnt = rem_len / period_len;

    if (transfer->direction == GPCDMA_XFER_DIR_IO_TO_MEM) {
        cont_buf = dst_addr;
        const_buf = src_addr;
    } else {
        cont_buf = src_addr;
        const_buf = dst_addr;
    }

    ret = gpcdma_port_init_queue(id, el_cnt, sizeof(struct gpcdma_buf_desc));

    if (ret != E_SUCCESS) {
        goto out_no_del;
    }

    while ((rem_len != 0UL) && (ret == E_SUCCESS)) {
        if (transfer->direction == GPCDMA_XFER_DIR_IO_TO_MEM) {
            desc.src_addr = (uint32_t)(const_buf & GPCDMA_ADDR_MASK);
            desc.dst_addr = (uint32_t)(cont_buf & GPCDMA_ADDR_MASK);
            desc.hi_src_addr = (uint32_t)((const_buf >> GPCDMA_HI_ADDR_SHIFT) &
                                          GPCDMA_HI_ADDR_MASK);
            desc.hi_dst_addr = (uint32_t)((cont_buf >> GPCDMA_HI_ADDR_SHIFT) &
                                          GPCDMA_HI_ADDR_MASK);
        } else {
            desc.src_addr = (uint32_t)(cont_buf & GPCDMA_ADDR_MASK);
            desc.dst_addr = (uint32_t)(const_buf & GPCDMA_ADDR_MASK);
            desc.hi_src_addr = (uint32_t)((cont_buf >> GPCDMA_HI_ADDR_SHIFT) &
                                          GPCDMA_HI_ADDR_MASK);
            desc.hi_dst_addr = (uint32_t)((const_buf >> GPCDMA_HI_ADDR_SHIFT) &
                                          GPCDMA_HI_ADDR_MASK);
        }
        ret = gpcdma_port_send_desc_to_back(id, (void *)&desc, 0UL, false);

        if (ret != E_SUCCESS) {
            continue;
        }

        if (rem_len < period_len) {
            ret = E_GPCDMA_INVALID_PARAM;
        } else {
            rem_len -= period_len;
        }

        if ((UINT64_MAX - cont_buf) < period_len) {
            ret = E_GPCDMA_INVALID_PARAM;
        } else {
            cont_buf += period_len;
        }
    }

    if (ret != E_SUCCESS) {
        gpcdma_port_delete_queue(id);
    }

out_no_del:
    return ret;
}
/* End of cyclic mode codes */

SECTION_GPCDMA_TEXT static inline uint32_t
gpcdma_wcount_to_wrap_sz(uint32_t wcount,
                         bool io_wrap)
{
    uint32_t wrap = 0UL;

    if (wcount == 0UL) {
        goto out;
    }

    /*
     * mmio address wrap : 2^n where n ranges from 0 to 6
     * mc address wrap : 2^n where n ranges from 5 to 11
     */
    wrap = io_wrap ? (bit_number(wcount) + 1UL) :
                     ((bit_number(wcount) + 1UL) - 5UL);

out:
    return wrap;
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_configure_wcount(const struct gpcdma_channel *dma_chan)
{
    /*
     * byte count register expects the value to be in words.
     * Value N means (N + 1) words transfer request size.
     */
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, Example-5, DR: SWE-FSP-046-SWSADR.docx");
    gpcdma_chan_writel(dma_chan, (dma_chan->words_to_xfer - 1UL),
                       GPCDMA_CHANNEL_CH0_WCOUNT_0);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_configure_mc_sequencer(const struct gpcdma_channel *dma_chan,
                              const struct gpcdma_xfer *xfer,
                              bool src_io,
                              bool dst_io)
{
    uint32_t val;
    uint32_t mc_src_wrap, mc_dst_wrap;

    mc_src_wrap = src_io ? 0UL :
                    gpcdma_wcount_to_wrap_sz(xfer->src_addr_wrap, false);
    mc_dst_wrap = dst_io ? 0UL :
                    gpcdma_wcount_to_wrap_sz(xfer->dst_addr_wrap, false);

    /*
     * FIXME: Figure out a way to drive this burst size by xfer->burst_size.
     */
    val = NV_DRF_DEF(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_BURST, DMA_BURST_16WORDS) |
        NV_DRF_DEF(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_REQ_CNT, DEFAULT) |
        NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_ADDR_WRAP0, mc_src_wrap) |
        NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_ADDR_WRAP1, mc_dst_wrap);

    /* program the stream ids depending on the xfer direction */
    switch (xfer->direction) {
    case GPCDMA_XFER_DIR_MEM_TO_MEM:
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ,
                          STREAMID0, xfer->src_sid);
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ,
                          STREAMID1, xfer->dst_sid);
        break;
    case GPCDMA_XFER_DIR_IO_TO_MEM:
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ,
                          STREAMID0, xfer->dst_sid);
        break;
    case GPCDMA_XFER_DIR_MEM_TO_IO:
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ,
                          STREAMID0, xfer->src_sid);
        break;

    default:
        /* will never run into this as the transfer is already validated */
        break;
    }

    gpcdma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_MC_SEQ_0);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_configure_mmio_sequencer(const struct gpcdma_channel *dma_chan,
                                const struct gpcdma_xfer *xfer,
                                bool src_io,
                                bool dst_io)
{
    uint32_t val = 0UL;
    uint32_t io_wrap;

    /*
     * MMIO sequencer needs to be programmed only for IO2MEM or MEM2IO
     * transfers.
     */
    if (src_io || dst_io) {
        val = NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MMIO_SEQ, MMIO_BUS_WIDTH,
                         xfer->bus_width);
        INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, Example-5, DR: SWE-FSP-046-SWSADR.docx");
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MMIO_SEQ, MMIO_BURST,
                          (xfer->burst_size - 1UL));
        io_wrap = src_io ? gpcdma_wcount_to_wrap_sz(xfer->src_addr_wrap, true) :
                           gpcdma_wcount_to_wrap_sz(xfer->dst_addr_wrap, true);
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MMIO_SEQ, MMIO_ADDR_WRAP, io_wrap);
    }

    gpcdma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_MMIO_SEQ_0);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_clear_error_status(const struct gpcdma_channel *dma_chan)
{
    gpcdma_chan_writel(dma_chan, 0xFFFFFFFFUL,
                       GPCDMA_CHANNEL_CH0_ERR_STA_0);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_configure_hi_adr_ptr(const struct gpcdma_channel *dma_chan,
                            uint32_t hi_src_ptr, uint32_t hi_dst_ptr)
{
    uint32_t val;

    val = NV_DRF_NUM(GPCDMA, CHANNEL_CH0_HI_ADR_PTR,
                     HI_DST_PTR, hi_dst_ptr) |
          NV_DRF_NUM(GPCDMA, CHANNEL_CH0_HI_ADR_PTR,
                     HI_SRC_PTR, hi_src_ptr);

    gpcdma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_HI_ADR_PTR_0);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_configure_addr_ptrs(const struct gpcdma_channel *dma_chan,
                           uint32_t src_addr, uint32_t dst_addr)
{
    gpcdma_chan_writel(dma_chan, src_addr, GPCDMA_CHANNEL_CH0_SRC_PTR_0);
    gpcdma_chan_writel(dma_chan, dst_addr, GPCDMA_CHANNEL_CH0_DST_PTR_0);
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_configure_csr(const struct gpcdma_channel *dma_chan,
                     const struct gpcdma_xfer *xfer,
                     uint32_t dma_mode)
{
    uint32_t val;

    val = dma_mode;
    if (xfer->en_flow_ctrl) {
        val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_CSR, REQ_SEL,
                          xfer->slave_req);
    }

    if (xfer->continuous) {
        val |= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, ONCE, CYCLIC_MODE);
    } else {
        val |= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, ONCE, SINGLE_BLOCK);
    }
    val |= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, IRQ_MASK, ENABLE) |
           NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, IE_EOC, ENABLE) |
           NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, WEIGHT, DEFAULT) |
           NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, ENB, ENABLE);

    gpcdma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_CSR_0);
}

SECTION_GPCDMA_TEXT static error_t
gpcdma_get_xfer_dma_mode(const struct gpcdma_xfer *transfer,
                         uint32_t *dma_mode,
                         bool *src_io,
                         bool *dst_io)
{
    error_t ret = E_SUCCESS;

    switch (transfer->direction) {
    case GPCDMA_XFER_DIR_MEM_TO_IO:
        *dma_mode = (transfer->en_flow_ctrl) ?
            NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, MEM2IO_FC) :
            NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, MEM2IO_NO_FC);
        *src_io = false;
        *dst_io = true;
        break;
    case GPCDMA_XFER_DIR_IO_TO_MEM:
        *dma_mode = (transfer->en_flow_ctrl) ?
            NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, IO2MEM_FC) :
            NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, IO2MEM_NO_FC);
        *src_io = true;
        *dst_io = false;
        break;
    case GPCDMA_XFER_DIR_MEM_TO_MEM:
        *dma_mode = NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, MEM2MEM);
        *src_io = false;
        *dst_io = false;
        break;
    default:
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_XFER_DIR;
        break;
    }

    return ret;
}

SECTION_GPCDMA_TEXT static inline error_t
gpcdma_validate_flow_cntrl(const struct gpcdma_xfer *xfer)
{
    error_t ret = E_SUCCESS;

    if (xfer->en_flow_ctrl) {
        if (xfer->slave_req > GPCDMA_CHANNEL_CH0_CSR_0_REQ_SEL_DEFAULT_MASK) {
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            ret = E_GPCDMA_INVALID_FC_REQ_ID;
        }
    }

    return ret;
}

SECTION_GPCDMA_TEXT static error_t
gpcdma_validate_wrap(uint32_t wrap,
                     bool io_addr)
{
    error_t     ret = E_SUCCESS;
    uint32_t    val;
    uint32_t    wrap_mask;

    /* wrap must be a power of 2 */
    if ((wrap > 0UL) && ((wrap & (wrap - 1UL)) != 0UL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_ADDR_WRAP;
        goto out;
    }

    val = gpcdma_wcount_to_wrap_sz(wrap, io_addr);
    wrap_mask = (io_addr ?
                 GPCDMA_CHANNEL_CH0_MMIO_SEQ_0_MMIO_ADDR_WRAP_DEFAULT_MASK :
                 GPCDMA_CHANNEL_CH0_MC_SEQ_0_MC_ADDR_WRAP0_DEFAULT_MASK);

    if (val > wrap_mask) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_ADDR_WRAP;
        goto out;
    }

out:
    return ret;
}

SECTION_GPCDMA_TEXT static error_t
gpcdma_validate_xfer(struct gpcdma_channel *dma_chan,
                     struct gpcdma_xfer const *xfer,
                     bool src_io,
                     bool dst_io)
{
    error_t     ret = E_SUCCESS;
    uint32_t    align_mask;

    /* check if callback exists for asynchronous transfers */
    if (!xfer->synchronous && (xfer->callback == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_NO_CALLBACK_ASYNC;
        goto out;
    }

    /* Sanity check for continuous mode */
    if (xfer->continuous) {
        if (xfer->period_len == 0UL) {
            ret = E_GPCDMA_INVALID_PARAM;
            goto out;
        }
        if ((xfer->period_len & 3UL) != 0UL) {
            ret = E_GPCDMA_INVALID_PARAM;
            goto out;
        }
        if ((xfer->xfer_count % xfer->period_len) != 0UL) {
            ret = E_GPCDMA_INVALID_PARAM;
            goto out;
        }
        if (xfer->direction == GPCDMA_XFER_DIR_MEM_TO_MEM) {
            ret = E_GPCDMA_INVALID_PARAM;
            goto out;
        }
    }

    /* update the channel callback */
    dma_chan->callback = (xfer->synchronous) ? NULL : xfer->callback;
    dma_chan->cdata = xfer->callback_param;
    dma_chan->synchronous = xfer->synchronous;

    /* check if the transfer size is word aligned */
    if ((xfer->xfer_count == 0UL) || ((xfer->xfer_count & 0x3UL) != 0UL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_XFER_SIZE;
        goto out;
    }

    /*
     * for IO2MEM or MEM2IO, transfer size should be aligned to MMIO burst
     * size. MMIO burst_size is represented in words. Convert it to bytes
     * in computing the mask for alignment check.
     */
    align_mask = (xfer->burst_size << 2) - 1UL;
    if (((xfer->xfer_count & align_mask) != 0UL) && (src_io || dst_io)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_XFER_SIZE_IO;
        goto out;
    }

    /*
     * validate [MC/IO] source address wrap and destination address wrap fields
     * of the transfer.
     */
    ret = gpcdma_validate_wrap(xfer->src_addr_wrap, src_io);
    if (ret != E_SUCCESS) {
        goto out;
    }
    ret = gpcdma_validate_wrap(xfer->dst_addr_wrap, dst_io);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /*
     * IO address needs to be aligned with IO address wrap size.
     * MC address needs to be word aligned.
     */
    align_mask = (xfer->src_addr_wrap != 0UL) ? (xfer->src_addr_wrap - 1UL) : 0UL;
    if (((xfer->src_addr & 0x3UL) != 0UL) || (src_io &&
            ((xfer->src_addr & align_mask) != 0UL))) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_UNALIGNED_SRC;
        goto out;
    }
    align_mask = (xfer->dst_addr_wrap != 0UL) ? (xfer->dst_addr_wrap - 1UL) : 0UL;
    if (((xfer->dst_addr & 0x3UL) != 0UL) || (dst_io &&
            ((xfer->dst_addr & align_mask) != 0UL))) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_UNALIGNED_DST;
        goto out;
    }

    /*
     * If flow control is enabled, validate the slave request ID.
     */
    ret = gpcdma_validate_flow_cntrl(xfer);
    if (ret != E_SUCCESS) {
        goto out;
    }

out:
    return ret;
}

SECTION_GPCDMA_TEXT static error_t
gpcdma_get_chan_context(const struct gpcdma_id *id,
                        uint32_t chan_num,
                        struct gpcdma_channel **dma_chan)
{
    error_t ret = E_SUCCESS;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }

    if (!id->inited) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_NOT_INITED;
        goto out;
    }

    if (!gpcdma_chan_is_valid(id, chan_num)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    *dma_chan = gpcdma_chan(id, chan_num);

out:
    return ret;
}

SECTION_GPCDMA_TEXT static error_t
gpcdma_pause(const struct gpcdma_channel *dma_chan)
{
    uint32_t timeout;
    uint32_t csre;
    error_t ret = E_SUCCESS;

    if ((dma_chan == NULL) || !dma_chan->busy) {
        ret = E_INVALID_PARAM;
        goto out;
    }

    timeout = GPCDMA_BURST_COMPLETION_TIMEOUT;
    csre = gpcdma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_CSRE_0);
    csre |= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSRE, DMA_ACTIVITY, PAUSE);
    gpcdma_chan_writel(dma_chan, csre, GPCDMA_CHANNEL_CH0_CSRE_0);

    /* Wait until busy bit is de-asserted */
    do {
        if (((gpcdma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_STA_0) &
            GPCDMA_CHANNEL_CH0_STA_0_BSY_FIELD)) == 0UL) {
           break;
        }
        udelay(GPCDMA_BURST_COMPLETE_TIME);
        timeout = (timeout < GPCDMA_BURST_COMPLETE_TIME) ? 0UL :
                   (timeout - GPCDMA_BURST_COMPLETE_TIME);
    } while (timeout > 0UL);

    if (timeout == 0UL) {
        ret = E_TIMEOUT;
    }

out:
    return ret;
}

SECTION_GPCDMA_TEXT static inline void
gpcdma_resume(const struct gpcdma_channel *dma_chan)
{
    uint32_t csre;

    csre = gpcdma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_CSRE_0);
    csre &= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSRE, DMA_ACTIVITY, RESUME);
    gpcdma_chan_writel(dma_chan, csre, GPCDMA_CHANNEL_CH0_CSRE_0);
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "Approval: Bug 2687886, DR: SWE-FSP-017-SWSADR.docx")
SECTION_GPCDMA_TEXT void gpcdma_chan_irq(void *data)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_5, "Approval: Bug 200542277, DR:  SWE-FSP-024-SWSADR.docx");
    struct gpcdma_channel   *dma_chan = (struct gpcdma_channel *)fsp_c_v_ptr_to_v_ptr(data);
    struct gpcdma_buf_desc  desc;
    uint32_t                status;

    smp_lock(dma_chan->lock);

    if (!dma_chan->continuous) {
        ASSERT(irq_safe_disable(dma_chan->irq) == E_SUCCESS);
        dma_chan->busy = false;
    }

    /* Clear the DMA interrupt first */
    status = gpcdma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_STA_0);
    if (NV_DRF_VAL(GPCDMA, CHANNEL_CH0_STA, ISE_EOC, status) != 0UL) {
        gpcdma_chan_writel(dma_chan, status, GPCDMA_CHANNEL_CH0_STA_0);
    }

    if (dma_chan->synchronous) {
        UNUSED((gpcdma_port_chan_sync_end(dma_chan->id)));
    } else {
        if (dma_chan->continuous) {
            ASSERT(gpcdma_port_get_desc(dma_chan->id, &desc, 0UL,
                   true) == E_SUCCESS);
            ASSERT(gpcdma_port_send_desc_to_back(dma_chan->id,
                   &desc, 0, true) == E_SUCCESS);
            gpcdma_configure_addr_ptrs(dma_chan, desc.src_addr,
                                       desc.dst_addr);
            gpcdma_configure_hi_adr_ptr(dma_chan, desc.hi_src_addr,
                                        desc.hi_dst_addr);
            dma_chan->total_bytes = dma_chan->total_bytes +
                                            (dma_chan->words_to_xfer * 4ULL);
        }
        dma_chan->callback(dma_chan->cdata, DMA_STATUS_COMPLETE);
    }
    smp_unlock(dma_chan->lock);
}

SECTION_GPCDMA_TEXT static error_t
gpcdma_program_cycl_mode(struct gpcdma_xfer const *xfer,
                         struct gpcdma_channel *dma_chan, bool first)
{
    error_t  ret = E_SUCCESS;
    int32_t  i;
    struct   gpcdma_buf_desc desc;
    uint32_t st;

    if (gpcdma_port_get_desc(dma_chan->id, &desc, 0UL,
                             false) != E_SUCCESS) {
        ret = E_GPCDMA_QUEUE_OP_FAIL;
        goto op_failed;
    }

    if (first) {
        dma_chan->words_to_xfer = xfer->period_len >> 2UL;
        dma_chan->continuous = true;
        dma_chan->total_bytes = 0;
    } else {
        i = 20;
        do {
            st = gpcdma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_STA_0);
            if ((st & GPCDMA_CHANNEL_CH0_STA_0_BSY_FIELD) != 0UL) {
                break;
            }
            udelay(1);
            i--;
        } while (i != 0);

        if (i == 0) {
            ret = E_GPCDMA_CONT_MODE_OP_FAIL;
            goto op_failed;
        }
    }

    gpcdma_configure_addr_ptrs(dma_chan, desc.src_addr, desc.dst_addr);
    gpcdma_configure_hi_adr_ptr(dma_chan, desc.hi_src_addr, desc.hi_dst_addr);

    if (gpcdma_port_send_desc_to_back(dma_chan->id, &desc, 0UL,
                                      false) != E_SUCCESS) {
        ret = E_GPCDMA_QUEUE_OP_FAIL;
        goto op_failed;
    }

op_failed:
    return ret;
}

SECTION_GPCDMA_TEXT error_t
gpcdma_transfer(const struct gpcdma_id *id,
                uint32_t chan_num,
                struct gpcdma_xfer const *xfer)
{
    error_t     ret = E_SUCCESS;
    struct      gpcdma_channel *dma_chan;
    uint32_t    dma_mode;
    bool        src_io, dst_io;
    uint32_t    flags = 0;
    uint32_t    src_addr, dst_addr, hi_src_ptr, hi_dst_ptr;

    if (xfer == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }

    if (xfer->synchronous && xfer->continuous) {
        ret = E_GPCDMA_INVALID_XFER_MODE;
        goto out;
    }

    ret = gpcdma_get_chan_context(id, chan_num, &dma_chan);
    if (ret != E_SUCCESS) {
        goto out;
    }

    smp_lock_irqsave(dma_chan->lock, &flags);
    if (dma_chan->busy) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_CHAN_BUSY;
        goto unlock_out;
    }

    ret = gpcdma_get_xfer_dma_mode(xfer, &dma_mode, &src_io, &dst_io);
    if (ret != E_SUCCESS) {
        goto unlock_out;
    }

    ret = gpcdma_validate_xfer(dma_chan, xfer, src_io, dst_io);
    if (ret != E_SUCCESS) {
        goto unlock_out;
    }

    dma_chan->continuous = false;
    src_addr = (uint32_t)(xfer->src_addr & GPCDMA_ADDR_MASK);
    dst_addr = (uint32_t)(xfer->dst_addr & GPCDMA_ADDR_MASK);
    hi_src_ptr = (uint32_t)((xfer->src_addr >> GPCDMA_HI_ADDR_SHIFT) &
                            GPCDMA_HI_ADDR_MASK);
    hi_dst_ptr = (uint32_t)((xfer->dst_addr >> GPCDMA_HI_ADDR_SHIFT) &
                            GPCDMA_HI_ADDR_MASK);
    dma_chan->words_to_xfer = xfer->xfer_count >> 2UL;

    if (xfer->continuous) {
        ret = gpcdma_create_list(xfer, dma_chan->id);
        if (ret != E_SUCCESS) {
            ret = E_GPCDMA_QUEUE_ALLOC_FAIL;
            goto unlock_out;
        }
        ret = gpcdma_program_cycl_mode(xfer, dma_chan, true);
        if (ret != E_SUCCESS) {
            goto op_failed;
        }
    } else {
        gpcdma_configure_addr_ptrs(dma_chan, src_addr, dst_addr);
        gpcdma_configure_hi_adr_ptr(dma_chan, hi_src_ptr, hi_dst_ptr);
    }

    gpcdma_clear_error_status(dma_chan);
    gpcdma_configure_mc_sequencer(dma_chan, xfer, src_io, dst_io);
    gpcdma_configure_mmio_sequencer(dma_chan, xfer, src_io, dst_io);
    gpcdma_configure_wcount(dma_chan);
    dma_chan->busy = true;
    gpcdma_configure_csr(dma_chan, xfer, dma_mode);

    /*
     * For the continuous or cyclic mode, next buffer needs to be
     * programmed right away after CSR.
     */
    if (dma_chan->continuous) {
        ret = gpcdma_program_cycl_mode(xfer, dma_chan, false);
        if (ret != E_SUCCESS) {
            goto op_failed;
        }
    }

    ret = irq_safe_enable(dma_chan->irq);
    if (ret != E_SUCCESS) {
        goto op_failed;
    }

    /*
     * For synchronous transfers, the driver relies on the port specific
     * implementation of some kind of synchronization mechanism until the
     * DMA transfer is complete.
     */
    smp_unlock_irqrestore(dma_chan->lock, &flags);
    if (dma_chan->callback == NULL) {
        ret = gpcdma_port_chan_sync(chan_num, xfer->timeout);
        if (ret != E_SUCCESS) {
            (void)gpcdma_abort(id, chan_num);
            dma_chan->busy = false;
            goto out;
        }
    }

op_failed:
    if (ret != E_SUCCESS) {
        if (xfer->continuous) {
            gpcdma_chan_writel(dma_chan, 0UL, GPCDMA_CHANNEL_CH0_CSR_0);
            gpcdma_port_delete_queue(dma_chan->id);
        }
    }

unlock_out:
    if (ret != E_SUCCESS) {
        smp_unlock_irqrestore(dma_chan->lock, &flags);
        dma_chan->busy = false;
    }

out:
    return ret;
}

SECTION_GPCDMA_TEXT error_t
gpcdma_get_bytes_xferred(const struct gpcdma_id *id,
                         uint32_t chan_num,
                         uint64_t *xfer_count)
{
    error_t                 ret = E_SUCCESS;
    struct gpcdma_channel   *dma_chan;
    uint32_t                flags = 0;

    if (xfer_count == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }

    ret = gpcdma_get_chan_context(id, chan_num, &dma_chan);
    if (ret != E_SUCCESS) {
        goto out;
    }

    smp_lock_irqsave(dma_chan->lock, &flags);
    ret = gpcdma_pause(dma_chan);
    if (ret != E_SUCCESS) {
        goto unlock_out;
    }

    if (!dma_chan->continuous) {
        *xfer_count = gpcdma_chan_readl(dma_chan,
                                        GPCDMA_CHANNEL_CH0_DMA_WORD_STA_0);
        *xfer_count = (*xfer_count) * 4ULL;
    } else {
        *xfer_count = gpcdma_chan_readl(dma_chan,
                                        GPCDMA_CHANNEL_CH0_DMA_WORD_TRA_0);
        *xfer_count = dma_chan->words_to_xfer - *xfer_count;
        *xfer_count = (*xfer_count) * 4ULL;
        if ((UINT64_MAX - *xfer_count) < dma_chan->total_bytes) {
            ret = E_GPCDMA_INVALID_PARAM;
            goto unlock_out;
        }
        *xfer_count = (*xfer_count) + dma_chan->total_bytes;
    }

    gpcdma_resume(dma_chan);

unlock_out:
    smp_unlock_irqrestore(dma_chan->lock, &flags);

out:
    return ret;
}

SECTION_GPCDMA_TEXT error_t
gpcdma_abort(const struct gpcdma_id *id,
             uint32_t chan_num)
{
    error_t                 ret = E_SUCCESS;
    struct gpcdma_channel   *dma_chan;
    uint32_t                flags = 0;

    ret = gpcdma_get_chan_context(id, chan_num, &dma_chan);
    if (ret != E_SUCCESS) {
        goto out;
    }

    dma_chan->busy = false;
    smp_lock_irqsave(dma_chan->lock, &flags);
    if (dma_chan->busy) {
        /* Disable IRQ and then disable the channel */
        ret = irq_safe_disable(dma_chan->irq);
        if (ret != E_SUCCESS) {
            goto unlock_out;
        }
        gpcdma_chan_writel(dma_chan, 0, GPCDMA_CHANNEL_CH0_CSR_0);
        dma_chan->busy = false;

        /*
         * This function can be called internally from gpcdma_transfer()
         * for synchronous xfers, and there's no callback for those.
         */
        if (dma_chan->callback != NULL) {
            dma_chan->callback(dma_chan->cdata, DMA_STATUS_ABORTED);
        }
    } else {
        ret = E_GPCDMA_CHAN_NOT_BUSY;
    }

unlock_out:
    smp_unlock_irqrestore(dma_chan->lock, &flags);
out:
    return ret;
}

SECTION_GPCDMA_INIT_TEXT error_t
gpcdma_init(struct gpcdma_id *id)
{
    error_t                 ret = E_SUCCESS;
    uint32_t                i;
    struct gpcdma_channel   *dma_chan;

    if (id == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_NULL_PTR;
        goto out;
    }

    if ((id->conf.base_addr == 0UL) || (id->conf.base_addr == UINT32_MAX)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_GPCDMA_INVALID_PARAM;
        goto out;
    }

    if (id->inited) {
        goto out;
    }

    /* setup per channel attributes */
    for (i = 0UL; i < ARRAY_SIZE(id->channels); i += 1UL) {
        dma_chan = id->channels[i];

        dma_chan->chan_base = gpcdma_chan_base(id, i);
        dma_chan->irq = id->conf.irqs[i];
        dma_chan->id = i;
        dma_chan->busy = false;
        dma_chan->total_bytes = 0;
        smp_lock_init(dma_chan->lock);

        /* port specific per channel setup hook */
        ret = gpcdma_port_chan_setup(dma_chan->id);

        if (ret == E_SUCCESS) {
            ret = irq_safe_set_handler(dma_chan->irq, gpcdma_chan_irq, dma_chan);
        }
        if (ret != E_SUCCESS) {
            goto out;
        }

    }

    /* port specific GPCDMA controller setup hook */
    ret = gpcdma_port_init(id);
    if (ret != E_SUCCESS) {
        goto out;
    }

    /**
     * WAR for HW Bug 200667678: Disable virtualization for GPCDMA and
     * cluster DMAs out of reset.
     */
#if defined(HW_BUG_200667678) && (HW_BUG_200667678 == 1)
    gpcdma_disable_virtualization(id);
#endif

    id->inited = true;

out:
    return ret;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_8, "Approval: Bug 200534384, DR: SWE-FSP-011-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
