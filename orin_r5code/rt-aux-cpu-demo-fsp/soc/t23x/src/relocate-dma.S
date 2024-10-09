/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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

#include <soc-common/hw-const.h>
#include <asm/asm.h>
#include <address_map_new.h>
#include <argpcdma_flv_8ch_non_safe.h>

.syntax unified
.arm
.align 2

/**
 * Copy a region of memory with DMA.
 *
 * This function must not use stack since it is used from very early boot to
 * copy the TCM region from DRAM. It is also set to use 16-word burst size
 * so it should only be used to copy regions that are aligned to 64 bytes.
 *   - R0 = Destination address
 *   - R1 = Source address
 *   - R2 = Number of bytes
 *
 * void relocate_memcpy_dma(void *dst, const void *src, size_t size);
 */

#define CH_REG(_reg_) (GPCDMA_FLV_8CH_NON_SAFE_CHANNEL_CH0_ ## _reg_ ##_0 - GPCDMA_FLV_8CH_NON_SAFE_CHANNEL_CH0_CSR_0)
#define CH_DRF_NUM(_r, _f, _v) (_v << GPCDMA_FLV_8CH_NON_SAFE_CHANNEL_CH0_ ## _r ## _0_ ## _f ## _SHIFT)
#define CH_DRF_DEF(_r, _f, _v) CH_DRF_NUM(_r, _f, GPCDMA_FLV_8CH_NON_SAFE_CHANNEL_CH0_ ## _r ## _0_ ## _f ## _ ## _v)
#define CH_DRF_FIELD(_r, _f) GPCDMA_FLV_8CH_NON_SAFE_CHANNEL_CH0_ ## _r ## _0_ ## _f ##_FIELD

FUNCTION(relocate_memcpy_dma)

    r_dst   .req r0
    r_src   .req r1
    r_size  .req r2

    /* Copy TCM overlay from DRAM using channel 0 */
    r_ch0   .req r3

	/* Disable virtualization */
	ldr	r4, =0x0
	ldr	r3, =(NV_ADDRESS_MAP_AON_DMA_BASE + \
		      GPCDMA_FLV_8CH_NON_SAFE_COMMON_DMA_CHAN_VIRTUALIZATION_ENABLE_0)
	str	r4, [r3]

    /* load ch0 CSR into r3 */
    ldr     r_ch0, gpcdma_ch0_base

    /* check for BCOUNT word aligned (word_count - 1) */
    lsrs r_size, r_size, #2
    bxeq lr
    sub  r_size, r_size, #1

    /* Set destination address */
    str r_dst, [r_ch0, #CH_REG(DST_PTR)]

	/* clear error status */
	ldr	r4, =0xffffffff
	str	r4, [r_ch0, #CH_REG(ERR_STA)]

    /* Set source address */
    str r_src, [r_ch0, #CH_REG(SRC_PTR)]

    /* Clear the high addresses */
    mov r4, #0
    str r4, [r_ch0, #CH_REG(HI_ADR_PTR)]

    /* Write settings suitable for MEM2MEM in MC_SEQ */
    ldr r4, ch0_mc_seq
    str r4, [r_ch0, #CH_REG(MC_SEQ)]

    /* Set BCOUNT */
    str r_size, [r_ch0, #CH_REG(WCOUNT)]

    /* Write MEM2MEM transfer settings and enable */
    ldr r4, ch0_csr
    str r4, [r_ch0, #CH_REG(CSR)]

    /* spin waiting for the DMA to complete */
wait_dma:
    /* Load channel status register value */
    ldr r4, [r_ch0, #CH_REG(STA)]
    /* Compare busy field value against active flag */
    and r4, #CH_DRF_FIELD(STA, BSY)
    cmp r4, #0
    bne wait_dma

    .unreq r_ch0

    /* Done */
    bx  lr

.ltorg
END_FUNCTION(relocate_memcpy_dma)

gpcdma_ch0_base:
    .word NV_ADDRESS_MAP_AON_DMA_BASE + GPCDMA_FLV_8CH_NON_SAFE_CHANNEL_CH0_CSR_0
ch0_csr:
    .word CH_DRF_DEF(CSR, ENB, ENABLE) | \
        CH_DRF_DEF(CSR, ONCE, SINGLE_BLOCK) | \
        CH_DRF_DEF(CSR, IE_EOC, DISABLE) | \
        CH_DRF_DEF(CSR, DMA_MODE, MEM2MEM) | \
        CH_DRF_DEF(CSR, FC_MODE, NO_MMIO) | \
        CH_DRF_DEF(CSR, IRQ_MASK, ENABLE) | \
        CH_DRF_NUM(CSR, WEIGHT, 1)
ch0_mc_seq:
    .word CH_DRF_DEF(MC_SEQ, MC_DATA_SWAP, DISABLE) | \
        CH_DRF_NUM(MC_SEQ, MC_REQ_CNT, 1) | \
        CH_DRF_DEF(MC_SEQ, MC_BURST, DMA_BURST_16WORDS) | \
        CH_DRF_DEF(MC_SEQ, MC_ADDR_WRAP0, NO_WRAP) | \
        CH_DRF_DEF(MC_SEQ, MC_ADDR_WRAP1, NO_WRAP)
