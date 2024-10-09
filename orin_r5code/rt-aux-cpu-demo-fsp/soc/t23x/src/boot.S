@ Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
@
@ Redistribution and use in source and binary forms, with or without
@ modification, are permitted provided that the following conditions
@ are met:
@  * Redistributions of source code must retain the above copyright
@    notice, this list of conditions and the following disclaimer.
@  * Redistributions in binary form must reproduce the above copyright
@    notice, this list of conditions and the following disclaimer in the
@    documentation and/or other materials provided with the distribution.
@  * Neither the name of NVIDIA CORPORATION nor the names of its
@    contributors may be used to endorse or promote products derived
@    from this software without specific prior written permission.
@
@ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
@ EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@ IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
@ PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
@ CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
@ EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
@ PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
@ PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
@ OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@ (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <soc-common/hw-const.h>
#include <address_map_new.h>
#include <arclk_rst.h>
#include <araoncluster_evp.h>
#include <arclk_rst.h>
#include <spe-map.h>

.extern main
.extern _Undefined_Instruction_Handler
.extern _Data_Abort_Handler
.extern _Prefetch_Abort_Handler
.extern vPortYieldProcessor
.extern cache_enable

.extern _svcstack
.extern _fiqstack
.extern _irqstack
.extern _abtstack
.extern _undstack

.section .vectors
    B   _reset_handler                 @Reset
    B   _Undefined_Instruction_Handler @Undefined_Handler
    B   vPortYieldProcessor            @SVC_Handler
    B   _Prefetch_Abort_Handler        @Prefetch_Handler
    B   _Data_Abort_Handler            @Abort_Handler
    B   _dummy_handler                 @Reserved
    B   _irq_handler                   @IRQ_Handler
    B   _irq_handler      	       @FIQ_Handler

.section .text
@ SCTLR.VE should be set to 1 and should not get here
_irq_handler:
    B  _irq_handler

_dummy_handler:
    B  _dummy_handler

_reset_handler:
@==================================================================
@ Copy TCM overlay area to BTCM using DMA. This also zero initializes
@ the TCM BSS sections and zero-fills the rest of the TCM with zero.
@
@ relocate_memcpy_dma(btcm_base, mem_base, tcm_size);
@==================================================================
    LDR     R0, =RUN_ADDR
    LDR     R1, =LOAD_ADDR
    LDR     R2, =NV_ADDRESS_MAP_AON_BTCM_SIZE
    BL      relocate_memcpy_dma

@==================================================================
@ Execution continues here after DMA relocation is complete. This
@ code transitions execution from the LOAD_ADDR space to the
@ RUN_ADDR space.  R0 is loaded with address in the RUN_ADDR space.
@ After setting PC=R0 execution continues at the run-address.
@==================================================================
    MOVW    R0, #:lower16:relocation_label
    MOVT    R0, #:upper16:relocation_label
    MOV     PC, R0

@==================================================================
@ Disable MPU and cache in case it was left enabled from an earlier run
@ This does not need to be done from a cold reset
@==================================================================
relocation_label:
    MRC     p15, 0, r0, c1, c0, 0       @ Read CP15 Control Register
    BIC     r0, r0, #0x05               @ Disable MPU (M bit) and data cache (C bit)
    BIC     r0, r0, #0x1000             @ Disable instruction cache (I bit)
    DSB                                 @ Ensure all previous loads/stores have completed
    MCR     p15, 0, r0, c1, c0, 0       @ Write CP15 Control Register
    ISB                                 @ Ensure subsequent insts execute wrt new MPU settings

@==================================================================
@ Setup the various exception vectors
@
@ Only the reset vector is set up by the bootloader
@==================================================================
	LDR	R1, =NV_ADDRESS_MAP_AON_ATCM_CFG_EVP_BASE
	LDR	R0, =_Undefined_Instruction_Handler
	STR	R0, [R1, #AONCLUSTER_EVP_UNDEF_ADDR_0]
	LDR	R0, =vPortYieldProcessor
	STR	R0, [R1, #AONCLUSTER_EVP_SWI_ADDR_0]
	LDR	R0, =_Prefetch_Abort_Handler
	STR	R0, [R1, #AONCLUSTER_EVP_PREFETCH_ABORT_ADDR_0]
	LDR	R0, =_Data_Abort_Handler
	STR	R0, [R1, #AONCLUSTER_EVP_DATA_ABORT_ADDR_0]
	LDR	R0, =_irq_handler
	STR	R0, [R1, #AONCLUSTER_EVP_IRQ_ADDR_0]
	LDR	R0, =_irq_handler
	STR	R0, [R1, #AONCLUSTER_EVP_FIQ_ADDR_0]

@==================================================================
@ Init Stack pointer
@==================================================================
    MSR     cpsr_c, #0xD1       @FIQ
    LDR     sp, =_fiqstack
    MSR     cpsr_c, #0xD2       @IRQ
    LDR     sp, =_irqstack
    MSR     cpsr_c, #0xD7       @ABT
    LDR     sp, =_abtstack
    MSR     cpsr_c, #0xDB       @UND
    LDR     sp, =_undstack
    MSR     cpsr_c, #0xD3       @SVC
    LDR     sp, =_svcstack

@==================================================================
@ Setup Frame Pointers. Zero out ARM and Thumb frame pointers.
@==================================================================
    MOV     a2, #0
    MOV     fp, a2
    MOV     r7, a2

@==================================================================
@ Enable Branch prediction
@==================================================================

    MRC     p15, 0, r0, c1, c0, 1       @ Read ACTLR
    BIC     r0, r0, #(0x1 << 17)        @ Clear RSDIS bit 17 to enable return stack
    BIC     r0, r0, #(0x1 << 16)        @ Clear BP bit 15 and BP bit 16:
    BIC     r0, r0, #(0x1 << 15)        @ Normal operation, BP is taken from the global history table.
    MCR     p15, 0, r0, c1, c0, 1       @ Write ACTLR

@==================================================================
@ Enable VFP
@==================================================================

    MRC     p15, 0, r0, c1, c0, 2
    ORR     r0, r0, #(0xF << 20)
    MCR     p15, 0, r0, c1, c0, 2
    ISB
    MOV     r0, #0x40000000
    VMSR    FPEXC, r0

@==================================================================
@ Configure VIC to provide handler addr for IRQ
@==================================================================
    MRC     p15, 0, r0, c1, c0, 0       @ read CP15 register 1
    ORR     r0, r0, #(0x1 << 24)        @ Configure VIC to provide handler addr for IRQ
    MCR     p15, 0, r0, c1, c0, 0       @ write CP15 register 1

@==================================================================
@ Enable asynchronous aborts
@==================================================================
    CPSIE A

@==================================================================
@ Enable Caches
@==================================================================
    BL      cache_enable

@==================================================================
@ Clear BSS
@==================================================================
    LDR	    R1, _sbss_val
    LDR	    R2, _ebss_val
    SUBS    R2, R2, R1 /* END-START = LENGTH */
    BEQ	    _clear_bss_end
    MOV	    R0, #0
_clear_bss_loop:
    STR	    R0, [R1], #4
    SUBS    R2, R2, #4
    BGT	    _clear_bss_loop
_clear_bss_end:

@==================================================================
@ Jump to Reset Handler
@==================================================================
    BLX     main

_loop:
   B	    _loop

_sbss_val:
	.word	_sbss
_ebss_val:
	.word	_ebss

