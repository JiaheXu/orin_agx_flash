/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION. All rights reserved.
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

#include <stdio.h>
#include <reg-access/reg-access.h>
#include <debug/debug.h>
#include <err-hook.h>
#include <tcu.h>
#include "debug_init.h"
#include "config.h"

void debug_early_init(void)
{
	tcu_preinit();
}

void debug_init(uint32_t reg)
{
	uint32_t scratch_reg = readl(reg);
	int ret = 0;
	uint32_t uart_port = 0;
	if ((scratch_reg & SCRATCH_MAGIC_MASK_COMB) == SCRATCH_MAGIC_VAL_COMB) {
		uart_port = scratch_reg & SCRATCH_UART_MASK_COMB;
		switch (uart_port) {
		case 2:
			set_tcu_status(true);
			tcu_tasks_init();
			ret = tcu_init(uart_port);
			printf("init uartc for combined uart\r\n");
			break;
		default:
			printf("unknown scratch value (%d) for combined uart\r\n", (int)scratch_reg & SCRATCH_UART_MASK_COMB);
			break;
		}
		if (ret)
			error_hook("tcu_init() failed");
	}
}
