/*
 * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef UART_MACROS_H
#define UART_MACROS_H

/* Compiler headers */
#include <stdint.h>                 // for uint32_t, int32_t, uint8_t

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>        // for NV_ADDRESS_MAP_CAR_BASE
#include <arclk_rst.h>              // for CLK_RST_CONTROLLER_....

/* Late FSP headers */
#include <reg-access/reg-access.h>
#include <delay/delay.h>
#include <misc/nvrm_drf.h>

/**
 * Read UARTx CAR register value.
 */
#define SPE_UART_CAR_READ(uart, reg)					\
		(readl(NV_ADDRESS_MAP_CAR_BASE +			\
		       CLK_RST_CONTROLLER_##reg##_##uart##_0))	\

/**
 * Write UARTx CAR register value.
 */
#define SPE_UART_CAR_WRITE(uart, reg, value)				\
	do {								\
		writel((value), NV_ADDRESS_MAP_CAR_BASE +			\
		          CLK_RST_CONTROLLER_##reg##_##uart##_0);					\
	} while (0 == 1)

/**
 * Read UARTx CAR register field value.
 */
#define SPE_UART_CAR_READ_FIELD(uart, reg, field) \
		(NV_DRF_VAL(CLK_RST_CONTROLLER, reg##_##uart, \
			   field, SPE_UART_CAR_READ(uart, reg)))

/**
 * Evaluate to true if UARTx CAR register field value matches.
 */
#define SPE_UART_CAR_IS_FIELD_DEF(uart, reg, field, def)		       \
		(SPE_UART_CAR_READ_FIELD(uart, reg, field) ==		       \
			CLK_RST_CONTROLLER_##reg##_##uart##_0_##field##_##def)

/**
 * Evaluate to true if UARTx clock is enabled.
 */
#define SPE_UART_IS_CLK_ENABLED(uart)				    \
		(SPE_UART_CAR_IS_FIELD_DEF(uart, CLK_OUT_ENB,      \
					    CLK_ENB_##uart, ENABLE))

/**
 * Evaluate to true if UARTx reset is deasserted.
 */
#define SPE_UART_IS_RESET_DEASSERTED(uart)				\
		(SPE_UART_CAR_IS_FIELD_DEF(uart, RST_DEV,		\
					    SWR_##uart##_RST, DISABLE))

/**
 * Enable UARTx clock in CAR
 */
#define SPE_UART_ENABLE_CLK(uart)					    \
	do {								    \
		const uint32_t value = NV_DRF_DEF(CLK_RST_CONTROLLER,	    \
						  CLK_OUT_ENB_##uart##_SET, \
						  SET_CLK_ENB_##uart,	    \
						  ENABLE);		    \
		writel(value, NV_ADDRESS_MAP_CAR_BASE +			    \
			CLK_RST_CONTROLLER_CLK_OUT_ENB_##uart##_SET_0);						    \
	} while (0 == 1)

/**
 * Assert UARTx reset
 */
#define SPE_UART_ASSERT_RESET(uart)					    \
	do {								    \
		const uint32_t value = NV_DRF_DEF(CLK_RST_CONTROLLER,	    \
						  RST_DEV_##uart##_SET,	    \
						  SET_SWR_##uart##_RST,	    \
						  ENABLE);		    \
		writel(value, NV_ADDRESS_MAP_CAR_BASE +			    \
		          CLK_RST_CONTROLLER_RST_DEV_##uart##_SET_0);					    \
	} while (0 == 1)

/**
 * Deassert UARTx reset
 */
#define SPE_UART_DEASSERT_RESET(uart)					    \
	do {								    \
		const uint32_t value = NV_DRF_DEF(CLK_RST_CONTROLLER,	    \
						  RST_DEV_##uart##_CLR,	    \
						  CLR_SWR_##uart##_RST,	    \
						  ENABLE);		    \
		writel(value, NV_ADDRESS_MAP_CAR_BASE +			    \
		          CLK_RST_CONTROLLER_RST_DEV_##uart##_CLR_0);					    \
	} while (0 == 1)

/**
 * Reset UARTx
 */
#define SPE_UART_RESET(uart)			\
	do {					\
		SPE_UART_ASSERT_RESET(uart);	\
		udelay(5U);		\
		SPE_UART_DEASSERT_RESET(uart);	\
	} while (0 == 1)

/**
 * Reset UARTx and configure and enable clock for UARTx in one go.
 */
#define SPE_UART_ENABLE(uart, src, div_enb, div)		  \
	do {							  \
		uint32_t val = 0U;				  \
								  \
		SPE_UART_RESET(uart);				  \
								  \
		val = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,	  \
					   CLK_SOURCE_##uart,	  \
					   uart##_CLK_SRC,	  \
					   src,			  \
					   val);		  \
		val = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,	  \
					   CLK_SOURCE_##uart,	  \
					   uart##_DIV_ENB,	  \
					   div_enb,		  \
					   val);		  \
		val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,	  \
					   CLK_SOURCE_##uart,	  \
					   uart##_CLK_DIVISOR,	  \
					   (div),		  \
					   val);		  \
		SPE_UART_CAR_WRITE(uart, CLK_SOURCE, val);	  \
								  \
		SPE_UART_ENABLE_CLK(uart);			  \
	} while (0 == 1)

/* vim: set ts=8 noexpandtab: */

#endif /* UART_MACROS_H */
