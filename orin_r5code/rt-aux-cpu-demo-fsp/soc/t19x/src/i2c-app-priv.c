/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
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
#include <string.h>
#include <misc/bitops.h>
#include <i2c/i2c.h>
#include <i2c/i2c-tegra.h>
#include "config.h"
#include "i2c-app-priv.h"

#define I2C_TEST_CLIENT_ADDR	0x1a
#define I2C_XFER_TIMEOUT	500000

void i2c_test(struct i2c_handle *hi2c)
{
	error_t ret;

	uint8_t data_to_read[] = {0xff, 0xff};
	uint8_t data_to_compare[] = {0x63, 0x11};
	uint8_t id_reg[] = {0x00, 0xff};

	struct i2c_xfer_msg xfers[] = {
		{
			.dev_addr = I2C_TEST_CLIENT_ADDR,
			.xfer_flags = 0,
			.pwbuf = id_reg,
			.buf_len = ARRAY_SIZE(id_reg),
		},
		{
			.dev_addr = I2C_TEST_CLIENT_ADDR,
			.xfer_flags = I2C_XFER_FLAG_RD,
			.prbuf = data_to_read,
			.buf_len = ARRAY_SIZE(data_to_read),
		},
	};
	ret = i2c_do_transfer(hi2c, xfers, ARRAY_SIZE(xfers), I2C_XFER_TIMEOUT);

	if (ret == E_SUCCESS){
		if (!memcmp(data_to_read, data_to_compare, 2)){
			printf("I2C test successful\r\n");
			return;
		}
	}
	printf("I2C failed reading id reg\r\n");
}
