/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
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

/* Note: Follow doc/i2c-app.md to successfully run this demo app */

#include <stdio.h>
#include <string.h>
#include <osa/rtos-task.h>
#include <processor/i2c-tegra-hw.h>
#include <misc/bitops.h>
#include <err-hook.h>
#include <i2c/i2c.h>
#include "config.h"
#include "i2c-app.h"
#include "i2c-app-priv.h"

#define I2C_TEST_BUS_CLKRATE	100000
#define I2C_TEST_RETRIES	5
#define I2C_TEST_DELAY		10000

static void i2c_app_task(void *pvParameters)
{
	int count;
	(void)pvParameters;
	struct i2c_handle hi2c;

	error_t ret = i2c_controller_init(I2C_AON_BUS_NUM, &hi2c);

	if (ret != E_SUCCESS) {
		error_hook("Couldn't initialize I2C\r\n");
		goto done;
	}
	for (count = 0; count < I2C_TEST_RETRIES; count++) {
		i2c_test(&hi2c);
		rtosTaskDelay(I2C_TEST_DELAY);
	}
done:
	rtosTaskDelete(NULL);
}

void i2c_app_init(void)
{
	int ret;
	rtosTaskParameters i2c_app_task_params = {
		.pvTaskCode = &i2c_app_task,
		.pcTaskName = (char *)"i2ctest",
		.uxPriority = rtosIDLE_PRIORITY,
		.pvParameters = NULL,
		.uxStackDepthBytes = 512,
	};

	ret = rtosTaskCreate(&i2c_app_task_params, NULL);
	if (ret != rtosPASS)
		error_hook("xTaskCreate for i2c_app_task failed\r\n");
}
