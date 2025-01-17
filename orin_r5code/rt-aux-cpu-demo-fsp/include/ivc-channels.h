/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _IVC_CHANNELS_H_
#define _IVC_CHANNELS_H_

#include <stdbool.h>
#include <ospl/rtos-port.h>
#include <tegra-ivc.h>
#include <ivc-config.h>

/* Structures used for descirbing the IVC Channel */

struct ivc_task_id;
extern struct ivc_task_id *ivc_ccplex_task_ids[IVC_NUM_CCPLEX_CHS];

struct ivc_channel_ops {
	void *(*init)(struct ivc_task_id *id);
	int (*notify)(void *state, rtosPortBaseType *woken);
	int (*init_complete)(void *state, rtosPortBaseType *woken);
	void (*debug_enable)(rtosPortBaseType *woken, bool enable);
};

struct ivc_task_id {
	const char *name;
	struct tegra_ivc_channel *ivc_ch;
	struct ivc_channel_ops *ops;
	void *priv;
};

/*
 * Initialize the associated handling code for CCPLEX IVC channels.
 *
 * Returns:
 * 0:		OK.
 * Other:	Error.
 */
int ivc_init_channels_ccplex(void);

/*
 * Notify CCPLEX that SPE has produced data on the IVC channel
 */
int hsp_ivc_notify_ccplex(struct tegra_ivc_channel *channel, bool is_read);

#endif
