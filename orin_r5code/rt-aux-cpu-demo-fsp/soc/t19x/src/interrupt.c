/* Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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

#include <osa/rtos-task.h>
#include <err-hook.h>
#include <irq/irqs.h>
#include <irq/safe-irqs.h>
#include <vic/vic-errors.h>

#define MAX_IRQS 64

struct irq_handler {
	void (*handler)(void*);
	void *data;
};

static struct irq_handler irq_handlers[MAX_IRQS];

void irq_handler(uint32_t irq)
{
	struct irq_handler *h;

	if (irq >= MAX_IRQS) {
		error_hook("Recieved invalid interrupt!");
		return;
	}

	h = &irq_handlers[irq];
	if (h->handler) {
		h->handler(h->data);
	} else {
		warning_hook("Spurious interrupt!");
	}
}

void irq_set_handler(uint32_t irq, irq_callback_fn routine, void *data)
{
	struct irq_handler *h;

	if (irq >= MAX_IRQS) {
		error_hook("Trying to set invalid interrupt!");
		return;
	}

	h = &irq_handlers[irq];
	rtosTaskEnterCritical();
	h->handler = routine;
	h->data = data;
	rtosTaskExitCritical();
}

error_t irq_safe_set_handler(uint32_t irq, irq_callback_fn routine, void *data)
{
	struct irq_handler *h;
	error_t err = E_SUCCESS;

	if (irq >= MAX_IRQS) {
		error_hook("Trying to set invalid interrupt!");
		err = E_VIC_INVALID_VECTOR;
		goto out;
	}

	h = &irq_handlers[irq];
	rtosTaskEnterCritical();
	h->handler = routine;
	h->data = data;
	rtosTaskExitCritical();
out:
	return err;
}
