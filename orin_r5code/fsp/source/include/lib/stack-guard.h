/* Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef LIB__STACK_GUARD_H
#define LIB__STACK_GUARD_H
#define FSP__LIB__STACK_GUARD_H                         1

#include <stdint.h>
#include <misc/macros.h>         // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                CERTC, DEVIATE, DCL37_C, "Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx")
/**
 * @file stack-guard.h
 * @brief Definition of stack guard as required by compiler
 */

/**
 * @brief Initialize Stack Guard Variable
 *
 * This function will initialize the stack guard variable with the
 * supplied value.  This allows the application a means to initialize
 * the stack check variable with a (hopefully) randomized value.
 */
void stack_chk_init(uintptr_t check_value);

/**
 * @brief Stack Check Failure
 *
 * Called when the guard value on the stack has been overwritten.
 * This function should only be called by compiler generated code.  No
 * application code should call this function.
 *
 * @return Does not return
 */
INLINE_RFD(MISRA, DEVIATE, Rule_21_2, "Approval: JIRA TID-1063, DR: SWE-FSP-053-SWSADR.docx")
void __stack_chk_fail(void);

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              CERTC, DEVIATE, DCL37_C, "Approval:  Bug 2731179, DR: SWE-FSP-036-SWSADR.docx")
#endif
