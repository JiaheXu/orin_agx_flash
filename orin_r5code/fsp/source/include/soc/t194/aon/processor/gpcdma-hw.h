/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef GPCDMA_HW_H
#define GPCDMA_HW_H
#define FSP__PROCESSOR__GPCDMA_HW_H                  1

/* Compiler headers */

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/macros.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */

#define GPCDMA_CHAN0    gpcdma_chan0
#define GPCDMA_CHAN1    gpcdma_chan1
#define GPCDMA_CHAN2    gpcdma_chan2
#define GPCDMA_CHAN3    gpcdma_chan3
#define GPCDMA_CHAN4    gpcdma_chan4
#define GPCDMA_CHAN5    gpcdma_chan5
#define GPCDMA_CHAN6    gpcdma_chan6
#define GPCDMA_CHAN7    gpcdma_chan7

#ifndef __ASSEMBLER__

extern struct gpcdma_id gpcdma_id_aon;
extern struct gpcdma_channel gpcdma_chan0;
extern struct gpcdma_channel gpcdma_chan1;
extern struct gpcdma_channel gpcdma_chan2;
extern struct gpcdma_channel gpcdma_chan3;
extern struct gpcdma_channel gpcdma_chan4;
extern struct gpcdma_channel gpcdma_chan5;
extern struct gpcdma_channel gpcdma_chan6;
extern struct gpcdma_channel gpcdma_chan7;

#endif

#endif
