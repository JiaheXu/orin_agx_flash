/*
 * Copyright (c) 2018-2020 NVIDIA CORPORATION.  All rights reserved.
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
#ifndef LOGGER__LOGGER_HEADERS_H
#define LOGGER__LOGGER_HEADERS_H
#define FSP__LOGGER__LOGGER_HEADERS_H                   1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */
#include <logger-config.h>                              /* Immune from CT_ASSERT protection */

/*
 * Log block header that is written to DRAM, the indicated number of
 * log entries immediately follows the header.
 */
typedef struct log_block_hdr_s {
    uint64_t            start_time;     // time at start of block
    uint16_t            n_entries;      // number of log entries in block
    uint16_t            reserved_1;     // reserved field
    uint32_t            reserved_2;     // reserved field
    uint8_t             pad[48];        // get block header to be 64 bytes
} log_block_hdr_t;

/*
 * Header for the overall log in memory.
 *
 * In this particular implementation, the log buffer is a circular
 * buffer that is managed by head and tail offsets (from the start
 * of the buffer...assuming that the header is at the start of the
 * buffer).
 *
 * Each field in the header is aligned to the start of a cache line
 * (if the two endpoints have different cache line sizes, then the
 * value chosen should be the larger of the two) to prevent potential
 * corruption of the non-updated offset (e.g. if both head and tail
 * offsets were in the same cache line, updating one of them could
 * result in the other being overwitten with a stale value when the
 * cacheline is written back to DRAM).
 */
typedef struct log_header_s {
    uint32_t            block_size;
    uint8_t             header_align[LOG_CACHE_LINE_SIZE - sizeof(uint32_t)];
    uint32_t            head_offset;
    uint8_t             tail_align[LOG_CACHE_LINE_SIZE - sizeof(uint32_t)];
    uint32_t            tail_offset;
    uint8_t             pad[LOG_CACHE_LINE_SIZE - sizeof(uint32_t)];
} log_header_t;

#endif
