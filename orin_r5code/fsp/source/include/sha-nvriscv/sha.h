/**
 * @file sha.h
 *
 * @brief SHA library for SHA-2 and HMAC SHA-2.
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SHA_NVRISCV__SHA_H
#define SHA_NVRISCV__SHA_H
#define FSP__SHA_NVRISCV__SHA_H 1


/* Compiler headers */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>

/* Hardware headers */

/* Late FSP headers */
#include <cpu/type-conversion.h>
#include <misc/macros.h>
#include <error/common-errors.h>

/* Module-specific FSP headers */
#include <sha-nvriscv/sha-errors.h>


START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#define BYTE_TO_BIT_LENGTH(a) ((a) << 3U)
#define BIT_TO_BYTE_LENGTH(a) ((a) >> 3U)

/* Size defines for SHA variants */
#define SHA_1_BLOCK_SIZE_BYTE         (64U)
#define SHA_1_BLOCK_SIZE_BIT          BYTE_TO_BIT_LENGTH(SHA_1_BLOCK_SIZE_BYTE)
#define SHA_1_HASH_SIZE_BYTE          (20U)
#define SHA_1_HASH_SIZE_BIT           BYTE_TO_BIT_LENGTH(SHA_1_HASH_SIZE_BYTE)

#define SHA_224_BLOCK_SIZE_BYTE       (64U)
#define SHA_224_BLOCK_SIZE_BIT        BYTE_TO_BIT_LENGTH(SHA_224_BLOCK_SIZE_BYTE)
#define SHA_224_HASH_SIZE_BYTE        (28U)
#define SHA_224_HASH_SIZE_BIT         BYTE_TO_BIT_LENGTH(SHA_224_HASH_SIZE_BYTE)

#define SHA_256_BLOCK_SIZE_BYTE       (64U)
#define SHA_256_BLOCK_SIZE_BIT        BYTE_TO_BIT_LENGTH(SHA_256_BLOCK_SIZE_BYTE)
#define SHA_256_HASH_SIZE_BYTE        (32U)
#define SHA_256_HASH_SIZE_BIT         BYTE_TO_BIT_LENGTH(SHA_256_HASH_SIZE_BYTE)

#define SHA_384_BLOCK_SIZE_BYTE       (128U)
#define SHA_384_BLOCK_SIZE_BIT        BYTE_TO_BIT_LENGTH(SHA_384_BLOCK_SIZE_BYTE)
#define SHA_384_HASH_SIZE_BYTE        (48U)
#define SHA_384_HASH_SIZE_BIT         BYTE_TO_BIT_LENGTH(SHA_384_HASH_SIZE_BYTE)

#define SHA_512_BLOCK_SIZE_BYTE       (128U)
#define SHA_512_BLOCK_SIZE_BIT        BYTE_TO_BIT_LENGTH(SHA_512_BLOCK_SIZE_BYTE)
#define SHA_512_HASH_SIZE_BYTE        (64U)
#define SHA_512_HASH_SIZE_BIT         BYTE_TO_BIT_LENGTH(SHA_512_HASH_SIZE_BYTE)

#define SHA_512_224_BLOCK_SIZE_BYTE   (128U)
#define SHA_512_224_BLOCK_SIZE_BIT    BYTE_TO_BIT_LENGTH(SHA_512_224_BLOCK_SIZE_BYTE)
#define SHA_512_224_HASH_SIZE_BYTE    (28U)
#define SHA_512_224_HASH_SIZE_BIT     BYTE_TO_BIT_LENGTH(SHA_512_224_HASH_SIZE_BYTE)

#define SHA_512_256_BLOCK_SIZE_BYTE   (128U)
#define SHA_512_256_BLOCK_SIZE_BIT    BYTE_TO_BIT_LENGTH(SHA_512_256_BLOCK_SIZE_BYTE)
#define SHA_512_256_HASH_SIZE_BYTE    (32U)
#define SHA_512_256_HASH_SIZE_BIT     BYTE_TO_BIT_LENGTH(SHA_512_256_HASH_SIZE_BYTE)

/* Typedefs */
typedef enum
{
    SHA_ALGO_ID_SHA_1       = (0U),
    SHA_ALGO_ID_SHA_224,
    SHA_ALGO_ID_SHA_256,
    SHA_ALGO_ID_SHA_384,
    SHA_ALGO_ID_SHA_512,
    SHA_ALGO_ID_SHA_512_224,
    SHA_ALGO_ID_SHA_512_256,
    SHA_ALGO_ID_LAST
} SHA_ALGO_ID;

typedef struct
{
    uint8_t   src_type;
    bool      b_default_hash_iv;
    uint8_t   pad[2];
    uint32_t  dma_idx;
    uint32_t  size;
    uint64_t  addr;
} SHA_TASK_CONFIG;

typedef struct
{
    SHA_ALGO_ID algo_id;
    uint32_t    msg_size;
    uint8_t     *p_buf_out;
    uint32_t    buf_size;
    uint8_t     mutex_token;
} SHA_CONTEXT;

typedef struct
{
    SHA_CONTEXT sha_context;
    /*
     *  key_buffer: Buffer to hold key used for HMAC operation. Maximum size is
     *             SHA_512_BLOCK_SIZE_BYTE - if key is larger, user should hash first.
     */
    uint8_t key_buffer[SHA_512_BLOCK_SIZE_BYTE];

    /*
     *  key_size: Size of key in above buffer, in bytes.
     */
    uint32_t key_size;
} HMAC_CONTEXT;

/* Function Declarations */
error_t sha_acquire_mutex(uint8_t mutex_token);
error_t sha_release_mutex(uint8_t mutex_token);
error_t sha_operation_init(const SHA_CONTEXT *p_sha_context);
error_t sha_engine_halt(void);
error_t sha_insert_task(SHA_CONTEXT *p_sha_context, const SHA_TASK_CONFIG *p_task_cfg);
error_t sha_read_hash_result(const SHA_CONTEXT *p_sha_context, bool b_scrub_reg);
error_t sha_get_hash_size_byte(SHA_ALGO_ID algo_id, uint32_t *p_size);
error_t sha_get_block_size_byte(SHA_ALGO_ID algo_id, uint32_t *p_size);
error_t sha_run_single_task_common(const SHA_TASK_CONFIG *p_task_cfg, SHA_CONTEXT *p_sha_ctx);
error_t sha_hmac_operation_init(HMAC_CONTEXT *p_hmac_context);
error_t sha_hmac_insert_task(HMAC_CONTEXT *p_hmac_context, SHA_TASK_CONFIG *p_task_cfg);
error_t sha_hmac_read_hash_result(const HMAC_CONTEXT *p_hmac_context);
error_t sha_hmac_run_single_task_common(HMAC_CONTEXT *p_hmac_context, SHA_TASK_CONFIG *p_task_cfg);

/* Backward-compatibility for old code style */
#define shaAcquireMutex                     sha_acquire_mutex
#define shaReleaseMutex                     sha_release_mutex
#define shaOperationInit                    sha_operation_init
#define shaEngineHalt                       sha_engine_halt
#define shaInsertTask                       sha_insert_task
#define shaReadHashResult                   sha_read_hash_result
#define shaGetHashSizeByte                  sha_get_hash_size_byte
#define shaGetBlockSizeByte                 sha_get_block_size_byte
#define shaRunSingleTaskCommon              sha_run_single_task_common
#define shaHmacOperationInit                sha_hmac_operation_init
#define shaHmacInsertTask                   sha_hmac_insert_task
#define shaHmacReadHashResult               sha_hmac_read_hash_result
#define shaHmacRunSingleTaskCommon          sha_hmac_run_single_task_common

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")

#endif // SHA_NVRISCV__SHA_H
/*** end of file ***/
