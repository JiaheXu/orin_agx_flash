/**
 * @file sha.c
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

/* Compiler headers */
#include <stdint.h>
#include <stddef.h>

/* Early FSP headers */
#include <misc/ct-assert.h>
#include <error/common-errors.h>


/* Hardware headers */

/* Late FSP headers */
#include <cpu/type-conversion.h>
#include <misc/macros.h>
#include <cpu/csr.h>
#include <cpu/io.h>
#include <cpu/io_dio.h>
#include <cpu/io_local.h>
#include <cpu/ptimer.h>
#include <libc-lite/libc.h>
#include <misc/nvmisc_drf.h>
#include <misc/bitops.h>

/* Module-specific FSP headers */
#include <sha-nvriscv/sha.h>

/*
 * Compile-time check for FSP header files
 *   Each FSP header file contains a signature unique to that file, and the
 *   FSP project.  The CT_ASSERT macro (contained in misc/ct-assert.h) can
 *   check for this signature.  If it does not exist, then the build will
 *   abort.
 *
 *   This is a trap for projects which have their own include files of the
 *   same names, but different contents.  This trap ensures that only the
 *   files from the FSP project, are built into the FSP source code.
 */

START_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
                MISRA, DEVIATE, Rule_8_7, "Approval: TODO, DR: TODO")

HEADER_CHECK(FSP__ERROR__COMMON_ERRORS_H)
HEADER_CHECK(FSP__MISC__MACROS_H)
HEADER_CHECK(FSP__CPU__TYPE_CONVERSION_H)
HEADER_CHECK(FSP__CPU__IO_H)
HEADER_CHECK(FSP__CPU__IO_DIO_H)
HEADER_CHECK(FSP__LIBC__LIBC_H)
HEADER_CHECK(FSP__MISC__NVMISC_DRF_H)
HEADER_CHECK(FSP__MISC__BITOPS_H)
HEADER_CHECK(FSP__SHA_NVRISCV__SHA_H)
HEADER_CHECK(FSP__SHA_NVRISCV__SHA_ERRORS_H)

// sync with PTIMER, currently 100ms is safe for the longest message
#define SHA_ENGINE_IDLE_TIMEOUT_NS          (100000000ULL)
// sync with PTIMER, base on idle timeout, HW recommend SHA_ENGINE_IDLE_TIMEOUT_NS + 20ns
#define SHA_ENGINE_SW_RESET_TIMEOUT_NS      ((SHA_ENGINE_IDLE_TIMEOUT_NS) + 20ULL)
#define SHA_ENGINE_HALT_TIMEOUT_NS          (SHA_ENGINE_IDLE_TIMEOUT_NS)
#define SHA_MSG_BYTES_MAX                   (BIT_TO_BYTE_LENGTH(UINT32_MAX))
#define HMAC_SHA_IPAD_MASK                  ((uint8_t)0x36)
#define HMAC_SHA_OPAD_MASK                  ((uint8_t)0x5C)
#define HMAC_SHA_MAX_BLOCK_SIZE_BYTE        SHA_512_BLOCK_SIZE_BYTE
#define HMAC_SHA_MAX_HASH_SIZE_BYTE         SHA_512_HASH_SIZE_BYTE

//TODO: Remove this when moving to nvriscv and use byteswap function from utils
#define NV_BYTESWAP32(a) (      \
    (((a) & 0xff000000U) >> 24U) |  \
    (((a) & 0x00ff0000U) >> 8U)  |  \
    (((a) & 0x0000ff00U) << 8U)  |  \
    (((a) & 0x000000ffU) << 24U) )

/* Private Function Declaration */
static void     sha_set_init_vector(SHA_ALGO_ID algo_id);
static error_t  sha_wait_engine_idle(uint64_t time_out_ns);
static error_t  sha_engine_soft_reset(uint64_t time_out_ns);
static error_t  sha_wait_for_busy(uint64_t time_out_ns);
static error_t  sha_get_config_encode_mode(SHA_ALGO_ID algo_id, uint8_t* p_mode);
static bool     sha_hmac_is_sha_algo_supported(SHA_ALGO_ID algo_id);
static bool     sha_task_config_is_valid
(
    const SHA_CONTEXT *p_sha_context,
    const SHA_TASK_CONFIG *p_task_cfg
);
static inline uint64_t pointer_to_address(uint8_t* buf);
static inline bool timeout_is_hit(uint64_t start_time, uint64_t timeout);
static inline error_t sha_get_result_word(uint32_t register_index, uint8_t* result, bool b_scrub_reg);


/*!
 * @brief Execute SHA engine halt
 *
 * @return E_SUCCESS if halt engine successfully
 *         E_SHA_ENG_ERROR if failed.
 *
 */
error_t
sha_engine_halt
(
    void
)
{
    uint32_t reg = 0;
    error_t status = E_SUCCESS;

    reg = localRead(NV_PRGNLCL_FALCON_SHA_OPERATION);
    reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_OPERATION, _HALT, _ENABLE, reg);
    localWrite(NV_PRGNLCL_FALCON_SHA_OPERATION, reg);

    reg = localRead(NV_PRGNLCL_FALCON_SHA_STATUS);

    if (!FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA, _STATUS_STATE, _HALTED, reg))
    {
        status = E_SHA_ENG_ERROR;
    }

    return status;
}

/*!
 * @brief Acquire SHA mutex
 *
 * @param[in] mutex_token The mutex_token allocated by client to acquire SHA mutex.
 *
 * @return E_SUCCESS if acquire or release mutex successfuly.
 *         E_SHA_MUTEX_ACQUIRE_FAILED if failed.
 *
 */
error_t
sha_acquire_mutex
(
    uint8_t mutex_token
)
{
    uint32_t reg = 0;
    bool locked = false;
    error_t status = E_SUCCESS;

    if (mutex_token == 0U)
    {
        status = E_INVALID_PARAM;
    }

    if (status == E_SUCCESS)
    {
        reg = localRead(NV_PRGNLCL_FALCON_SHA_MUTEX_STATUS);
        locked = FLD_TEST_DRF_NUM(_PRGNLCL, _FALCON_SHA_MUTEX_STATUS, _LOCKED, mutex_token, reg);
        if (locked)
        {
            status = E_SHA_MUTEX_ACQUIRE_FAILED; // mutex blocked
        }
    }

    if (status == E_SUCCESS)
    {
        reg = DRF_NUM(_PRGNLCL, _FALCON_SHA_MUTEX, _VAL, mutex_token);
        localWrite(NV_PRGNLCL_FALCON_SHA_MUTEX , reg);
        reg = localRead(NV_PRGNLCL_FALCON_SHA_MUTEX_STATUS);
        locked = FLD_TEST_DRF_NUM(_PRGNLCL, _FALCON_SHA_MUTEX_STATUS, _LOCKED, mutex_token, reg);

        status = locked ? E_SUCCESS : E_SHA_MUTEX_ACQUIRE_FAILED;
    }

    return status;
}

/*!
 * @brief Release SHA mutex
 *
 * @param[in] mutex_token The mutex_token allocated by client to release SHA mutex.
 *
 * @return E_SUCCESS if acquire or release mutex successfuly.
 *         E_SHA_MUTEX_ACQUIRE_FAILED if failed.
 *
 */
error_t
sha_release_mutex
(
    uint8_t mutex_token
)
{
    uint32_t reg = 0;
    bool unlocked = false;
    error_t status = E_SUCCESS;

    if (mutex_token == 0U)
    {
        status = E_INVALID_PARAM;
    }
    if (status == E_SUCCESS)
    {
        reg = DRF_NUM(_PRGNLCL, _FALCON_SHA_MUTEX_RELEASE, _VAL, mutex_token);
        localWrite(NV_PRGNLCL_FALCON_SHA_MUTEX_RELEASE, reg);
        reg = localRead(NV_PRGNLCL_FALCON_SHA_MUTEX_STATUS);
        unlocked = FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA_MUTEX_STATUS, _LOCKED, _INIT, reg);

        status = unlocked ? E_SUCCESS : E_SHA_MUTEX_RELEASE_FAILED;
    }

    return status;
}

/*!
 * @brief To get SHA hash size in byte per SHA algorithm id
 *
 * @param[in]  algo_id SHA algorithm id
 * @param[out] *p_size The pointer to save return hash size
 *
 * @return E_SUCCESS if get hash size successfully.
 *         E_INVALID_PARAM if failed.
 *
 */
error_t
sha_get_hash_size_byte
(
    SHA_ALGO_ID  algo_id,
    uint32_t     *p_size
)
{
    error_t status = E_SUCCESS;
    if (p_size == NULL)
    {
        status = E_INVALID_PARAM;
    }
    if (status == E_SUCCESS)
    {
        switch (algo_id)
        {
            case SHA_ALGO_ID_SHA_1:
                *p_size = SHA_1_HASH_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_224:
                *p_size = SHA_224_HASH_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_256:
                *p_size = SHA_256_HASH_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_384:
                *p_size = SHA_384_HASH_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_512:
                *p_size = SHA_512_HASH_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_512_224:
                *p_size = SHA_512_224_HASH_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_512_256:
                *p_size = SHA_512_256_HASH_SIZE_BYTE;
                break;

            default:
                status = E_INVALID_PARAM;
                break;
        }
    }
    return status;
}

/*!
 * @brief To get SHA block size in byte per SHA algorithm id
 *
 * @param[in]  algo_id SHA algorithm id
 * @param[out] *p_size The pointer to save return block size
 *
 * @return E_SUCCESS if get hash size successfully.
 *         E_INVALID_PARAM if failed.
 *
 */
error_t
sha_get_block_size_byte
(
    SHA_ALGO_ID  algo_id,
    uint32_t     *p_size
)
{
    error_t status = E_SUCCESS;
    if (p_size == NULL)
    {
        status = E_INVALID_PARAM;
    }
    if (status == E_SUCCESS)
    {
        switch (algo_id)
        {
            case SHA_ALGO_ID_SHA_1:
                *p_size = SHA_1_BLOCK_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_224:
                *p_size = SHA_224_BLOCK_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_256:
                *p_size = SHA_256_BLOCK_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_384:
                *p_size = SHA_384_BLOCK_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_512:
                *p_size = SHA_512_BLOCK_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_512_224:
                *p_size = SHA_512_224_BLOCK_SIZE_BYTE;
                break;

            case SHA_ALGO_ID_SHA_512_256:
                *p_size = SHA_512_256_BLOCK_SIZE_BYTE;
                break;

            default:
                status = E_INVALID_PARAM;
                break;
        }
    }
    return status;
}

/*!
 * @brief Initilaize SHA operation.
 *
 * @param[in] p_sha_context The designated SHA context
 *
 * @return E_SUCCESS if initialization is successfuly; otherwise NVRV_ERR_SHA_XXX return.
 *
 * Please refer to 17. Programming Guidelines in SHA IAS to get more detail description.
 */
error_t
sha_operation_init
(
    const SHA_CONTEXT      *p_sha_context
)
{
    uint32_t reg;
    uint32_t msg_size, i;
    uint8_t  enc_mode = 0;
    error_t status = E_SUCCESS;

    if (p_sha_context == NULL)
    {
        status = E_INVALID_PARAM;
    }
    if (status == E_SUCCESS)
    {
        msg_size = p_sha_context->msg_size;

        if (msg_size > SHA_MSG_BYTES_MAX)
        {
            status = E_INVALID_PARAM;
        }
    }
    if (status == E_SUCCESS)
    {
        /* Issue HALT when SHA state is IDLE or HALTED will be dropped silently.
         * So we only halt SHA engine when it's at busy state.
         */
        reg = localRead(NV_PRGNLCL_FALCON_SHA_STATUS);

        if (FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA, _STATUS_STATE, _BUSY, reg))
        {
            status = sha_wait_for_busy(SHA_ENGINE_HALT_TIMEOUT_NS);
        }
    }

    if (status == E_SUCCESS)
    {
        status = sha_engine_soft_reset(SHA_ENGINE_SW_RESET_TIMEOUT_NS);
    }

    if (status == E_SUCCESS)
    {
        status = sha_get_config_encode_mode(p_sha_context->algo_id, &enc_mode);
    }

    if (status == E_SUCCESS)
    {
        reg = DRF_NUM(_PRGNLCL, _FALCON_SHA_CONFIG, _ENC_MODE, enc_mode) |
                    DRF_DEF(_PRGNLCL, _FALCON_SHA_CONFIG, _ENC_ALG, _SHA) |
                    DRF_DEF(_PRGNLCL, _FALCON_SHA_CONFIG, _DST, _HASH_REG);
        localWrite(NV_PRGNLCL_FALCON_SHA_CONFIG, reg);

        /*
         * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         * SW reset should clear NV_CSEC_FALCON_SHA_MSG_LENGTH(i) and NV_CSEC_FALCON_SHA_MSG_LEFT(i)
         * But we observed NV_CSEC_FALCON_SHA_MSG_LEFT(i) still has garbage values.
         * So we still clear these all registers to make sure operation able to work correctly.
         * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         *
         * TODO: Reset SHA_HASH_RESULT registers after soft reset as these are not cleared by it.
         *       Add bool arument to check for chaining tasks before clearing results.
         *
         */
        for (i = 0; i < NV_PRGNLCL_FALCON_SHA_MSG_LENGTH__SIZE_1; i++)
        {
            localWrite(NV_PRGNLCL_FALCON_SHA_MSG_LENGTH(i), 0);
        }

        for (i = 0; i < NV_PRGNLCL_FALCON_SHA_MSG_LEFT__SIZE_1; i++)
        {
            localWrite(NV_PRGNLCL_FALCON_SHA_MSG_LEFT(i), 0);
        }

        /*
         * In NIST spec, the MIN unit for SHA message is bit, but in real case, we just use byte unit.
         * Although HW provide 4 registers for message length and left message length separately,
         * but currently SW just uses one 32-bit register for SHA length(0x1FFFFFFF bytes)
         * In real case usage, openSSL supports byte unit as well.
         * And it should be enough for current requirement.
         */

        // Set bit length
        localWrite(NV_PRGNLCL_FALCON_SHA_MSG_LENGTH(0U), BYTE_TO_BIT_LENGTH(msg_size));
    }
    return status;
}

/*!
 * @brief Insert SHA task configuration.
 *
 * @param[in] p_sha_context The designated SHA context
 * @param[in] p_task_cfg The designated SHA task config
 *
 * @return E_SUCCESS if task config is inserted successfully; otherwise return NVRV_ERR_SHA_XXX.
 *
 * Please refer to 17. Programming Guidelines in SHA IAS to get more detail
 *
 */
error_t
sha_insert_task
(
    SHA_CONTEXT *p_sha_context,
    const SHA_TASK_CONFIG *p_task_cfg
)
{
    uint32_t reg, tmp;
    SHA_ALGO_ID  algo_id;
    error_t status = E_SUCCESS;
    uint64_t msg_addr;
    uint32_t left_msg_length = 0;

    if ((p_sha_context == NULL) ||
        (p_task_cfg == NULL))
    {
        status = E_INVALID_PARAM;
    }
    if (status == E_SUCCESS)
    {
        if (sha_task_config_is_valid(p_sha_context, p_task_cfg))
        {
            algo_id = p_sha_context->algo_id;
            left_msg_length = p_sha_context->msg_size;
        }
        else
        {
            status = E_INVALID_PARAM;
        }
    }
    if (status == E_SUCCESS)
    {
        status = sha_wait_engine_idle(SHA_ENGINE_IDLE_TIMEOUT_NS);
    }

    if (status == E_SUCCESS)
    {
        // For each task, we need to set length for left message, bit unit.
        localWrite(NV_PRGNLCL_FALCON_SHA_MSG_LEFT(0U), BYTE_TO_BIT_LENGTH(left_msg_length));

        //TODO: Add validation for all things from p_task_cfg!
        // set source address
        msg_addr = p_task_cfg->addr;
        localWrite(NV_PRGNLCL_FALCON_SHA_IN_ADDR , LOW32(msg_addr));

        reg = DRF_NUM(_PRGNLCL, _FALCON_SHA_IN_ADDR_HI, _MSB, HI32(msg_addr)) |
              DRF_NUM(_PRGNLCL, _FALCON_SHA_IN_ADDR_HI, _SZ, p_task_cfg->size);
        localWrite(NV_PRGNLCL_FALCON_SHA_IN_ADDR_HI, reg);


        // Set source config
        // TODO: Rename variables such as tmp to appropriate ones.
        tmp = DRF_SIZE(NV_PRGNLCL_FALCON_SHA_IN_ADDR_HI_MSB);
        tmp = (HI32(msg_addr)) >> tmp;
        reg = DRF_NUM(_PRGNLCL, _FALCON_SHA_SRC_CONFIG, _FB_BASE, tmp) |
              DRF_NUM(_PRGNLCL, _FALCON_SHA_SRC_CONFIG, _CTXDMA, p_task_cfg->dma_idx) |
              DRF_NUM(_PRGNLCL, _FALCON_SHA_SRC_CONFIG, _SRC, p_task_cfg->src_type);
        localWrite(NV_PRGNLCL_FALCON_SHA_SRC_CONFIG , reg);

        // Set task config
        reg = 0;
        if (p_task_cfg->b_default_hash_iv)
        {
            // sha512-224 sha512-256, SW needs to set initial vector.
            if ((algo_id == SHA_ALGO_ID_SHA_512_224) ||
                (algo_id == SHA_ALGO_ID_SHA_512_256))
            {
                sha_set_init_vector(algo_id);
                reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_TASK_CONFIG, _HW_INIT_HASH, _DISABLE, reg);
            }
            else
            {
                reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_TASK_CONFIG, _HW_INIT_HASH, _ENABLE, reg);
            }
        }
        else
        {
            reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_TASK_CONFIG, _HW_INIT_HASH, _DISABLE, reg);
        }
        localWrite(NV_PRGNLCL_FALCON_SHA_TASK_CONFIG, reg);

        // Trigger OP start
        reg = 0;
        reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_OPERATION, _LAST_BUF, _TRUE, reg);
        reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_OPERATION, _OP, _START, reg);
        localWrite(NV_PRGNLCL_FALCON_SHA_OPERATION, reg);
    }

    if (status == E_SUCCESS)
    {
        status = sha_wait_engine_idle(SHA_ENGINE_IDLE_TIMEOUT_NS);
    }

    if (status == E_SUCCESS)
    {
        // Check if any error interrupt asserted
        reg = localRead(NV_PRGNLCL_FALCON_SHA_ERR_STATUS);

        if (reg == 0U)
        {
            // Use msg_size to track left message length.
            p_sha_context->msg_size = left_msg_length - p_task_cfg->size;
        }
        else
        {
            status = E_SHA_ENG_ERROR;
        }
    }
    return status;

}

/*!
 * @brief Read SHA hash result.
 *
 * @param[in]  p_sha_context The designated SHA context
 * @param[out] p_buf The output buffer to save computed SHA hash value
 * @param[in]  buf_size The output buffer size.
 * @param[in]  b_scrub_reg true to clear hash registers after hash is read out; false doesn't clear hash registers.
 *
 * @return E_SUCCESS if hash read successfully; otherwise return NVRV_ERR_SHA_XXX.
 *
 */
error_t
sha_read_hash_result
(
    const SHA_CONTEXT *p_sha_context,
    bool b_scrub_reg
)
{
    error_t        status = E_SUCCESS;
    uint32_t       num_words = 0;
    bool           b_double_word = false;

    if (p_sha_context == NULL)
    {
        status = E_INVALID_PARAM;
    }
    else if (p_sha_context->p_buf_out == NULL)
    {
        status = E_INVALID_PARAM;
    }
    else
    {
        uint32_t hash_size_bytes, block_size_bytes;

        // Format inputs
        status = sha_get_hash_size_byte(p_sha_context->algo_id, &hash_size_bytes);
        num_words = hash_size_bytes >> 2;
        if (status == E_SUCCESS)
        {
            status = sha_get_block_size_byte(p_sha_context->algo_id, &block_size_bytes);
        }
        if (status == E_SUCCESS)
        {
            // SHA384 and up have 128B blocks. These algorithms all report results as double-words.
            b_double_word = (block_size_bytes == SHA_384_BLOCK_SIZE_BYTE);
            status = (p_sha_context->buf_size == hash_size_bytes) ? E_SUCCESS : E_INVALID_PARAM;
        }
    }

    if (status == E_SUCCESS)
    {
        uint8_t* p_dest = p_sha_context->p_buf_out;
        uint32_t index = 0;

        for (uint32_t word = 0; word < num_words; word++)
        {
            if (b_double_word)
            {
                bool most_sig_word = (word & 1U) == 1U;
                if (most_sig_word)
                {
                    INLINE_RFD(CERTC, FP, INT30_C, "Under Review: Bug TID-1900, DR: TODO")
                    index = word - 1U;
                }
                else
                {
                    INLINE_RFD(CERTC, FP, INT30_C, "Under Review: Bug TID-1900, DR: TODO")
                    index = word + 1U;
                }
            }
            else
            {
                index = word;
            }

            status = sha_get_result_word(index, p_dest, b_scrub_reg);
            if (status == E_SUCCESS)
            {
                p_dest = &p_dest[sizeof(uint32_t)];
            }
            else
            {
                break;
            }
        }
    }
    return status;
}

/*!
 * @brief Run single task through SHA HW.
 *
 * @param[in] p_task_cfg The designated SHA task config
 * @param[in] p_sha_ctx  The struct containing SHA task parameters and output buffer pointer.
 *
 * @return E_SUCCESS if hash read successfully; otherwise return NVRV_ERR_SHA_XXX.
 *
 */
error_t sha_run_single_task_common
(
    const SHA_TASK_CONFIG *p_task_cfg,
    SHA_CONTEXT     *p_sha_ctx
)
{
    error_t status = E_SUCCESS;
    bool has_mutex = false;

    if ((p_task_cfg == NULL) || (p_sha_ctx == NULL))
    {
        status = E_INVALID_PARAM;
    }
    else if ((p_sha_ctx->msg_size == 0UL) ||
            (p_sha_ctx->msg_size > SHA_MSG_BYTES_MAX) ||
            (p_sha_ctx->p_buf_out == NULL) ||
            (p_sha_ctx->mutex_token == 0UL) ||

            // Only DMEM operations are supported in current SW driver. We can extend to IMEM/FB if necessary.
            (p_task_cfg->src_type != NV_PRGNLCL_FALCON_SHA_SRC_CONFIG_SRC_DMEM) ||
            ((p_task_cfg->size + p_task_cfg->addr) >= NV_RISCV_AMAP_DMEM_END) ||
            ((p_task_cfg->size + p_task_cfg->addr) >= NV_RISCV_AMAP_DMEM_END) ||
            (p_task_cfg->size == 0UL))
    {
        status = E_INVALID_PARAM;
    }
    else
    {
        status = sha_acquire_mutex(p_sha_ctx->mutex_token);
    }

    if (status == E_SUCCESS)
    {
        has_mutex = true;
    }

    if (status == E_SUCCESS)
    {
        status = sha_operation_init(p_sha_ctx);
    }

    if (status == E_SUCCESS)
    {
        status = sha_insert_task(p_sha_ctx, p_task_cfg);
    }

    if (status == E_SUCCESS)
    {
        status = sha_read_hash_result(p_sha_ctx, true);
    }

    if (has_mutex)
    {
        status = sha_release_mutex(p_sha_ctx->mutex_token);
    }
    return status;
}


/*!
 * @brief Initilaize SHA HW to start HMAC SHA-256 operation.
 * @param[in] p_hmac_context The designated HMAC SHA-256 context
 *
 * @return E_SUCCESS if initialization is successfuly; otherwise NVRV_ERR_SHA_XXX return.
 *
 */
error_t
sha_hmac_operation_init
(
    HMAC_CONTEXT *p_hmac_context
)
{
    error_t      status                        = E_SUCCESS;
    uint32_t         loop_count                     = 0;
    uint32_t         algo_block_size_byte             = 0;
    SHA_CONTEXT      *p_sha_context                  = NULL;
    SHA_TASK_CONFIG  task_cfg;
    uint8_t          ipad[HMAC_SHA_MAX_BLOCK_SIZE_BYTE] = {0};

    if (p_hmac_context == NULL)
    {
        status = E_INVALID_PARAM;
    }
    if (status == E_SUCCESS)
    {
        p_sha_context = &(p_hmac_context->sha_context);
    }

    if (status == E_SUCCESS)
    {
        status = sha_get_block_size_byte(p_sha_context->algo_id, &algo_block_size_byte);
    }

    if (status == E_SUCCESS)
    {
        // Ensure context is initialized and valid.
        if ((p_sha_context->msg_size == 0U)                     ||
            !sha_hmac_is_sha_algo_supported(p_sha_context->algo_id)||
            (p_hmac_context->key_size > algo_block_size_byte)     ||
            (p_hmac_context->key_size == 0U))
        {
            status = E_INVALID_PARAM;
        }
        else
        {
            /*
             * Initialize first SHA session - first hash will be H(inner key || message).
             * msg_size will be decremented by algo_hash_size_byte after call to sha_insert_task
             */
            p_sha_context->msg_size = p_sha_context->msg_size + algo_block_size_byte;
        }
    }

    if (status == E_SUCCESS)
    {
        status = sha_operation_init(p_sha_context);
    }

    if (status == E_SUCCESS)
    {
        // Generate inner key.
        (void) memset(ipad, 0, algo_block_size_byte);
        (void) memcpy(ipad, &(p_hmac_context->key_buffer), p_hmac_context->key_size);
        for (loop_count = 0; loop_count < algo_block_size_byte; loop_count++)
        {
           ipad[loop_count] = ipad[loop_count] ^ HMAC_SHA_IPAD_MASK;
        }

        task_cfg.src_type        = NV_PRGNLCL_FALCON_SHA_SRC_CONFIG_SRC_DMEM;
        task_cfg.b_default_hash_iv = true;
        task_cfg.dma_idx         = 0;      // dma_idx is unused here since src_type is DMEM
        task_cfg.size           = algo_block_size_byte;

        task_cfg.addr           =  pointer_to_address(ipad);
    }

    // Add inner key to hash.
    if (status == E_SUCCESS)
    {
        status = sha_insert_task(p_sha_context, &task_cfg);
    }

    return status;
}

/*!
 * @brief Insert HMAC-SHA-256 task configuration.
 *       Task is basically plain textfor which Hash needs to be computed
 *
 * @param[in] p_hmac_context The designated HMAC SHA-256 context
 * @param[in] p_task_cfg The designated SHA task config
 *            p_task_cfg->b_default_hash_iv should always be set to NV_FALSE for HMAC computation
 *            if not done then API itself forces it to NV_FALSE.
 *
 * @return E_SUCCESS if task config is inserted successfully; otherwise return NVRV_ERR_SHA_XXX.
 */
error_t
sha_hmac_insert_task
(
    HMAC_CONTEXT    *p_hmac_context,
    SHA_TASK_CONFIG *p_task_cfg
)
{
    error_t status = E_SUCCESS;

    if ((p_hmac_context == NULL) ||
        (p_task_cfg == NULL) ||
        (!sha_hmac_is_sha_algo_supported(p_hmac_context->sha_context.algo_id)))
    {
        status = E_INVALID_PARAM;
    }
    else
    {
        /*
         * b_default_hash_iv is expected to be false since already SHA computation is begin in
         * sha_hmac_operation_init where SHA(K xor ipad) is computed
         * If this is set to true complete hash value gets modified hence forcing it to false
         */
        p_task_cfg->b_default_hash_iv = false;
    }

    if (status == E_SUCCESS)
    {
        status = sha_insert_task(&(p_hmac_context->sha_context), p_task_cfg);
    }

    return status;
}

/*!
 * @brief Read HMAC-SHA result
 *        This API concludes the HMAC-SHA operation and reads out Hash result in output buffer.
 * @param[in] p_hmac_context The designated HMAC SHA context
 * @return E_SUCCESS if task config is inserted successfully; otherwise return NVRV_ERR_SHA_XXX.
 */
error_t
sha_hmac_read_hash_result
(
    const HMAC_CONTEXT *p_hmac_context
)
{
    error_t         status                                    = E_SUCCESS;
    uint32_t        msg_size                                   = 0;
    uint32_t        loop_count                                 = 0;
    uint32_t        algo_block_size_byte                         = 0;
    uint32_t        algo_hash_size_byte                          = 0;
    const SHA_CONTEXT     *p_sha_context;
    SHA_CONTEXT     temp_sha_context;
    SHA_TASK_CONFIG task_cfg;
    uint8_t         opad[HMAC_SHA_MAX_BLOCK_SIZE_BYTE]        = {0};
    uint8_t        inner_digest[HMAC_SHA_MAX_HASH_SIZE_BYTE]  = {0};

    if (p_hmac_context == NULL)
    {
        status = E_INVALID_PARAM;
    }
    else
    {
        p_sha_context = &(p_hmac_context->sha_context);
    }

    if (status == E_SUCCESS)
    {
        status = sha_get_block_size_byte(p_sha_context->algo_id , &algo_block_size_byte);
    }

    if (status == E_SUCCESS)
    {
        // Ensure context is initialized and valid.
        if (!sha_hmac_is_sha_algo_supported(p_sha_context->algo_id)       ||
            (p_hmac_context->key_size > algo_block_size_byte)              ||
            (p_hmac_context->key_size == 0U))
        {
            status = E_INVALID_PARAM;
        }
        else
        {
            temp_sha_context           = p_hmac_context->sha_context;
            temp_sha_context.p_buf_out = inner_digest;
        }
    }

    if (status == E_SUCCESS)
    {
        status = sha_read_hash_result(&temp_sha_context, true);
    }

    if (status == E_SUCCESS)
    {
        // Generate outer key.
        (void) memset(opad, 0, algo_block_size_byte);
        (void) memcpy(opad, &(p_hmac_context->key_buffer), p_hmac_context->key_size);
        for (loop_count = 0; loop_count < algo_block_size_byte; loop_count++)
        {
           opad[loop_count] = opad[loop_count] ^ HMAC_SHA_OPAD_MASK;
        }
    }

    // Initialize new SHA session for H(outer key || H(inner key || message)).
    if (status == E_SUCCESS)
    {
        status = sha_get_hash_size_byte(p_sha_context->algo_id, &algo_hash_size_byte);
    }

    if (status == E_SUCCESS)
    {
        msg_size = algo_block_size_byte + algo_hash_size_byte;

        temp_sha_context.msg_size = msg_size;
    }

    if (status == E_SUCCESS)
    {
        status = sha_operation_init(&temp_sha_context);
    }

    if (status == E_SUCCESS)
    {
        task_cfg.src_type        = NV_PRGNLCL_FALCON_SHA_SRC_CONFIG_SRC_DMEM;
        task_cfg.b_default_hash_iv = true;
        task_cfg.dma_idx         = 0;      // dma_idx is unused here since src_type is DMEM
        task_cfg.size           = algo_block_size_byte;

        task_cfg.addr           = pointer_to_address(opad);
    }

    // Add outer key to hash, ensuring we use default IV.
    if (status == E_SUCCESS)
    {
        status = sha_insert_task(&temp_sha_context, &task_cfg);
    }

    if (status == E_SUCCESS)
    {
        task_cfg.src_type        = NV_PRGNLCL_FALCON_SHA_SRC_CONFIG_SRC_DMEM;
        task_cfg.b_default_hash_iv = false;
        task_cfg.dma_idx         = 0;      // dma_idx is unused here since src_type is DMEM
        task_cfg.size           = algo_hash_size_byte;

        task_cfg.addr           = pointer_to_address(inner_digest);
    }

    // Add H(inner key || message) to digest, resulting in HMAC.
    if (status == E_SUCCESS)
    {
        status = sha_insert_task(&temp_sha_context, &task_cfg);
    }

    // Output HMAC.
    if (status == E_SUCCESS)
    {
        status = sha_read_hash_result(p_sha_context, true);
    }

    return status;
}

/*!
 * @brief Run single task through HMAC-SHA.
 *        When application has the complete plain text data then this API can be used
 *        This will compute Hash for plain text as well as return the result in output buffer of task_cfg.
 *
 * @param[in] p_hmac_context The struct containing HMAC SHA task parameters and output buffer pointer.
 * @param[in] p_task_cfg     The designated HMAC SHA task config
 *            p_task_cfg->b_default_hash_iv should always be set to NV_FALSE for HMAC computation
 *            if not done then API itself forces it to NV_FALSE.
 *
 * @return E_SUCCESS if hash read successfully; otherwise return NVRV_ERR_SHA_XXX.
 */
error_t sha_hmac_run_single_task_common
(
    HMAC_CONTEXT *p_hmac_context,
    SHA_TASK_CONFIG *p_task_cfg
)
{
    error_t status = E_SUCCESS;
    bool has_mutex = false;

    if ((p_task_cfg == NULL) || (p_hmac_context == NULL))
    {
        status = E_INVALID_PARAM;
    }
    else if ((p_hmac_context->sha_context.msg_size == 0U) ||
            (p_hmac_context->sha_context.msg_size > SHA_MSG_BYTES_MAX) ||
            (p_hmac_context->sha_context.p_buf_out == NULL) ||
            (p_hmac_context->sha_context.mutex_token == 0U) ||

            // Only DMEM operations are supported in current SW driver. We can extend to IMEM/FB if necessary.
            (p_task_cfg->src_type != NV_PRGNLCL_FALCON_SHA_SRC_CONFIG_SRC_DMEM) ||
            ((p_task_cfg->size + p_task_cfg->addr) >= NV_RISCV_AMAP_DMEM_END) ||
            ((p_task_cfg->size + p_task_cfg->addr) >= NV_RISCV_AMAP_DMEM_END) ||
            (p_task_cfg->size  == 0U))
    {
        status = E_INVALID_PARAM;
    }
    else
    {
        status = sha_acquire_mutex(p_hmac_context->sha_context.mutex_token);
    }

    if (status == E_SUCCESS)
    {
        has_mutex = true;
    }

    if (status == E_SUCCESS)
    {
        status = sha_hmac_operation_init(p_hmac_context);
    }

    if (status == E_SUCCESS)
    {
        status = sha_hmac_insert_task(p_hmac_context, p_task_cfg);
    }

    if (status == E_SUCCESS)
    {
        status = sha_hmac_read_hash_result(p_hmac_context);
    }

    if (has_mutex)
    {
        status = sha_release_mutex(p_hmac_context->sha_context.mutex_token);
    }

    return status;
}


/**********************************
* Private Function Definition
**********************************/


/*!
 * @brief To set initial vector for SHA-512-224 and SHA-512-256.
 *
 * @param[in] algo_id The SHA algorithm id.
 *
 */
static void
sha_set_init_vector
(
    SHA_ALGO_ID algo_id
)
{
    size_t arr_len = 0;
    static const uint32_t sha_512_224_iv[16] = {
        0x19544DA2, 0x8C3D37C8, 0x89DCD4D6, 0x73E19966,
        0x32FF9C82, 0x1DFAB7AE, 0x582F9FCF, 0x679DD514,
        0x7BD44DA8, 0x0F6D2B69, 0x04C48942, 0x77E36F73,
        0x6A1D36C8, 0x3F9D85A8, 0x91D692A1, 0x1112E6AD
    };

    static const uint32_t sha_512_256_iv[16] = {
        0xFC2BF72C, 0x22312194, 0xC84C64C2, 0x9F555FA3,
        0x6F53B151, 0x2393B86B, 0x5940EABD, 0x96387719,
        0xA88EFFE3, 0x96283EE2, 0x53863992, 0xBE5E1E25,
        0x2C85B8AA, 0x2B0199FC, 0x81C52CA2, 0x0EB72DDC
    };

    if (algo_id == SHA_ALGO_ID_SHA_512_224)
    {
        arr_len = sizeof(sha_512_224_iv)/sizeof(sha_512_224_iv[0]);
        for (size_t i = 0; i < arr_len; i++)
        {
            localWrite(LOW32(NV_PRGNLCL_FALCON_SHA_HASH_RESULT(i)), sha_512_224_iv[i]);
        }
    }
    else // algo_id == SHA_ALGO_ID_SHA_512_256
    {
        arr_len = sizeof(sha_512_256_iv)/sizeof(sha_512_256_iv[0]);
        for(size_t i = 0; i < arr_len; i++)
        {
            localWrite(LOW32(NV_PRGNLCL_FALCON_SHA_HASH_RESULT(i)), sha_512_256_iv[i]);
        }
    }
}

/*!
 * @brief To check SHA task configuration.
 *
 * @param[in] p_sha_context The designated SHA context
 * @param[in] p_task_cfg The designated SHA task config
 *
 * @return true if task config is valid; otherwise return false.
 *
 */
static bool
sha_task_config_is_valid
(
    const SHA_CONTEXT *p_sha_context,
    const SHA_TASK_CONFIG *p_task_cfg
)
{
    bool b_valid = true;

    /*
     * We use p_sha_context->msg_size to track left message length.
     * Once left message size is less(or equal to) than inserted task size,
     * This means this is the last task. Only the last task size doesn't
     * have to be aligned with block size.
     */
    if (p_sha_context->msg_size == p_task_cfg->size)
    {
        b_valid = true;
    }

    else if (p_sha_context->msg_size < p_task_cfg->size)
    {
        b_valid = false;
    }
    else
    {
        switch (p_sha_context->algo_id)
        {
            case SHA_ALGO_ID_SHA_1:
            case SHA_ALGO_ID_SHA_224:
            case SHA_ALGO_ID_SHA_256:
                b_valid = NV_IS_ALIGNED(p_task_cfg->size, SHA_256_BLOCK_SIZE_BYTE);
            break;

            case SHA_ALGO_ID_SHA_384:
            case SHA_ALGO_ID_SHA_512:
            case SHA_ALGO_ID_SHA_512_224:
            case SHA_ALGO_ID_SHA_512_256:
                b_valid = NV_IS_ALIGNED(p_task_cfg->size, SHA_512_BLOCK_SIZE_BYTE);
            break;

            default:
                b_valid = false;
            break;
        }
    }
    return b_valid;
}

/*
 * @brief This function checks whether particular SHA_ALGO_ID is supported for HMAC computation.
 * @param[in] algo_id SHA ALGO Id
 *
 * @return true if SHA_ALGO_ID is supported else false.
 */
static bool sha_hmac_is_sha_algo_supported
(
    SHA_ALGO_ID algo_id
)
{
    bool supported;
    switch (algo_id)
    {
        case SHA_ALGO_ID_SHA_256:
        case SHA_ALGO_ID_SHA_384:
        case SHA_ALGO_ID_SHA_512:
            supported = true;
            break;
        default:
            supported = false;
            break;
    }
    return supported;
}

/*!
 * @brief Wait for SHA engine idle
 *
 * @param[in] time_out_ns Timeout initialization value
 *
 * @return E_SUCCESS if engine enter idle state successfuly.
 *         E_SHA_ENG_ERROR if SHA halted status asserted
 *         E_SHA_WAIT_IDLE_TIMEOUT if timeout detected
 */
static error_t
sha_wait_for_busy
(
    uint64_t time_out_ns
)
{
    uint32_t    reg;
    uint64_t    start_time;
    error_t     status = E_SUCCESS;

    start_time = ptimer_read();

    do
    {
        reg = localRead(NV_PRGNLCL_FALCON_SHA_STATUS);

        if (timeout_is_hit(start_time, time_out_ns))
        {
            status = E_SHA_WAIT_IDLE_TIMEOUT;
            break;
        }
    } while (FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA, _STATUS_STATE, _BUSY, reg));

    return status;
}

/*!
 * @brief Wait for SHA engine idle
 *
 * @param[in] time_out_ns Timeout initialization value
 *
 * @return E_SUCCESS if engine enter idle state successfuly.
 *         E_SHA_ENG_ERROR if SHA halted status asserted
 *         E_SHA_WAIT_IDLE_TIMEOUT if timeout detected
 */
static error_t
sha_wait_engine_idle
(
    uint64_t time_out_ns
)
{
    uint32_t    reg;
    uint64_t    start_time;
    error_t     status = E_SUCCESS;

    start_time = ptimer_read();

    do
    {
        reg = localRead(NV_PRGNLCL_FALCON_SHA_STATUS);
        if (FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA, _STATUS_STATE, _HALTED, reg))
        {
            status = E_SHA_ENG_ERROR;
        }
        if (timeout_is_hit(start_time, time_out_ns) && (status == E_SUCCESS))
        {
            status = E_SHA_WAIT_IDLE_TIMEOUT;
        }
        if(status != E_SUCCESS)
        {
            break;
        }
    } while (FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA, _STATUS_STATE, _BUSY, reg));

    return status;
}

/*!
 * @brief Execute SHA engine soft reset and wait for engine idle
 *
 * @param[in] time_out_ns Timeout initialization value
 *
 * @return E_SUCCESS if reset and enter idle state successfuly.
           E_SHA_WAIT_IDLE_TIMEOUT/SW_RESET_TIMEOUT if timeout detected
 *
 */
static error_t
sha_engine_soft_reset
(
    uint64_t time_out_ns
)
{
    uint32_t    reg;
    uint64_t    start_time = 0;
    error_t status = E_SUCCESS;

    reg = localRead(NV_PRGNLCL_FALCON_SHA_STATUS);

    if (FLD_TEST_DRF(_PRGNLCL, _FALCON_SHA, _STATUS_STATE, _BUSY, reg))
    {
        status = sha_engine_halt();
    }

    if (status == E_SUCCESS)
    {
        reg = FLD_SET_DRF(_PRGNLCL_FALCON, _SHA_OPERATION, _SOFTRESET, _ENABLE, reg);
        localWrite(NV_PRGNLCL_FALCON_SHA_OPERATION, reg);

        start_time = ptimer_read();

        // wait for soft reset clear
        do
        {
            reg = localRead(NV_PRGNLCL_FALCON_SHA_OPERATION);

            if (timeout_is_hit(start_time, time_out_ns))
            {
                status = E_SHA_SW_RESET_TIMEOUT;
                break;
            }
        } while (FLD_TEST_DRF(_PRGNLCL_FALCON, _SHA_OPERATION, _SOFTRESET, _ENABLE, reg));
    }

    if (status == E_SUCCESS)
    {
        status = sha_wait_engine_idle(SHA_ENGINE_IDLE_TIMEOUT_NS);
    }

    return status;
}

/*!
 * @brief To get SHA encode value per SHA algorithm id
 *
 * @param[in]  algo_id SHA algorithm id
 * @param[out] *p_mode The pointer to save encode value
 *
 * @return E_SUCCESS if get encode value successfully.
 *         E_INVALID_PARAM if failed.
 *
 */
static error_t
sha_get_config_encode_mode
(
    SHA_ALGO_ID algo_id,
    uint8_t     *p_mode
)
{
    error_t status = E_SUCCESS;
    switch (algo_id)
    {
        case SHA_ALGO_ID_SHA_1:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA1;
            break;
        case SHA_ALGO_ID_SHA_224:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA224;
            break;
        case SHA_ALGO_ID_SHA_256:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA256;
            break;
        case SHA_ALGO_ID_SHA_384:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA384;
            break;
        case SHA_ALGO_ID_SHA_512:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA512;
            break;
        case SHA_ALGO_ID_SHA_512_224:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA512_224;
            break;
        case SHA_ALGO_ID_SHA_512_256:
            *p_mode = NV_PRGNLCL_FALCON_SHA_CONFIG_ENC_MODE_SHA512_256;
            break;
        default:
            status = E_INVALID_PARAM;
            break;
    }

    return status;
}

/*!
 * @brief Convert a pointer to a uint64_t so that it can be passed as 64b address to sha engine
 *        indicating location of data to hash.
 *
 * @param[in]  buf               pointer to buffer/data to hash
 *
 * @return uint64_t
 *
 */
INLINE_RFD(MISRA, DEVIATE, Rule_8_13, "TODO: The need for this deviation will be removed when COREUCODES-2066 is resolved")
static inline uint64_t pointer_to_address(uint8_t* buf)
{
    INLINE_RFD(MISRA, DEVIATE, Rule_11_4, "TODO: The need for this deviation will be removed when COREUCODES-2066 is resolved")
    return (uint64_t) buf;
}

/*!
 * @brief Identify if a timeout has been hit (accounting for integer overflow)
 *
 * @param[in]  start_time        start time of operation
 * @param[in]  timeout           max duration to wait before timeout
 *
 * @return E_SUCCESS if fetch configuration can be set correctly; otherwise return E_INVALID_PARAM.
 *
 */
static inline bool timeout_is_hit(uint64_t start_time, uint64_t timeout)
{
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Under Review: Bug TID-1899, DR: TODO")
    return (ptimer_read() - start_time) > timeout;
}

/*!
 * @brief Get Hash Result. Guard against illegal reads.
 *
 * @param[in]  register_index   register to read from [0,SHA_MAX_RESULT_REGISTER]
 * @param[out] result           buffer to write result into
 * @param[in]  b_scrub_reg      scrub (zero) hash result after read if true
 *
 * @return E_SUCCESS if read is valid, E_INVALID_PARAM otherwise
 *
 */
static inline error_t sha_get_result_word(uint32_t register_index, uint8_t* result, bool b_scrub_reg)
{
    error_t status = E_SUCCESS;
    if((result == NULL) || (register_index > NV_PRGNLCL_FALCON_SHA_HASH_RESULT__SIZE_1))
    {
        status = E_INVALID_PARAM; // Unreachable due to input formatting in sha_read_hash_result
    }
    else
    {
        INLINE_RFD(CERTC, FP, INT30_C, "Under Review: Bug TID-1901, DR: TODO")
        uint32_t reg = NV_PRGNLCL_FALCON_SHA_HASH_RESULT(register_index);
        uint32_t hash = localRead(reg);
        hash = NV_BYTESWAP32(hash);
        (void) memcpy(result, (uint8_t*) &hash, sizeof(hash));
        if(b_scrub_reg)
        {
            localWrite(reg, 0);
        }
    }
    return status;
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx",
              MISRA, DEVIATE, Rule_8_7, "Approval: TODO, DR: TODO")


/*** end of file ***/
