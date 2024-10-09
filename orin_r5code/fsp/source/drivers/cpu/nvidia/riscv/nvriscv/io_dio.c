/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#include <stdbool.h>
#include <stddef.h>

#include <misc/nvmisc_drf.h>
#include <cpu/io_local.h>
#include <cpu/io_dio.h>

/* =============================================================================
 * DIO additional macros
 * ========================================================================== */
#ifdef UTF_UCODE_BUILD
uint32_t g_dio_max_wait = DIO_MAX_WAIT_DEFAULT;
#   define DIO_MAX_WAIT g_dio_max_wait
#else
#   define DIO_MAX_WAIT 0xfffffffeU
#endif

// sw defines to unify hw manual defined dio registers fields
// NV_PRGNLCL_FALCON_x and
// NV_PRGNLCL_FALCON_DIO_x(i) with
// NV_PRGNLCL_DIO_x
//
#define NV_PRGNLCL_DIO_DOC_CTRL_RESET_XMSB                                   16U
#define NV_PRGNLCL_DIO_DOC_CTRL_RESET_XLSB                                   16U
#define NV_PRGNLCL_DIO_DOC_CTRL_EMPTY_XMSB                                   18U
#define NV_PRGNLCL_DIO_DOC_CTRL_EMPTY_XLSB                                   18U
#define NV_PRGNLCL_DIO_DOC_CTRL_WR_FINISHED_XMSB                             19U
#define NV_PRGNLCL_DIO_DOC_CTRL_WR_FINISHED_XLSB                             19U
#define NV_PRGNLCL_DIO_DOC_CTRL_RD_FINISHED_XMSB                             20U
#define NV_PRGNLCL_DIO_DOC_CTRL_RD_FINISHED_XLSB                             20U
#define NV_PRGNLCL_DIO_DOC_CTRL_WR_ERROR_XMSB                                21U
#define NV_PRGNLCL_DIO_DOC_CTRL_WR_ERROR_XLSB                                21U
#define NV_PRGNLCL_DIO_DOC_CTRL_RD_ERROR_XMSB                                22U
#define NV_PRGNLCL_DIO_DOC_CTRL_RD_ERROR_XLSB                                22U
#define NV_PRGNLCL_DIO_DOC_CTRL_PROTOCOL_ERROR_XMSB                          23U
#define NV_PRGNLCL_DIO_DOC_CTRL_PROTOCOL_ERROR_XLSB                          23U
#define NV_PRGNLCL_DIO_DIC_CTRL_COUNT_XMSB                                    7U
#define NV_PRGNLCL_DIO_DIC_CTRL_COUNT_XLSB                                    0U
#define NV_PRGNLCL_DIO_DIC_CTRL_VALID_XMSB                                   19U
#define NV_PRGNLCL_DIO_DIC_CTRL_VALID_XLSB                                   19U
#define NV_PRGNLCL_DIO_DIC_CTRL_POP_XMSB                                     20U
#define NV_PRGNLCL_DIO_DIC_CTRL_POP_XLSB                                     20U

// sw defines for specific DOC Dx interface
//
#if NVRISCV_HAS_DIO_SE
#define NV_PRGNLCL_DIO_DOC_D0_SEHUB_READ_XMSB                                16U
#define NV_PRGNLCL_DIO_DOC_D0_SEHUB_READ_XLSB                                16U
#define NV_PRGNLCL_DIO_DOC_D0_SEHUB_ADDR_XMSB                                15U
#define NV_PRGNLCL_DIO_DOC_D0_SEHUB_ADDR_XLSB                                 0U
#define NV_PRGNLCL_DIO_DOC_D1_SEHUB_WDATA_XMSB                               31U
#define NV_PRGNLCL_DIO_DOC_D1_SEHUB_WDATA_XLSB                                0U
#endif // NVRISCV_HAS_DIO_SE
#if NVRISCV_HAS_DIO_SNIC
#define NV_PRGNLCL_DIO_DOC_D0_SNIC_WDATA_XMSB                                31U
#define NV_PRGNLCL_DIO_DOC_D0_SNIC_WDATA_XLSB                                 0U
#define NV_PRGNLCL_DIO_DOC_D1_SNIC_ADDR_XMSB                                 31U
#define NV_PRGNLCL_DIO_DOC_D1_SNIC_ADDR_XLSB                                  0U
#define NV_PRGNLCL_DIO_DOC_D2_SNIC_READ_XMSB                                  0U
#define NV_PRGNLCL_DIO_DOC_D2_SNIC_READ_XLSB                                  0U
#endif // NVRISCV_HAS_DIO_SNIC
#if NVRISCV_HAS_DIO_FBHUB
#define NV_PRGNLCL_DIO_DOC_D0_FBHUB_READ_XMSB                                16U
#define NV_PRGNLCL_DIO_DOC_D0_FBHUB_READ_XLSB                                16U
#define NV_PRGNLCL_DIO_DOC_D0_FBHUB_ADDR_XMSB                                15U
#define NV_PRGNLCL_DIO_DOC_D0_FBHUB_ADDR_XLSB                                 0U
#define NV_PRGNLCL_DIO_DOC_D1_FBHUB_WDATA_XMSB                               31U
#define NV_PRGNLCL_DIO_DOC_D1_FBHUB_WDATA_XLSB                                0U
#endif // NVRISCV_HAS_DIO_FBHUB

/* =============================================================================
 * DIO control registers
 * ========================================================================== */
typedef struct
{
    uint32_t doc_ctrl;
    uint32_t doc_d0, doc_d1, doc_d2;
    uint32_t dic_ctrl;
    uint32_t dic_d0, dic_d1, dic_d2;
    uint32_t dio_error;
} DIO_CTRL_REGS;

static const DIO_CTRL_REGS dio_ctrl_regs_array[] =
{
    // DIO_TYPE_INVALID
    //
    {},
#if NVRISCV_HAS_DIO_SE
    // DIO_TYPE_SE
    //
    {
        NV_PRGNLCL_FALCON_DOC_CTRL,
        NV_PRGNLCL_FALCON_DOC_D0,
        NV_PRGNLCL_FALCON_DOC_D1,
        NV_PRGNLCL_FALCON_DOC_D2,
        NV_PRGNLCL_FALCON_DIC_CTRL,
        NV_PRGNLCL_FALCON_DIC_D0,
        NV_PRGNLCL_FALCON_DIC_D1,
        NV_PRGNLCL_FALCON_DIC_D2,
        NV_PRGNLCL_FALCON_DIO_ERR,
    },
#endif // NVRISCV_HAS_DIO_SE
#if NVRISCV_HAS_DIO_SNIC
    // DIO_TYPE_SNIC
    //
    {
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_CTRL(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_D0(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_D1(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_D2(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_CTRL(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_D0(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_D1(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_D2(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIO_ERR(0),
    },
#endif // NVRISCV_HAS_DIO_SNIC
#if NVRISCV_HAS_DIO_FBHUB
    // DIO_TYPE_FBHUB
    //
    {
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_CTRL(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_D0(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_D1(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DOC_D2(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_CTRL(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_D0(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_D1(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIC_D2(0),
        (uint32_t)NV_PRGNLCL_FALCON_DIO_DIO_ERR(0),
    },
#endif // NVRISCV_HAS_DIO_FBHUB
};

/* =============================================================================
 * DIO core helper functions
 * ========================================================================== */
static error_t
dio_send_request
(
    DIO_TYPE type,
    DIO_OPERATION operation,
    uint32_t addr,
    const uint32_t *p_data
);

static error_t
dio_wait_for_doc_empty (DIO_TYPE type);

static error_t
dio_wait_for_operation_complete
(
    DIO_TYPE type,
    DIO_OPERATION operation,
    uint32_t *p_data
);

static bool
dio_if_timeout (uint32_t *p_timer_count);

static error_t
dio_reset (DIO_TYPE type);

/* =============================================================================
 * DIO API implementation
 * ========================================================================== */
error_t
dioReadWrite
(
    DIO_TYPE type,
    DIO_OPERATION operation,
    uint32_t addr,
    uint32_t *p_data
)
{
    error_t status = E_SUCCESS;

    if ((type == DIO_TYPE_INVALID) ||
        (type >= DIO_TYPE_END)) {
        status = (E_NOTSUPPORTED);
    }

    if (status == E_SUCCESS) {
        status = dio_wait_for_doc_empty(type);
    }

    if (status == E_SUCCESS) {
        status = dio_send_request(type, operation, addr, p_data);
    }

    if (status == E_SUCCESS) {
        status = (dio_wait_for_operation_complete(type, operation, p_data));
    }

    return status;
} /* dioReadWrite() */

/* =============================================================================
 * DIO core helper functions implementation
 * ========================================================================== */
/*!
 * @brief   Send single dio read or write request
 * @details Request is sent by populating corresponding DOCDx registers based on
 *          DIO_TYPE and operation.
 */
static error_t
dio_send_request
(
    DIO_TYPE type,
    DIO_OPERATION operation,
    uint32_t addr,
    const uint32_t *p_data
)
{
    error_t status = (E_SUCCESS);
    switch (type) {
#if NVRISCV_HAS_DIO_SE
        case DIO_TYPE_SE:
            localWrite(dio_ctrl_regs_array[type].doc_d1,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D1, _SEHUB_WDATA,
                    (operation == DIO_OPERATION_RD) ? 0U : *p_data));
            localWrite(dio_ctrl_regs_array[type].doc_d0,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D0, _SEHUB_READ,
                    (operation == DIO_OPERATION_RD) ? 1U : 0U) |
                DRF_NUM(_PRGNLCL, _DIO_DOC_D0, _SEHUB_ADDR, addr));
            break;
#endif // NVRISCV_HAS_DIO_SE
#if NVRISCV_HAS_DIO_SNIC
        case DIO_TYPE_SNIC:
            localWrite(dio_ctrl_regs_array[type].doc_d2,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D2, _SNIC_READ,
                    (operation == DIO_OPERATION_RD) ? 1U : 0U));
            localWrite(dio_ctrl_regs_array[type].doc_d1,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D1, _SNIC_ADDR, addr));
            localWrite(dio_ctrl_regs_array[type].doc_d0,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D0, _SNIC_WDATA,
                    (operation == DIO_OPERATION_RD) ? 0U : *p_data));
            break;
#endif // NVRISCV_HAS_DIO_SNIC
#if NVRISCV_HAS_DIO_FBHUB
        case DIO_TYPE_FBHUB:
            localWrite(dio_ctrl_regs_array[type].doc_d1,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D1, _FBHUB_WDATA,
                    (operation == DIO_OPERATION_RD) ? 0U : *p_data));
            localWrite(dio_ctrl_regs_array[type].doc_d0,
                DRF_NUM(_PRGNLCL, _DIO_DOC_D0, _FBHUB_READ,
                    (operation == DIO_OPERATION_RD) ? 1U : 0U) |
                DRF_NUM(_PRGNLCL, _DIO_DOC_D0, _FBHUB_ADDR, addr));
            break;
#endif // NVRISCV_HAS_DIO_FBHUB
        default:
            status = (E_NOTSUPPORTED);
            break;
    }
    return status;
} /* dio_send_request() */

/*!
 * @brief    Wait for free entry in DOC
 * @details  The function tries to take a free entry in DOC and exit with no DIO
 *           errors.
 */
static error_t
dio_wait_for_doc_empty (DIO_TYPE type)
{
    uint32_t timer_count = 0U;
    uint32_t doc_ctrl = 0U;
    uint32_t dio_error_code = 0U;
    error_t  status = (E_SUCCESS);
    do {
        doc_ctrl = localRead(dio_ctrl_regs_array[type].doc_ctrl);

        if ((DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _WR_ERROR, doc_ctrl) |
             DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _RD_ERROR, doc_ctrl) |
             DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _PROTOCOL_ERROR, doc_ctrl)) != 0U) {
            dio_error_code = localRead(dio_ctrl_regs_array[type].dio_error);
            status = dio_reset(type);
            if (status == E_SUCCESS) {
                status = ((dio_error_code != 0U) ? E_INVALID_PARAM : E_FAULT);
            }
        }

        if (dio_if_timeout(&timer_count)) {
            status = dio_reset(type);
            if (status == E_SUCCESS) {
                status = (E_TIMEOUT);
            }
        }

    } while ((status == (E_SUCCESS)) && 
             (DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _EMPTY, doc_ctrl) == 0U));

    return status;
} /* dio_wait_for_doc_empty() */

/*!
 * @brief    Wait for operation to complete and get response for read.
 * @details  We make sure no error is caused by the operation.
 */
static error_t
dio_wait_for_operation_complete
(
    DIO_TYPE type,
    DIO_OPERATION operation,
    uint32_t *p_data
)
{
    uint32_t timer_count = 0U;
    uint32_t doc_ctrl = 0U;
    uint32_t dio_error_code = 0U;
    error_t  status = (E_SUCCESS);
    do {
        doc_ctrl = localRead(dio_ctrl_regs_array[type].doc_ctrl);
        if ((DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _WR_ERROR, doc_ctrl) |
             DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _RD_ERROR, doc_ctrl) |
             DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _PROTOCOL_ERROR, doc_ctrl)) != 0U) {
            dio_error_code = localRead(dio_ctrl_regs_array[type].dio_error);
            status = dio_reset(type);
            if (status == E_SUCCESS) {
                status = ((dio_error_code != 0U) ? E_INVALID_PARAM : E_FAULT);
            }
        }

        if (dio_if_timeout(&timer_count)) {
            status = dio_reset(type);
            if (status == E_SUCCESS) {
                status = (E_TIMEOUT);
            }
        }

    } while ((status == (E_SUCCESS)) && (((operation == DIO_OPERATION_RD) ?
        DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _RD_FINISHED, doc_ctrl) :
        DRF_VAL(_PRGNLCL, _DIO_DOC_CTRL, _WR_FINISHED, doc_ctrl)) == 0U));

    // check dic_ctrl once doc_ctrl indicate operation done
    //
    timer_count = 0U;
    if ((status == (E_SUCCESS)) && (operation == DIO_OPERATION_RD)) {
        uint32_t dic_ctrl = 0U;
        do {
            dic_ctrl = localRead(dio_ctrl_regs_array[type].dic_ctrl);
            if (dio_if_timeout(&timer_count)) {
                status = dio_reset(type);
                if (status == E_SUCCESS) {
                    status = (E_TIMEOUT);
                }
            }

        } while ((status == (E_SUCCESS)) &&
                 (DRF_VAL(_PRGNLCL, _DIO_DIC_CTRL, _COUNT, dic_ctrl) == 0U));

        // pop data and clear
        //
        if (status == (E_SUCCESS)) {
            localWrite(dio_ctrl_regs_array[type].dic_ctrl,
                DRF_NUM(_PRGNLCL, _DIO_DIC_CTRL, _POP, 0x1U));
            *p_data = localRead(dio_ctrl_regs_array[type].dic_d0);
            localWrite(dio_ctrl_regs_array[type].dic_ctrl,
                DRF_NUM(_PRGNLCL, _DIO_DIC_CTRL, _VALID, 0x1U));
        }
    }

    return status;
} /* dio_wait_for_operation_complete */

/*!
 * @brief  Check the given counter reaches timeout
 */
static bool
dio_if_timeout (uint32_t *p_timer_count)
{
    bool b_timeout = (*p_timer_count >= DIO_MAX_WAIT);
    if (b_timeout == false)
    {
        (*p_timer_count)++;
    }
    return b_timeout;
} /* dio_if_timeout */

/*!
 * @brief  Set clear bit and wait it to be cleared by hw on finish
 */
static error_t
dio_reset (DIO_TYPE type)
{
    error_t status = (E_SUCCESS);
    uint32_t timer_count = 0U;

    localWrite(dio_ctrl_regs_array[type].doc_ctrl,
        DRF_NUM(_PRGNLCL, _DIO_DOC_CTRL, _RESET, 0x1U));

    while (FLD_TEST_DRF_NUM(_PRGNLCL, _DIO_DOC_CTRL, _RESET, 0x1U,
            localRead(dio_ctrl_regs_array[type].doc_ctrl))) {
        if (dio_if_timeout(&timer_count)) {
            status = (E_TIMEOUT);
            break;
        }
    }

    return status;
} /* dio_reset */

/*** end of file ***/