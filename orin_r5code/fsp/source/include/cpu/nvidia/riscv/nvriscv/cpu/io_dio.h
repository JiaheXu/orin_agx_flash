/* _NVRM_COPYRIGHT_BEGIN_
 *
 * Copyright 2021 by NVIDIA Corporation.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 * _NVRM_COPYRIGHT_END_
 */
#ifndef CPU__IO_DIO_H
#define CPU__IO_DIO_H
#define FSP__CPU__IO_DIO_H                    1

#include <error/common-errors.h>

/* =============================================================================
 * Public APIs
 * ========================================================================== */
#ifdef UTF_UCODE_BUILD
extern uint32_t g_dio_max_wait;
#define DIO_MAX_WAIT_DEFAULT 1U
#endif

typedef enum
{
    DIO_TYPE_INVALID = 0U,
#if NVRISCV_HAS_DIO_SE
    DIO_TYPE_SE,
#endif // NVRISCV_HAS_DIO_SE
#if NVRISCV_HAS_DIO_SNIC
    DIO_TYPE_SNIC,
#endif // NVRISCV_HAS_DIO_SNIC
#if NVRISCV_HAS_DIO_FBHUB
    DIO_TYPE_FBHUB,
#endif // NVRISCV_HAS_DIO_FBHUB
    DIO_TYPE_END
} DIO_TYPE;

typedef enum
{
    DIO_OPERATION_RD,
    DIO_OPERATION_WR
} DIO_OPERATION;

/*!
 * @brief   Perform one DIO read or write operation.
 *
 * @param[in]     type      one of the supported DIO type
 * @param[in]     operation DIO_OPERATION_RD or DIO_OPERATION_WR
 * @param[in]     addr      address
 * @param[in/out] p_data    read back data for DIO_OPERATION_RD, or data input
 *                          for DIO_OPERATION_WR.
 *
 * @return E_SUCCESS if operation finished successfully.
 * @return E_NOTSUPPORTED if the combination of the inputs is not supported.
 * @return E_TIMEOUT if operation timeout.
 * @return E_FAULT if error reported through DOC_CTRL register.
 * @return E_INVALID_PARAM if error reported through DOC_CTRL register and DIO_ERR_INFO
 *         reports extra info.
 */
error_t
dioReadWrite
(
    DIO_TYPE type,
    DIO_OPERATION operation,
    uint32_t addr,
    uint32_t *p_data
);

#endif // CPU__IO_DIO_H
/*** end of file ***/