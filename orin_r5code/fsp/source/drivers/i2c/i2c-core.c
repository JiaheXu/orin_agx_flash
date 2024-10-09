/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */
#include <stdbool.h>                      // for true
#include <stdint.h>                       // for uint16_t, uint32_t, uint8_t
#include <string.h>                       // for NULL, memcpy, size_t

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <error/common-errors.h>          // for E_INVALID_PARAM, E_SUCCESS...
#include <misc/attributes.h>              // for UNUSED
#include <misc/macros.h>                  // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD

/* Module-specific FSP headers */
#include <i2c/i2c-errors.h>               // for E_I2C_INVALID_OPS, FSP__I2C...
#include <i2c/i2c-hw-ops.h>               // for i2c_hw_ops, FSP__I2C__I2C_H...
#include <i2c/i2c-tegra.h>                // for i2c_xfer_msg, tegra_i2c_init
#include <i2c/i2c.h>                      // for i2c_handle, FSP__I2C__I2C_H
#include <soc-common/i2c-dependencies.h>  // for appfw_mutex_acquire, appfw_...

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_HW_OPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEPENDENCIES_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/* Custom Defines */
/* First msg contains the slave address and second contains data to transfer */
#define I2C_REG_XFER_NUM_MSGS       2U
#define I2C_XFER_NUM_MSGS           1U

/* Stuct Definitions and Declarations */

/* API Definitions */

error_t i2c_reg_write_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                           uint8_t reg_address, uint16_t flags, uint8_t const *pdata,
                           uint16_t num_data, uint32_t timeout)
{
    struct i2c_xfer_msg msg;
    error_t ret;
    uint8_t data[I2C_PKT_XFER_MAX_BYTES];

    data[0] = reg_address;

    if ((hi2c == NULL) || (pdata == NULL) || (num_data == 0U)
            || (num_data > (I2C_PKT_XFER_MAX_BYTES - 1U))) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" :
                        ((pdata == NULL) ? "pdata" : "num_data")), (int32_t)ret);
        goto out;
    }

    UNUSED((memcpy(&data[1], pdata, (size_t)num_data)));
    msg.dev_addr = slave_address;
    msg.pwbuf = data;
    msg.buf_len = num_data + 1U;
    msg.xfer_flags = flags;

    ret = i2c_do_transfer(hi2c, &msg, I2C_XFER_NUM_MSGS, timeout);

out:
    return ret;
}

error_t i2c_reg_read_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                          uint8_t reg_address, uint16_t flags, uint8_t *pdata,
                          uint16_t num_data, uint32_t timeout)
{
    struct i2c_xfer_msg m[I2C_REG_XFER_NUM_MSGS];
    error_t ret;

    if ((hi2c == NULL) || (pdata == NULL) || (num_data == 0ULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" :
                        ((pdata == NULL) ? "pdata" : "num_data")), (int32_t)ret);
        goto out;
    }

    m[0].dev_addr = slave_address;
    m[0].pwbuf = &reg_address;
    m[0].buf_len = 1U;
    m[0].xfer_flags = flags;

    m[1].dev_addr = slave_address;
    m[1].prbuf = pdata;
    m[1].buf_len = num_data;
    m[1].xfer_flags = flags;
    m[1].xfer_flags |= (uint16_t)I2C_XFER_FLAG_RD;

    ret = i2c_do_transfer(hi2c, m, I2C_REG_XFER_NUM_MSGS, timeout);

out:
    return ret;
}

error_t i2c_send_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                      uint16_t flags, const uint8_t *pdata, uint32_t num_data,
                      uint32_t timeout)
{
    struct i2c_xfer_msg m;
    error_t ret;

    if ((hi2c == NULL) || (pdata == NULL) || (num_data == 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" :
                        ((pdata == NULL) ? "pdata" : "num_data")), (int32_t)ret);
        goto out;
    }

    m.dev_addr = slave_address;
    m.pwbuf = pdata;
    m.buf_len = (uint16_t)num_data;
    m.xfer_flags = flags;

    ret = i2c_do_transfer(hi2c, &m, I2C_XFER_NUM_MSGS, timeout);

out:
    return ret;
}

error_t i2c_receive_data(const struct i2c_handle *hi2c, uint16_t slave_address,
                         uint16_t flags, uint8_t *pdata, uint32_t num_data,
                         uint32_t timeout)
{
    struct i2c_xfer_msg m;
    error_t ret;

    if ((hi2c == NULL) || (pdata == NULL) || (num_data == 0ULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" :
                        ((pdata == NULL) ? "pdata" : "num_data")), (int32_t)ret);
        goto out;
    }

    m.dev_addr = slave_address;
    m.prbuf = pdata;
    m.buf_len = (uint16_t)num_data;
    m.xfer_flags = flags;
    m.xfer_flags |= (uint16_t)I2C_XFER_FLAG_RD;

    ret = i2c_do_transfer(hi2c, &m, I2C_XFER_NUM_MSGS, timeout);

out:
    return ret;
}

error_t i2c_do_transfer(const struct i2c_handle *hi2c,
                        struct i2c_xfer_msg *msgs,
                        uint32_t num_msgs, uint32_t timeout)
{
    error_t ret;
    error_t tmp;
    uint32_t i;

    if ((hi2c == NULL) || (msgs == NULL) || (num_msgs == 0UL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" :
                        ((msgs == NULL) ? "msgs" : "num_data")), (int32_t)ret);
        goto out;
    }

    for (i = 0U; i < num_msgs; i++) {
        if (msgs[i].buf_len == 0U){
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            ret = E_INVALID_PARAM;
            i2c_pr_error("%s: invalid msg[%lu].buf_len, ret - 0x%x\n", __func__,
                            i, (int32_t)ret);
            goto out;
        }
    }

    ret = appfw_mutex_acquire(hi2c->ctrl_id, timeout);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: failed to acquire mutex, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    if (hi2c->hw_ops->xfer != NULL) {
        ret = hi2c->hw_ops->xfer(hi2c->hi2c_tegra, msgs, num_msgs, timeout);
    } else {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_INVALID_OPS;
        i2c_pr_error("%s: invalid hi2c->hw_ops->xfer, ret - 0x%x\n", __func__,
                        (int32_t)ret);
    }

    tmp = appfw_mutex_release(hi2c->ctrl_id);
    if (ret == E_SUCCESS) {
        if (tmp != E_SUCCESS) {
            ret = tmp;
            i2c_pr_error("%s: failed to release mutex, ret - 0x%x\n", __func__,
                            (int32_t)ret);
        }
    }

out:
    return ret;
}

error_t i2c_get_bus_state(const struct i2c_handle *hi2c, uint32_t *state)
{
    error_t ret = E_SUCCESS;

    if ((hi2c == NULL) || (hi2c->hw_ops == NULL) || (state == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" :
                        ((hi2c->hw_ops == NULL) ? "hi2c->hw_ops" : "state")),
                    (int32_t)ret);
        goto out;
    }

    if(hi2c->hw_ops->get_bus_state == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_INVALID_OPS;
        i2c_pr_error("%s: invalid hi2c->hw_ops->get_bus_state, ret - 0x%x\n",
                        __func__, (int32_t)ret);
        goto out;
    }

    ret = hi2c->hw_ops->get_bus_state(hi2c->hi2c_tegra, state);

out:
    return ret;
}

error_t i2c_suspend(const struct i2c_handle *hi2c)
{
    error_t ret;

    if ((hi2c == NULL) || (hi2c->hw_ops == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" : "hi2c->hw_ops"), (int32_t)ret);
        goto out;
    }

    if (hi2c->hw_ops->suspend == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_INVALID_OPS;
        i2c_pr_error("%s: invalid hi2c->hw_ops->suspend, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    ret = hi2c->hw_ops->suspend(hi2c->hi2c_tegra);

out:
    return ret;
}

error_t i2c_resume(const struct i2c_handle *hi2c)
{
    error_t ret;

    if ((hi2c == NULL) || (hi2c->hw_ops == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    ((hi2c == NULL) ? "hi2c" : "hi2c->hw_ops"), (int32_t)ret);
        goto out;
    }

    if (hi2c->hw_ops->resume == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_INVALID_OPS;
        i2c_pr_error("%s: invalid hi2c->hw_ops->resume, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    ret = hi2c->hw_ops->resume(hi2c->hi2c_tegra);

out:
    return ret;
}

error_t i2c_controller_pre_init(uint32_t ctrl_id, struct i2c_handle *hi2c)
{
    error_t ret;

    if (ctrl_id > I2C_MAX_CTRLS) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid ctrl_id - %lu, allowed MAX - %u, ret - 0x%x\n",
                        __func__, ctrl_id, I2C_MAX_CTRLS, (int32_t)ret);
        goto out;
    }

    if (hi2c != NULL) {
        if (hi2c->enabled) {
            i2c_pr_debug("%s: i2c controller already enabled\n", __func__);
            ret = E_SUCCESS;
            goto out;
        }
    } else {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: hi2c not declared, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

#ifdef I2C_CUSTOM_PORT_INIT
    ret = appfw_port_init(ctrl_id);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: appfw_port_init failed, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }
#else
    ret = appfw_mutex_create(ctrl_id);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: failed to acquire mutex, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }
#endif

    hi2c->hi2c_tegra = i2c_get_tegra_handle(ctrl_id);
    if (hi2c->hi2c_tegra == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: hi2c->hi2c_tegra not declared, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    ret = tegra_i2c_pre_init(ctrl_id, hi2c->hi2c_tegra);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: tegra_i2c_pre_init failed, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    hi2c->pre_init_done = true;

out:
    return ret;
}

error_t i2c_controller_init(uint32_t ctrl_id, struct i2c_handle *hi2c)
{
    error_t ret;

    if (ctrl_id > I2C_MAX_CTRLS) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid ctrl_id - %lu, allowed MAX - %u, ret - 0x%x\n",
                        __func__, ctrl_id, I2C_MAX_CTRLS, (int32_t)ret);
        goto out;
    }

    if (hi2c != NULL) {
        if (hi2c->enabled) {
            i2c_pr_debug("%s: i2c controller already enabled\n", __func__);
            ret = E_SUCCESS;
            goto out;
        }
    } else {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: hi2c not declared, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    if (!hi2c->pre_init_done) {
        ret = i2c_controller_pre_init(ctrl_id, hi2c);
        if (ret != E_SUCCESS) {
            i2c_pr_error("%s: i2c_controller_pre_init failed, ret - 0x%x\n",
                            __func__, (int32_t)ret);
            goto out;
        }
    }

    ret = tegra_i2c_init(ctrl_id, hi2c->hi2c_tegra, &hi2c->hw_ops);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: tegra_i2c_init failed, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    i2c_pr_info("%s: i2c controller init success\n", __func__);
    hi2c->ctrl_id = ctrl_id;
    hi2c->enabled = true;
out:
    return ret;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_8_7, "Approval: Bug 200531999, DR: SWE-FSP-015-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
