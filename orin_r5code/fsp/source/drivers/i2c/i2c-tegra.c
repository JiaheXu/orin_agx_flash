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
#include <stdbool.h>                      // for false, true, bool
#include <stddef.h>                       // for NULL
#include <stdint.h>                       // for uint32_t, uint8_t, UINT32_C

/* Early FSP headers */
#include <misc/ct-assert.h>               // for CT_ASSERT

/* Hardware headers */
#include <soc-common/hw-const.h>
#include <ari2c.h>                        // for I2C_I2C_INTERFACE_TIMING_0_...

/* Late FSP headers */
#include <error/common-errors.h>          // for E_SUCCESS, E_INVALID_PARAM,...
#include <misc/attributes.h>              // for UNUSED
#include <misc/macros.h>                  // for START_RFD_BLOCK, END_RFD_BLOCK, INLINE_RFD
#include <misc/nvrm_drf.h>                // for NV_FLD_SET_DRF_NUM, NV_DRF_NUM
#include <reg-access/reg-access.h>        // for readl, readl_base_offset

/* Module-specific FSP headers */
#include <i2c/i2c.h>                      // for I2C_BUS_BUSY, I2C_SCL_BUSY
#include <i2c/i2c-config.h>               // for i2c_config_data
#include <i2c/i2c-errors.h>               // for E_I2C_IO, E_I2C_ARB_LOST
#include <i2c/i2c-hw-ops.h>               // for FSP__I2C__I2C_HW_OPS_H, i2c...
#include <i2c/i2c-tegra.h>                // for i2c_tegra_handle, i2c_xfer_msg
#include <processor/i2c-tegra-hw.h>       // for i2c_hw_handle, i2c_get_hw_h...
#include <soc-common/i2c-defs.h>          // for I2C_TX_FIFO, I2C_FIFO_CONTROL
#include <soc-common/i2c-dependencies.h>  // for appfw_udelay, appfw_timer_c...

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
                MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx",
                MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
CT_ASSERT(FSP__ERROR__COMMON_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__MACROS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_CONFIG_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_ERRORS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_HW_OPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__I2C__I2C_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__PROCESSOR__I2C_TEGRA_HW_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEFS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__SOC_COMMON__I2C_DEPENDENCIES_H, "Header file missing or invalid.")
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx")

/* Custom Defines */
#define CONV_TO_WORD_CEIL(x)            ((((x) % 4U) != 0U) ? \
                                            (((x) / 4U) + 1U) : ((x) / 4U))

/* The granularity of all timing parameters is US(Microseconds) */
#define I2C_SECONDS_TO_US               1000000ULL
#define I2C_REG_POLL_DELAY_US           1U
#define I2C_REG_POLL_TIMEOUT_US         (2U * 1000U)
#define I2C_FIFO_MAX_SIZE_B             512U
#define I2C_FIFO_MAX_SIZE_W             128U
#define I2C_PACKET_HEADER_SIZE_W        3U
#define WORD_TO_BYTES                   4U

#define I2C_CLK_RATE_SM                 100000U
#define I2C_CLK_RATE_FM                 400000U
#define I2C_CLK_RATE_FMPLUS             1000000U
#define I2C_CLK_RATE_HS                 3400000U

#define RESET_DELAY                     5U
#define MSG_END_STOP                    0U
#define MSG_END_REPEAT_START            1U
#define MSG_END_CONTINUE                2U
#define MSG_END_NULL                    3U

#define TEGRA_I2C_ISR_RET_MAGIC         0xBABA

static error_t tegra_i2c_hw_init(const struct i2c_tegra_handle *hi2c_tegra);

/* msg is a read operation */
#define msg_is_read(msg) (((uint32_t)(msg)->xfer_flags & I2C_XFER_FLAG_RD) != 0U)

/* msg does not use repeated start for the transfer */
#define msg_nostart(msg) (((uint32_t)(msg)->xfer_flags & I2C_XFER_FLAG_NOSTART) != 0U)

/* msg continues without ACK from slave */
#define msg_ignore_nak(msg) (((uint32_t)(msg)->xfer_flags & I2C_XFER_FLAG_IGNORE_NAK) != 0U)

#define MASKED_REG_READ(x, mask) ((readl(x)) & (mask))
END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_5, "Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx")

/* Struct Definitions and Declarations */

/* API definitions */
static uint32_t tegra_i2c_timeout_remaining(uint32_t timeout, uint64_t start)
{
    uint64_t timer_elapsed = 0ULL;
    uint32_t ret = 0U;

    timer_elapsed = appfw_timer_cur_us() - start;
    if (timer_elapsed <= (uint64_t)timeout) {
        ret = timeout - ((uint32_t)timer_elapsed);
    }

    return ret;
}

static uint32_t tegra_i2c_readl(const struct i2c_hw_handle *hi2c_hw,
            uint32_t reg)
{
    return readl_base_offset(hi2c_hw->base, reg);
}

static void tegra_i2c_writel(const struct i2c_hw_handle *hi2c_hw, uint32_t val,
            uint32_t reg)
{
    writel_base_offset(val, hi2c_hw->base, reg);
}

static void tegra_i2c_mask_irq(const struct i2c_hw_handle *hi2c_hw, uint32_t mask)
{
    uint32_t int_mask = tegra_i2c_readl(hi2c_hw, I2C_INTERRUPT_MASK_REGISTER_0);

    int_mask &= ~mask;
    tegra_i2c_writel(hi2c_hw, int_mask, I2C_INTERRUPT_MASK_REGISTER_0);
}

static void tegra_i2c_unmask_irq(const struct i2c_hw_handle *hi2c_hw, uint32_t mask)
{
    uint32_t int_mask = tegra_i2c_readl(hi2c_hw, I2C_INTERRUPT_MASK_REGISTER_0);

    int_mask |= mask;
    tegra_i2c_writel(hi2c_hw, int_mask, I2C_INTERRUPT_MASK_REGISTER_0);
}

static void tegra_i2c_modifyl(const struct i2c_hw_handle *hi2c_hw, uint32_t reg,
             uint32_t clr, uint32_t set)
{
    uint32_t value = readl_base_offset(hi2c_hw->base, reg);
    value &= ~clr;
    value |= set;
    writel_base_offset(value, hi2c_hw->base, reg);
}

static error_t tegra_i2c_poll_reg(const uint32_t address, const uint32_t mask,
                const uint32_t value, uint32_t timeout)
{
    INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
    error_t ret = E_FAULT;
    uint64_t timer_start;
    uint64_t timer_cur;

    timer_start = appfw_timer_cur_us();
    do {
        if (MASKED_REG_READ(address, mask) == value) {
            ret = E_SUCCESS;
            break;
        }
        /*
         * NOTE: no yield in interrupt state
         * 10 times polled in the given timeout in non-interrupt mode.
         */
        appfw_udelay(I2C_REG_POLL_DELAY_US);
        timer_cur = appfw_timer_cur_us();
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    } while ((timer_cur - timer_start) < timeout);

    if (ret != E_SUCCESS) {
        if (MASKED_REG_READ(address, mask) == value) {
            ret = E_SUCCESS;
        }
    }

    return ret;
}

static error_t tegra_i2c_clock_conf(const struct i2c_tegra_handle *hi2c_tegra)
{
    int64_t  ret;
    uint64_t val = 0U;
    uint64_t t_val;
    uint32_t clk_divider, x;
    uint32_t tlow, thigh, reg;
    const struct i2c_config_data *pi2c_config = hi2c_tegra->pi2c_config;
    const struct i2c_hw_handle *hi2c_hw = hi2c_tegra->hi2c_hw;

    ret = appfw_configure_ctrl_clk_rate(hi2c_hw->ctrl_id);
    if (ret < 0) {
        i2c_pr_error("%s: failed to configure requested clock rate, ret - %x\n",
                        __func__, (int32_t)ret);
        goto out;
    }
    val = (uint64_t)ret;

    /* Read tlow and thigh from hardware */
    reg = tegra_i2c_readl(hi2c_hw, I2C_I2C_INTERFACE_TIMING_0_0);
    tlow = NV_DRF_VAL(I2C, I2C_INTERFACE_TIMING_0, TLOW, reg);
    thigh = NV_DRF_VAL(I2C, I2C_INTERFACE_TIMING_0, THIGH, reg);

    /*
     * w.r.t T186_I2C_IAS 11.1.1
     * bus_clk_rate = (source_clk) / (Tlow+Thigh+x)*(N+1)
     * if N > 3 then x = 2
     * if N <=3 then x = 3
     * Solve for N, x with above eq
     * if N > 3
     * => N+1 > 4
     * => (source_clk) / (bus_clk_rate) > 4(Tlow+Thigh)+8
     * If above exp is true => x = 2 and N can be calculated.
     * Similarly for N <= 3
     */
    x = ((val / pi2c_config->bus_clk_rate) > ((4U * (tlow + thigh)) + 8U))
        ? 2U : 3U;

    /*
     * clk_divider value is rounded up so that we do not
     * exceed the bus_clk_rate value
     */
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    t_val = (val /
                (((uint64_t)tlow + thigh + x) *
                (uint64_t)pi2c_config->bus_clk_rate));
    if (t_val > UINT32_MAX) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid t_val param\n", __func__);
        goto out;
    }
    clk_divider = (uint32_t)t_val;
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    if ((val % (((uint64_t)tlow + thigh + x) *
                (uint64_t)pi2c_config->bus_clk_rate)) != 0U) {
        if (clk_divider != UINT32_MAX) {
            clk_divider++;
        }
    }

    /*
     clk dvisor is n+1 divisor, write n to the I2C_CLK_DIVISOR
     register, hw increments it by 1.
    */
    if (clk_divider != 0U) {
        clk_divider--;
    }

    if (pi2c_config->bus_clk_rate <= I2C_CLK_RATE_FMPLUS) {
        clk_divider <<= I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT;
    }

    tegra_i2c_modifyl(hi2c_hw, I2C_CLK_DIVISOR, I2C_CLK_DIVISOR_STD_FAST_MASK,
                            clk_divider);

    tegra_i2c_writel(hi2c_hw, I2C_MST_CORE_CLKEN_OVR, I2C_CLKEN_OVERRIDE);

out:
    if (ret >= 0) {
        ret = E_SUCCESS;
    }

    return (error_t)ret;
}

static error_t tegra_i2c_flush_fifos(const struct i2c_hw_handle *hi2c_hw)
{
    error_t err;

    tegra_i2c_modifyl(hi2c_hw, I2C_FIFO_CONTROL, 0U,
             I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH);

    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    err = tegra_i2c_poll_reg(hi2c_hw->base + I2C_FIFO_CONTROL,
            I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH,
            0U, I2C_REG_POLL_TIMEOUT_US);
    if (err != E_SUCCESS) {
    }

    return err;
}

static error_t tegra_i2c_config_load(const struct i2c_hw_handle *hi2c_hw)
{
    error_t err;
    uint32_t val;

    val = NV_DRF_NUM(I2C, I2C_CONFIG_LOAD, MSTR_CONFIG_LOAD, 1U);
    tegra_i2c_writel(hi2c_hw, val, I2C_I2C_CONFIG_LOAD_0);

    /** I2C IAS section 7.1.1.5:
     *
     * SW has to wait until these bits are auto-cleared before
     * going for any further programming. The wait time is not
     * more than 100ns for apb_clk = 100MHz and i2c_clk =
     * 100MHz. And at apb_clk = 20MHz and i2c_clk = 20MHz, it is
     * not more than 500ns time.
     */
    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
    err = tegra_i2c_poll_reg(hi2c_hw->base + I2C_I2C_CONFIG_LOAD_0,
            I2C_I2C_CONFIG_LOAD_0_MSTR_CONFIG_LOAD_FIELD, 0U, I2C_REG_POLL_TIMEOUT_US);
    if (err != E_SUCCESS) {
    }

    return err;
}

static void tegra_i2c_master_reset(const struct i2c_hw_handle *hi2c_hw)
{
    tegra_i2c_modifyl(hi2c_hw, I2C_MSTR_RESET, 0U, I2C_MSTR_SOFT_RESET);
    appfw_udelay(2);
    tegra_i2c_modifyl(hi2c_hw, I2C_MSTR_RESET, I2C_MSTR_SOFT_RESET, 0);
    appfw_udelay(2);
}

static error_t tegra_i2c_read_rx_msg(struct i2c_tegra_handle *hi2c_tegra,
                                      const struct i2c_xfer_msg *msg)
{
    error_t ret = E_SUCCESS;
    uint32_t i = 0U;
    uint32_t cnt;
    uint32_t val;
    uint32_t rx_index = hi2c_tegra->rx_index;

    /* check that RX fifo has (at least) expected amount of data */
    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_MST_FIFO_STATUS_0);
    cnt = NV_DRF_VAL(I2C, MST_FIFO_STATUS, RX_FIFO_FULL_CNT, val);

    cnt = min(cnt, CONV_TO_WORD_CEIL(hi2c_tegra->bytes_to_xfer));

    /* Stop when required number of (4 words) are read */
    while (i < cnt) {
        val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_I2C_RX_FIFO_0);

        if (rx_index < msg->buf_len) {
            msg->prbuf[rx_index] = (uint8_t)(val & 0xffU);
            ++rx_index;
        }
        if (rx_index < msg->buf_len) {
            msg->prbuf[rx_index] = (uint8_t)((val >> 8U) & 0xffU);
            ++rx_index;
        }
        if (rx_index < msg->buf_len) {
            msg->prbuf[rx_index] = (uint8_t)((val >> 16U) & 0xffU);
            ++rx_index;
        }
        if (rx_index < msg->buf_len) {
            msg->prbuf[rx_index] = (uint8_t)((val >> 24U) & 0xffU);
            ++rx_index;
        }
        ++i;
    }

    hi2c_tegra->rx_index = rx_index;
    if (rx_index > msg->buf_len) {
        ret = E_I2C_RX_FIFO_ERROR;
        i2c_pr_error("%s: failed to read index %lu, buf_len - %hu, ret - 0x%x\n",
                __func__, rx_index, msg->buf_len, (int32_t)ret);
    } else {
        hi2c_tegra->bytes_to_xfer = msg->buf_len - rx_index;
    }

    return ret;
}

static void tegra_i2c_send_packet_hdr(const struct i2c_tegra_handle *hi2c_tegra,
                 const struct i2c_xfer_msg *xfer_msg,
                 uint32_t msg_end)
{
    uint32_t xfer_packet_header;
    uint32_t xfer_payload_size;
    uint32_t xfer_io_header;

    xfer_packet_header =
        PACKET_HEADER0_PROTOCOL_I2C
        | (hi2c_tegra->hi2c_hw->ctrl_id << PACKET_HEADER0_CONT_ID_SHIFT)
        | (1UL << PACKET_HEADER0_PACKET_ID_SHIFT);

    INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-451, DR: SWE-FSP-046-SWSADR.docx");
    xfer_payload_size = xfer_msg->buf_len - 1UL;

    xfer_io_header = I2C_HEADER_IE;
    xfer_io_header |=
        ((msg_end == MSG_END_CONTINUE) ?
          I2C_HEADER_CONTINUE_XFER :
          ((msg_end == MSG_END_REPEAT_START) ?
            I2C_HEADER_REPEAT_START : 0U));

    xfer_io_header |=
        (((xfer_msg->xfer_flags & I2C_XFER_FLAG_TEN) != 0U) ?
          (uint32_t)(xfer_msg->dev_addr | I2C_HEADER_10BIT_ADDR) :
          ((uint32_t)xfer_msg->dev_addr << I2C_HEADER_SLAVE_ADDR_SHIFT));

    xfer_io_header |=
        ((msg_ignore_nak(xfer_msg)) ? I2C_HEADER_CONT_ON_NAK : 0U);

    xfer_io_header |=
        ((msg_is_read(xfer_msg)) ? I2C_HEADER_READ : 0U);

    xfer_io_header |=
        ((hi2c_tegra->pi2c_config->bus_clk_rate == I2C_CLK_RATE_HS)
            ? (I2C_HEADER_HIGHSPEED_MODE |
                ((uint32_t)(hi2c_tegra->pi2c_config->hs_master_code & 0x7U)
                << I2C_HEADER_MASTER_ADDR_SHIFT))
            : 0U);

    tegra_i2c_writel(hi2c_tegra->hi2c_hw, xfer_packet_header, I2C_TX_FIFO);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, xfer_payload_size, I2C_TX_FIFO);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, xfer_io_header, I2C_TX_FIFO);
}

/** Transfer i2c_xfer_msg write data to TX_PACKET_FIFO */
static void tegra_i2c_send_tx_data(struct i2c_tegra_handle *hi2c_tegra,
                                   const struct i2c_xfer_msg *msg)
{
    uint32_t i = 0U;
    uint32_t cnt;
    uint32_t val = 0U;
    uint32_t tx_index = hi2c_tegra->tx_index;

    cnt = min(msg->buf_len, 512U);

    /* Stop when required number of (4 words) are written */
    while (i < CONV_TO_WORD_CEIL(cnt)) {
        if (tx_index < msg->buf_len) {
            val = msg->pwbuf[tx_index];
            ++tx_index;
        }
        if (tx_index < msg->buf_len) {
            val |= (uint32_t)msg->pwbuf[tx_index] << 8U;
            ++tx_index;
        }
        if (tx_index < msg->buf_len) {
            val |= (uint32_t)msg->pwbuf[tx_index] << 16U;
            ++tx_index;
        }
        if (tx_index < msg->buf_len) {
            val |= (uint32_t)msg->pwbuf[tx_index] << 24U;
            ++tx_index;
        }
        ++i;

        tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_I2C_TX_PACKET_FIFO_0);
    }

    hi2c_tegra->tx_index = tx_index;
    hi2c_tegra->bytes_to_xfer = msg->buf_len - tx_index;
}

static uint32_t msg_end_type(const struct i2c_xfer_msg *next)
{
    uint32_t msg_end;

    if (next == NULL) {
        msg_end = MSG_END_STOP;
    } else if (msg_nostart(next)) {
        msg_end = MSG_END_CONTINUE;
    } else {
        msg_end = MSG_END_REPEAT_START;
    }

    return msg_end;
}

static error_t tegra_i2c_conf(const struct i2c_tegra_handle *hi2c_tegra)
{
    error_t err;
    uint32_t val;

    val = I2C_CNFG_NEW_MASTER_FSM | I2C_CNFG_PACKET_MODE_EN |
            I2C_CNFG_MULTI_MASTER_MODE;
    if (hi2c_tegra->pi2c_config->bus_clk_rate <= I2C_CLK_RATE_FM) {
        val |= ((uint32_t)0x2U << I2C_CNFG_DEBOUNCE_CNT_SHIFT);
    }

    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_CNFG_REG);

    err = tegra_i2c_flush_fifos(hi2c_tegra->hi2c_hw);
    if (err != E_SUCCESS) {
        i2c_pr_error("%s: failed to flush fifos, ret - 0x%x\n", __func__,
                        (int32_t)err);
        goto out;
    }

    err = tegra_i2c_config_load(hi2c_tegra->hi2c_hw);
    /* The following test technically isn't needed because the return
     * value falls through to return err.  However, without this test,
     * MISRA generates a Directive 4.7 violation (required). */
    if (err != E_SUCCESS) {
        i2c_pr_error("%s: tegra_i2c_config_load failed\n", __func__);
        goto out;
    }

    tegra_i2c_writel(hi2c_tegra->hi2c_hw, 0U, I2C_INTERRUPT_MASK_REGISTER_0);
out:
    return err;
}

static void tegra_i2c_arb_lost_recovery(const struct i2c_tegra_handle *hi2c_tegra)
{
    uint32_t v = 0;

    /* NOTE: all error return values in this function are
     * intentionally suppressed. The I2C transaction is already
     * failed. This routine tries to recover the bus with best effort
     * principle */

    if (!hi2c_tegra->hi2c_hw->is_multimaster) {
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
        UNUSED((tegra_i2c_hw_init(hi2c_tegra)));

        v |= NV_DRF_NUM(I2C, I2C_BUS_CLEAR_CONFIG, BC_SCLK_THRESHOLD, 9U);
        v |= NV_DRF_NUM(I2C, I2C_BUS_CLEAR_CONFIG, BC_STOP_COND, 1U);
        v |= NV_DRF_NUM(I2C, I2C_BUS_CLEAR_CONFIG, BC_TERMINATE, 1U);

        tegra_i2c_writel(hi2c_tegra->hi2c_hw, v, I2C_I2C_BUS_CLEAR_CONFIG_0);
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
        UNUSED((tegra_i2c_config_load(hi2c_tegra->hi2c_hw)));
        v |= NV_DRF_NUM(I2C, I2C_BUS_CLEAR_CONFIG, BC_ENABLE, 1U);
        tegra_i2c_writel(hi2c_tegra->hi2c_hw, v, I2C_I2C_BUS_CLEAR_CONFIG_0);
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
        UNUSED((tegra_i2c_config_load(hi2c_tegra->hi2c_hw)));

        appfw_udelay(10U);
        tegra_i2c_writel(hi2c_tegra->hi2c_hw, I2C_INT_BUS_CLEAR_DONE, I2C_INTERRUPT_STATUS_REGISTER_0);
    }
}

/*
 * checks that msgs are valid
 * ret E_INVALID_PARAM - Invalid parameters
 * ret 0               - SUCCESS
 */
static error_t tegra_i2c_validate_msgs(const struct i2c_xfer_msg *msgs, uint32_t num)
{
    error_t ret = 1;
    uint32_t i;
    const struct i2c_xfer_msg *msg;

    if ((msgs == NULL) || (num == 0U)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params, ret - 0x%x\n", __func__,
                    (msgs == NULL) ? "msgs" : "num", (int32_t)ret);
        goto out;
    }

    for (i = 0U; i < num; ++i) {
        msg = &msgs[i];
        if (msg->buf_len == 0U) {
            INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
            ret = E_INVALID_PARAM;
            i2c_pr_error("%s: msg[%lu] validation failed, ret - 0x%x\n",
                            __func__, i, (int32_t)ret);
            break;
        }
    }

out:
    return ret;
}

static error_t tegra_i2c_acquire(const struct i2c_tegra_handle *hi2c_tegra)
{
    error_t err;

    /* In case of suspended state i2c-acquire should fail */
    if (hi2c_tegra->is_suspended) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_INVALID_STATE;
        i2c_pr_error("%s: controller in suspended state, ret - 0x%x\n", __func__,
                        (int32_t)err);
        goto out;
    }

    err = appfw_clk_enable(hi2c_tegra->ctrl_id);

out:
    return err;
}

static error_t tegra_i2c_release(const struct i2c_tegra_handle *hi2c_tegra)
{
    return appfw_clk_disable(hi2c_tegra->ctrl_id);
}

struct tegra_i2c_prod {
    uint32_t tlow;
    uint32_t thigh;
    const uint32_t tsu_sta;
    const uint32_t thd_sta;
    const uint32_t tsu_sto;
    const uint32_t tbuf;
    const uint32_t thd_dat;
    const uint32_t mstr_tcapture;
    const uint32_t slv_tcapture;
};

START_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")
#define I2C_PRODS(_mode)                        \
    .tlow = I2C_I2C_INTERFACE_TIMING_0_0_TLOW__PROD_C_##_mode,  \
    .thigh = I2C_I2C_INTERFACE_TIMING_0_0_THIGH__PROD_C_##_mode, \
    .tsu_sta = I2C_I2C_INTERFACE_TIMING_1_0_TSU_STA__PROD_C_##_mode, \
    .thd_sta = I2C_I2C_INTERFACE_TIMING_1_0_THD_STA__PROD_C_##_mode, \
    .tsu_sto = I2C_I2C_INTERFACE_TIMING_1_0_TSU_STO__PROD_C_##_mode, \
    .tbuf = I2C_I2C_INTERFACE_TIMING_1_0_TBUF__PROD_C_##_mode, \
    .thd_dat = I2C_I2C_INTERFACE_TIMING_2_0_THD_DAT__PROD,  \
    .mstr_tcapture = I2C_I2C_MSTR_DATA_CAPTURE_TIMING_0_TCAPTURE_DAT__PROD, \
    .slv_tcapture = I2C_I2C_SLV_DATA_CAPTURE_TIMING_0_TCAPTURE_DAT__PROD,
END_RFD_BLOCK(MISRA, DEVIATE, Rule_20_10, "Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx")

static void tegra_i2c_apply_prod(const struct i2c_tegra_handle *hi2c_tegra)
{
    struct tegra_i2c_prod const *prod;
    uint32_t val;

    static struct tegra_i2c_prod prod_sm = {
        I2C_PRODS(SM)
    };

    static struct tegra_i2c_prod prod_fm = {
        I2C_PRODS(FM)
    };

    static struct tegra_i2c_prod prod_fmplus = {
        I2C_PRODS(FMPLUS)
    };

    if (hi2c_tegra->pi2c_config->bus_clk_rate <= I2C_CLK_RATE_SM) {
        prod = &prod_sm;
    } else if (hi2c_tegra->pi2c_config->bus_clk_rate <= I2C_CLK_RATE_FM) {
        prod = &prod_fm;
    } else {
        prod = &prod_fmplus;
    }

#ifdef CUSTOM_TIMING_PARAMS
    /* Overwrite the timing parameters from FW configuration if valid */
    if (hi2c_tegra->pi2c_config->thigh != UINT32_MAX) {
        prod->thigh = hi2c_tegra->pi2c_config->thigh;
    }
    if (hi2c_tegra->pi2c_config->tlow != UINT32_MAX) {
        prod->tlow = hi2c_tegra->pi2c_config->tlow;
    }
#endif

    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_I2C_INTERFACE_TIMING_0_0);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_0, TLOW,
                                prod->tlow, val);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_0, THIGH,
                                prod->thigh, val);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_I2C_INTERFACE_TIMING_0_0);

    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_I2C_INTERFACE_TIMING_1_0);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_1, TSU_STA,
                                prod->tsu_sta, val);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_1, THD_STA,
                                prod->thd_sta, val);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_1, TSU_STO,
                                prod->tsu_sto, val);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_1, TBUF,
                                prod->tbuf, val);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_I2C_INTERFACE_TIMING_1_0);

    /* following registers are only in >t194 */
    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_I2C_INTERFACE_TIMING_2_0);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_INTERFACE_TIMING_2, THD_DAT,
                                prod->thd_dat, val);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_I2C_INTERFACE_TIMING_2_0);

    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_I2C_MSTR_DATA_CAPTURE_TIMING_0);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_MSTR_DATA_CAPTURE_TIMING, TCAPTURE_DAT,
                                prod->mstr_tcapture, val);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_I2C_MSTR_DATA_CAPTURE_TIMING_0);

    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_I2C_SLV_DATA_CAPTURE_TIMING_0);
    val = NV_FLD_SET_DRF_NUM(I2C, I2C_SLV_DATA_CAPTURE_TIMING, TCAPTURE_DAT,
                                prod->slv_tcapture, val);
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_I2C_SLV_DATA_CAPTURE_TIMING_0);
}

static error_t tegra_i2c_hw_init(const struct i2c_tegra_handle *hi2c_tegra)
{
    error_t err;

    err = appfw_clk_reset_pulse(hi2c_tegra->ctrl_id, RESET_DELAY);
    if (err != E_SUCCESS) {
        i2c_pr_error("%s: appfw_clk_reset_pulse failed, ret - 0x%x\n", __func__,
                        (int32_t)err);
        goto out;
    }

    tegra_i2c_apply_prod(hi2c_tegra);

    err = tegra_i2c_clock_conf(hi2c_tegra);
    if (err != E_SUCCESS) {
        i2c_pr_error("%s: tegra_i2c_clock_conf failed, ret - 0x%x\n", __func__,
                        (int32_t)err);
        goto out;
    }

    err = tegra_i2c_conf(hi2c_tegra);
    if (err != E_SUCCESS) {
        i2c_pr_error("%s: tegra_i2c_conf failed, ret - 0x%x\n", __func__,
                        (int32_t)err);
        goto out;
    }

    /* Programming trigger levels to 1 word. */
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, 0U, I2C_FIFO_CONTROL);

    if (!hi2c_tegra->is_initialized) {
        tegra_i2c_master_reset(hi2c_tegra->hi2c_hw);
    }

out:
    return err;
}

static uint32_t tegra_i2c_intr_status(const struct i2c_tegra_handle *hi2c_tegra)
{
    uint32_t val;

    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_INTERRUPT_STATUS_REGISTER_0);
    /* Discard FIFO status interrupt bits, they are always
     * masked and irrelevant to the logic of this driver */
    val = NV_FLD_SET_DRF_NUM(I2C, INTERRUPT_STATUS_REGISTER, TFIFO_DATA_REQ, 0U, val);
    val = NV_FLD_SET_DRF_NUM(I2C, INTERRUPT_STATUS_REGISTER, RFIFO_DATA_REQ, 0U, val);

    return val;
}

/** check that i2c controller is at expected state before xfer is attempted */
static error_t tegra_i2c_check_state(const struct i2c_tegra_handle *hi2c_tegra)
{
    error_t ret = E_SUCCESS;
    uint32_t val;

    /* expect that TX and RX fifos are empty */
    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_MST_FIFO_STATUS_0);
    if (val != NV_RESETVAL(I2C, MST_FIFO_STATUS)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_FIFO_STATUS;
        i2c_pr_error("%s: TX/RX fifos not empty, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    /* no interrupt should be pending */
    if (tegra_i2c_intr_status((const struct i2c_tegra_handle *)hi2c_tegra)
                    != 0U) {
        ret = E_I2C_STALE_INT;
        i2c_pr_error("%s: stale interrupts present, ret - 0x%x\n", __func__,
                        (int32_t)ret);
    }
out:
    return ret;
}

static void tegra_i2c_disable_packet_mode(const struct i2c_tegra_handle *hi2c_tegra)
{
    uint32_t val;

    val = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_CNFG_REG);
    val = val & ~I2C_CNFG_PACKET_MODE_EN;
    tegra_i2c_writel(hi2c_tegra->hi2c_hw, val, I2C_CNFG_REG);
    /* We will reinit hw again and the error returned can be ignored */
    INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-383, DR: SWE-FSP-004-SWSADR.docx");
    UNUSED(tegra_i2c_config_load(hi2c_tegra->hi2c_hw));
}

INLINE_RFD(MISRA, DEVIATE, Rule_8_7, "Approval: JIRA TID-338, DR: SWE-FSP-008-SWSADR.docx")
void tegra_i2c_irq_handler(uint32_t ctrl_id)
{
    struct i2c_tegra_handle *hi2c_tegra = i2c_get_tegra_handle(ctrl_id);
    const struct i2c_xfer_msg *cur = hi2c_tegra->cur_msg;
    error_t ret = E_SUCCESS;
    uint32_t status;
    bool end_xfer = false;

    status = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_INTERRUPT_STATUS_REGISTER_0);
    i2c_pr_debug("%s: interrupt status reg - 0x%lx\n", __func__, status);

    if ((status & I2C_INT_NO_ACK) != 0U) {
        tegra_i2c_disable_packet_mode(hi2c_tegra);
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_NACK;
        i2c_pr_error("%s: NACK error - Slave not present, ret - 0x%x\n",
                        __func__, (int32_t)ret);
        end_xfer = true;
        goto out;
    }

    if ((status & I2C_INT_ARB_LOST) != 0U) {
        tegra_i2c_arb_lost_recovery(hi2c_tegra);
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_ARB_LOST;
        i2c_pr_error("%s: ARB lost error, ret - 0x%x\n", __func__, ret);
        end_xfer = true;
        goto out;
    }

    if ((msg_is_read(cur)) && ((status & I2C_INT_RX_FIFO_DATA_REQ) != 0U)) {
        if (hi2c_tegra->bytes_to_xfer > 0U) {
            ret = tegra_i2c_read_rx_msg(hi2c_tegra, cur);
            if (ret != E_SUCCESS) {
                i2c_pr_error("%s: failed to read msg from RX FIFO, ret - 0x%x\n",
                                __func__, (int32_t)ret);
                end_xfer = true;
            }
        } else {
            ret = E_I2C_RX_FIFO_UNDERFLOW;
            i2c_pr_error("%s: RX FIFO underflow, ret - 0x%x\n", __func__,
                            (int32_t)ret);
            end_xfer = true;
        }
    }

    if ((!msg_is_read(cur)) && ((status & I2C_INT_TX_FIFO_DATA_REQ) != 0U)) {
        if (hi2c_tegra->bytes_to_xfer > 0U) {
            tegra_i2c_send_tx_data(hi2c_tegra, cur);
            ret = E_SUCCESS;
        } else {
            tegra_i2c_mask_irq(hi2c_tegra->hi2c_hw, I2C_INT_TX_FIFO_DATA_REQ);
        }
    }

    if ((status & (I2C_INT_PACKET_XFER_COMPLETE |
                                    I2C_INT_ALL_PACKETS_XFER_COMPLETE)) != 0U) {
        if (hi2c_tegra->bytes_to_xfer != 0U) {
            ret = E_I2C_XFER_COMPLETE;
        } else {
            ret = E_SUCCESS;
        }
        end_xfer = true;
    }

out:
    if (ret != E_SUCCESS) {
        /* Error occured, mask all interrupts */
        tegra_i2c_mask_irq(hi2c_tegra->hi2c_hw, I2C_INT_NO_ACK |
                        I2C_INT_ARB_LOST | I2C_INT_PACKET_XFER_COMPLETE |
                        I2C_INT_ALL_PACKETS_XFER_COMPLETE |
                        I2C_INT_RX_FIFO_DATA_REQ |
                        I2C_INT_TX_FIFO_DATA_REQ);

    }

    tegra_i2c_writel(hi2c_tegra->hi2c_hw, status, I2C_INTERRUPT_STATUS_REGISTER_0);

    hi2c_tegra->end_xfer = end_xfer;
    hi2c_tegra->isr_ret = ret;
}

static void tegra_i2c_xfer_one_msg(struct i2c_tegra_handle *hi2c_tegra,
                              struct i2c_xfer_msg *cur,
                              const struct i2c_xfer_msg *next)
{
    uint32_t msg_end = msg_end_type(next);
    uint32_t mask = I2C_INT_ARB_LOST |
                    I2C_INT_NO_ACK |
                    I2C_INT_TX_FIFO_OVERFLOW |
                    I2C_INT_RX_FIFO_UNDERFLOW;

    hi2c_tegra->cur_msg = cur;
    hi2c_tegra->bytes_to_xfer = cur->buf_len;

    tegra_i2c_unmask_irq(hi2c_tegra->hi2c_hw, mask);

    tegra_i2c_send_packet_hdr(hi2c_tegra, cur, msg_end);

    if (!msg_is_read(cur)) {
        tegra_i2c_send_tx_data(hi2c_tegra, cur);
    } else {
        mask |= I2C_INT_RX_FIFO_DATA_REQ;
    }

    mask |= I2C_INT_PACKET_XFER_COMPLETE;
    tegra_i2c_unmask_irq(hi2c_tegra->hi2c_hw, mask);
}

static error_t tegra_i2c_poll_isr_ret(const struct i2c_tegra_handle *hi2c_tegra,
                                        uint32_t timeout, uint32_t delay)
{
    error_t ret = E_TIMEOUT;
    uint64_t timer_start = appfw_timer_cur_us();
    uint32_t timeout_remaining = timeout;
#ifdef I2C_POLLING_XFER
    uint32_t status;
    uint32_t int_mask;
#endif

    do {
#ifdef I2C_POLLING_XFER
        appfw_udelay((timeout_remaining == timeout) ? delay :
                                            I2C_REG_POLL_DELAY_US);

        int_mask = tegra_i2c_readl(hi2c_tegra->hi2c_hw,
                                        I2C_INTERRUPT_MASK_REGISTER_0);
        status = tegra_i2c_readl(hi2c_tegra->hi2c_hw,
                                    I2C_INTERRUPT_STATUS_REGISTER_0);
        if ((status & int_mask) != 0U) {
            tegra_i2c_irq_handler(hi2c_tegra->ctrl_id);
            ret = hi2c_tegra->isr_ret;
        }

        if (hi2c_tegra->end_xfer) {
            break;
        }
#else
        /* Pass remaining timeout as available timeout */
        ret = appfw_wait_irq(hi2c_tegra->ctrl_id, timeout_remaining);
        if (ret == E_SUCCESS) {
            ret = hi2c_tegra->isr_ret;
        } else {
            ret = E_TIMEOUT;
        }

        /* delay check is just to avoid misra and cert-c unused var errors */
        if (hi2c_tegra->end_xfer || (delay > timeout)) {
            break;
        }
#endif
        timeout_remaining = tegra_i2c_timeout_remaining(timeout,
                                                            timer_start);
    } while (timeout_remaining > 0U);

    return ret;
}

static error_t tegra_i2c_process_xfer(const struct i2c_tegra_handle *hi2c_tegra,
                                uint32_t timeout, uint32_t delay)
{
    error_t ret = E_SUCCESS;

#ifndef I2C_POLLING_XFER
    appfw_irq_enable(hi2c_tegra->ctrl_id);
#endif
    ret = tegra_i2c_poll_isr_ret(hi2c_tegra, timeout, delay);
#ifndef I2C_POLLING_XFER
    appfw_irq_disable(hi2c_tegra->ctrl_id);
#endif

    if (ret != E_SUCCESS) {
        /* Transer return takes precedence */
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
        UNUSED(tegra_i2c_hw_init(hi2c_tegra));
    }

    return ret;
}

static error_t tegra_i2c_xfer_msg_by_msg(struct i2c_tegra_handle *hi2c_tegra,
                              struct i2c_xfer_msg *msgs, uint32_t num_msgs,
                              uint32_t timeout)
{
    error_t ret;
    uint32_t tmp;
    uint32_t bytes_to_xfer;
    uint16_t buf_len_w;
    const struct i2c_xfer_msg *this;
    const struct i2c_xfer_msg *next;
    uint32_t fifo_trig_words;
    uint32_t fifo_trig = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_FIFO_CONTROL);
    uint64_t timer_start = appfw_timer_cur_us();
    uint32_t timeout_remaining = timeout;
    uint64_t wait_time;

    for (tmp = 0U; tmp < num_msgs; tmp++) {
        /* Set state variables for each message */
        hi2c_tegra->end_xfer = false;
        hi2c_tegra->isr_ret = TEGRA_I2C_ISR_RET_MAGIC;
        hi2c_tegra->rx_index = 0;
        hi2c_tegra->tx_index = 0;

        this = &msgs[tmp];
        next = ((tmp + 1U) < num_msgs) ? &msgs[tmp + 1U] : NULL;

        /* set trigger to one interrupt for all DATA REQ optimisation */
        buf_len_w = CONV_TO_WORD_CEIL(this->buf_len);
        fifo_trig_words = min(buf_len_w, I2C_FIFO_MAX_SIZE_W);
        if (!msg_is_read(this)) {
            fifo_trig &= fifo_trig_words << I2C_FIFO_CONTROL_TX_TRIG_SHIFT;
        } else {
            fifo_trig &= fifo_trig_words << I2C_FIFO_CONTROL_RX_TRIG_SHIFT;
        }
        tegra_i2c_writel(hi2c_tegra->hi2c_hw, fifo_trig, I2C_FIFO_CONTROL);

        tegra_i2c_xfer_one_msg(hi2c_tegra, &msgs[tmp], next);

        timeout_remaining = tegra_i2c_timeout_remaining(timeout,
                                                            timer_start);

        /*
         * Wait time(in us) for data to be written on bus.Adding the packet
         * header bytes in the calculation to account for HW delays in transfer
         * initiation. The time needed for transferring 1B of data is 9 clock
         * cycles.
         *
         * Note: This is not ideal wait time, but fixes most of the timing
         * mismatches in HW while transferring more than one message. The
         * trade-off on performance for the extra delay is negligible.
         */
        bytes_to_xfer = (I2C_PACKET_HEADER_SIZE_W + fifo_trig_words) *
                            WORD_TO_BYTES;
        wait_time = DIV_ROUND_UP(I2C_SECONDS_TO_US * bytes_to_xfer,
                            hi2c_tegra->pi2c_config->bus_clk_rate);
        INLINE_RFD(CERTC, DEVIATE, INT30_C, "Approval: JIRA TID-449, DR: SWE-FSP-045-SWSADR.docx");
        wait_time *= 9U;

        if ((timeout_remaining > 0U) &&
                (wait_time < (uint64_t)timeout_remaining)) {
            ret = tegra_i2c_process_xfer(hi2c_tegra, timeout_remaining,
                                         (uint32_t)wait_time);
        } else {
            ret = E_TIMEOUT;
            i2c_pr_error("%s: transfer timedout. msgs xfered(%lu)."
                   " remaining(%lu), ret - 0x%x\n",
                            __func__, tmp, num_msgs - tmp, (int32_t)ret);
        }

        if (ret != E_SUCCESS) {
            i2c_pr_error("%s: msg[%lu] failed to xfer, ret - 0x%x\n", __func__,
                            tmp, (int32_t)ret);
            break;
        }
    }

    return ret;
}

static error_t tegra_i2c_xfer(struct i2c_tegra_handle *hi2c_tegra,
                              struct i2c_xfer_msg *msgs, uint32_t num_msgs,
                              uint32_t timeout)
{
    error_t ret;
    error_t tmp;

    ret = tegra_i2c_acquire(hi2c_tegra);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: failed to acquire controller, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    ret = tegra_i2c_check_state(hi2c_tegra);
    if (ret != E_SUCCESS) {
        /* Assuming the worst possible scenario in the previous transfer */
        INLINE_RFD(MISRA, DEVIATE, Directive_4_7, "Approval: JIRA TID-1795, DR: SWE-FSP-062-SWSADR.docx");
        UNUSED((tegra_i2c_hw_init(hi2c_tegra)));
        i2c_pr_error("%s: i2c hw not ready for transfer, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto release;
    }

    ret = tegra_i2c_validate_msgs(msgs, num_msgs);
    if (ret < 0) {
        i2c_pr_error("%s: msgs validation failed, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto release;
    }

    ret = tegra_i2c_xfer_msg_by_msg(hi2c_tegra, msgs, num_msgs,
                                        timeout);

release:
    tmp = tegra_i2c_release(hi2c_tegra);
    if (tmp != E_SUCCESS) {
        i2c_pr_error("%s: failed to release controller, ret - 0x%x\n", __func__,
                        (int32_t)tmp);
        ret = (ret == E_SUCCESS) ? tmp : ret;
    }

out:
    return ret;
}

static error_t tegra_i2c_get_bus_state(const struct i2c_tegra_handle *hi2c_tegra,
                    uint32_t *state)
{
    error_t ret;
    uint32_t bus_status;
    *state = 0U;

    ret = tegra_i2c_acquire(hi2c_tegra);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: failed to acquire controller, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    bus_status = tegra_i2c_readl(hi2c_tegra->hi2c_hw, I2C_BUS_STATUS);

    ret = tegra_i2c_release(hi2c_tegra);
    if (ret != E_SUCCESS) {
        i2c_pr_error("%s: failed to release controller, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    if ((bus_status & I2C_BUS_STATUS_SCL_MASK) != 0U) {
        *state |= I2C_SCL_BUSY;
    }

    if ((bus_status & I2C_BUS_STATUS_SDA_MASK) != 0U) {
        *state |= I2C_SDA_BUSY;
    }

    if ((bus_status & I2C_BUS_STATUS_BUSY_MASK) != 0U) {
        *state |= I2C_BUS_BUSY;
    }

out:
    return ret;
}

static error_t tegra_i2c_suspend(struct i2c_tegra_handle *hi2c_tegra)
{
    error_t ret = E_SUCCESS;

    if ((hi2c_tegra == NULL) || (hi2c_tegra->hi2c_hw == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params, ret - 0x%x\n", __func__,
                        (hi2c_tegra == NULL) ? "hi2c_tegra" :
                            "hi2c_tegra->hi2c_hw", (int32_t)ret);
        goto out;
    }

    if (hi2c_tegra->is_suspended) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_STATE;
        i2c_pr_error("%s: controller already suspended, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    hi2c_tegra->is_suspended = true;
out:
    return ret;
}

static error_t tegra_i2c_resume(struct i2c_tegra_handle *hi2c_tegra)
{
    error_t ret = E_SUCCESS;
    error_t tmp = E_SUCCESS;

    if (hi2c_tegra == NULL) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_PARAM;
        i2c_pr_error("%s: hi2c_tegra not declared, ret - 0x%x\n", __func__,
                        (int32_t)ret);
        goto out;
    }

    if (!hi2c_tegra->is_suspended) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_INVALID_STATE;
        i2c_pr_error("%s: controller not in suspended state, ret - 0x%x\n",
                        __func__, (int32_t)ret);
        goto out;
    }

    /* Allow access to HW */
    hi2c_tegra->is_suspended = false;

    ret = tegra_i2c_acquire(hi2c_tegra);
    if (ret == E_SUCCESS) {
        ret = tegra_i2c_hw_init(hi2c_tegra);
    } else {
        i2c_pr_error("%s: failed to acquire controller, ret - 0x%x\n", __func__,
                        (int32_t)ret);
    }

    tmp = tegra_i2c_release(hi2c_tegra);
    if (tmp != E_SUCCESS) {
        i2c_pr_error("%s: failed to release controller, ret - 0x%x", __func__,
                        (int32_t)tmp);
        ret = (ret == E_SUCCESS) ? tmp : ret;
    }

    /*
     * The hw_init return is returned during probe. Hence, in case of resume
     * if hw init fails, returning IO error.
     */
    if(ret != E_SUCCESS) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        ret = E_I2C_IO;
        i2c_pr_error("%s: tegra_i2c_release/tegra_i2c_hw_init failed, "
                        "ret - 0x%x\n", __func__, (int32_t)ret);
    }

out:
    if (hi2c_tegra != NULL) {
        hi2c_tegra->is_suspended = (ret != E_SUCCESS);
    }
    return ret;
}

error_t tegra_i2c_pre_init(uint32_t ctrl_id, struct i2c_tegra_handle *hi2c_tegra)
{
    error_t err = E_SUCCESS;

    hi2c_tegra->ctrl_id = ctrl_id;
    hi2c_tegra->hi2c_hw = i2c_get_hw_handle(ctrl_id);
    hi2c_tegra->pi2c_config = i2c_get_config_data(ctrl_id);

    if ((hi2c_tegra->hi2c_hw == NULL) ||
        (hi2c_tegra->pi2c_config == NULL)) {
        INLINE_RFD(MISRA, FP, Rule_10_8, "Approval: JIRA TID-412, DR: SWE-FSP-021-SWSADR.docx");
        err = E_INVALID_PARAM;
        i2c_pr_error("%s: invalid (%s) params provided, ret - 0x%x\n", __func__,
                    (hi2c_tegra->hi2c_hw == NULL) ? "hi2c_tegra->hi2c_hw" :
                        "hi2c_tegra->pi2c_config", (int32_t)err);
    }

    hi2c_tegra->pre_init_done = true;

    return err;
}

error_t tegra_i2c_init(uint32_t ctrl_id, struct i2c_tegra_handle *hi2c_tegra,
                struct i2c_hw_ops **hw_ops)
{
    error_t err = E_SUCCESS;
    error_t tmp = E_SUCCESS;
    static struct i2c_hw_ops tegra_hw_ops = {
        .xfer = &tegra_i2c_xfer,
        .suspend = &tegra_i2c_suspend,
        .resume = &tegra_i2c_resume,
        .get_bus_state = &tegra_i2c_get_bus_state,
    };

    /* Setting to default values */
    hi2c_tegra->is_initialized = false;
    hi2c_tegra->is_suspended = false;
    *hw_ops = NULL;

    if (!hi2c_tegra->pre_init_done) {
        err = tegra_i2c_pre_init(ctrl_id, hi2c_tegra);
        if (err != E_SUCCESS) {
            i2c_pr_error("%s: tegra_i2c_pre_init failed, ret - 0x%x\n", __func__,
                            (int32_t)err);
            goto out;
        }
    }

    err = tegra_i2c_acquire(hi2c_tegra);
    if (err == E_SUCCESS) {
        err = tegra_i2c_hw_init(hi2c_tegra);
    } else {
        i2c_pr_error("%s: failed to acquire mutex, ret - 0x%x\n", __func__,
                        (int32_t)err);
    }

    tmp = tegra_i2c_release(hi2c_tegra);
    if (tmp != E_SUCCESS) {
        i2c_pr_error("%s: failed to release mutex, ret - 0x%x\n", __func__,
                        (int32_t)tmp);
        err = (err == E_SUCCESS) ? tmp : err;
    }

    if (err == E_SUCCESS) {
        hi2c_tegra->is_initialized = true;
        *hw_ops = &tegra_hw_ops;
    }

out:
    return err;
}
END_RFD_BLOCK(MISRA, DEVIATE, Rule_1_2, "Approval: Bug 200531996, DR: SWE-FSP-009-SWSADR.docx",
              MISRA, DEVIATE, Rule_15_1, "Approval: Bug 3176459, DR: SWE-FSP-028-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
