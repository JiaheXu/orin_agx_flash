/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

/* Compiler headers */
#include <stdbool.h>                   // for bool
#include <stdint.h>                    // for uint32_t, uint8_t, UINT32_...
#include <string.h>                    // for NULL

/* Early FSP headers */
#include <misc/ct-assert.h>            // for CT_ASSERT
#include <soc-common/hw-const.h>       // IWYU pragma: keep

/* Hardware headers */
//#include <aradcc.h>                  // for ADCC_CTL0_0, ADCC_TIMING_CTL0_0...

/* Late FSP headers */
#include <error/common-errors.h>       // for error_t, E_SUCCESS
#include <misc/attributes.h>           // for UNUSED
#include <misc/nvrm_drf.h>             // NV_DRF_* macros
#include <reg-access/reg-access.h>     // for readl_base_offset, writel_base_of...
#include <misc/bitops.h>               // for BIT, bit_number
#include <clk/clk-tegra.h>             // for tegra_clk_enable, ...
#include <soc-common/clk-tegra-hw.h>   // for tegra_clk_adcc_os_undiv, ...

/* Module specific headers */
#include <adcc/adcc.h>                 // for adcc_init, adcc_start_scan...
#include <adcc/adcc-errors.h>          // for E_ADCC_NULL_PTR, E_ADCC_...
#include <adcc/adcc-priv.h>            // for struct adcc_id
#include <adcc/sections-adcc.h>        // Immune from CT_ASSERT protection

#define ADCC_CTL0_0         0x00UL
#define ADCC_TIMING_CTL0_0  0x04UL
#define ADCC_SCAN_SEL_0     0x08UL
#define ADCC_SCAN_WINDOW_0  0x0CUL
#define ADCC_CH0_SAMPLE_0   0x10UL
#define ADCC_CH_OFFSET      0x04UL
#define ADCC_CTL1_0         0x2CUL

#define ADCC_SMPL_VALID     BIT(12U)
#define ADCC_SMPL_VAL_MSK   0x3FFUL
#define ADCC_SCAN_EN        BIT(0U)
#define ADCC_SCAN_SEL_MSK   0x3FUL
#define ADCC_TMG_CTL_MSK    0xFFFFUL
#define ADC_CONV_CYC_SHFT   28U
#define ADCC_PD             BIT(31U)
#define ADCC_CONT           0UL
#define ADCC_ONE_SHOT       BIT(1U)
#define ADCC_WAIT_TIME      0x45UL
#define ADCC_CH_SZ_MASK     0x00FFFFFFUL
#define ADC_WAIT_SEL_SHFT   0x8U

static bool adcc_inited = false;
static uint32_t wait_time = 64U;

static inline uint32_t
adcc_readl(const struct adcc_id * const id,
           uint32_t offset)
{
    return readl_base_offset(id->base_addr, offset);
}

static inline void
adcc_writel(const struct adcc_id * const id,
            uint32_t val,
            uint32_t offset)
{
    writel_base_offset(val, id->base_addr, offset);
}

static inline void
adcc_reset_chans(const struct adcc_id *id)
{
    uint32_t i;
    uint32_t val = 0UL;
    uint32_t ch_base;

    for (i = 0UL; i <= ADCC_NCHANS; i++) {
        ch_base = ADCC_CH0_SAMPLE_0 + (i * ADCC_CH_OFFSET);
        val = adcc_readl(id, ch_base);
        val &= ~ADCC_SMPL_VALID;
        adcc_writel(id, val, ch_base);
    }
}

static void
adcc_configure_control_regs(const struct adcc_id *id,
                            const uint32_t enabled_chans)
{
    uint32_t val;

    val = enabled_chans & ADCC_SCAN_SEL_MSK;
    adcc_writel(id, val, ADCC_SCAN_SEL_0);
}

SECTION_ADCC_TEXT static void
adcc_configure_scan_mode(const struct adcc_id *id, bool continuous)
{
    uint32_t mode;

    mode = continuous ? ADCC_CONT : ADCC_ONE_SHOT;
    adcc_writel(id, mode, ADCC_CTL0_0);
}

SECTION_ADCC_TEXT static uint32_t
adcc_get_wait_time(const struct adcc_id *id,
                   uint32_t sampling_dur)
{
    uint32_t val = 0x0U;

    switch (sampling_dur) {
        case 16U:
            val = 0x0U;
            break;
        case 32U:
            val = 0x0U;
            break;
        case 64U:
            val = 0x0U;
            break;
        case 128U:
            wait_time = 128U;
            val = 0x1U;
            break;
        default:
            break;
    }

    return val;
}

SECTION_ADCC_TEXT static uint32_t
adcc_get_conv_cycle(const struct adcc_id *id,
                    uint32_t sampling_dur)
{
    uint32_t val = 0x1U;

    switch (sampling_dur) {
        case 16U:
            val = 0x0U;
            break;
        case 32U:
            val = 0x1U;
            break;
        case 64U:
            val = 0x2U;
            break;
        case 128U:
            val = 0x3U;
            break;
        default:
            break;
    }

    return val;
}

SECTION_ADCC_TEXT static void
adcc_configure_pad_ctl(const struct adcc_id *id,
                       const struct adcc_conf *conf)
{
    uint32_t val;
    uint32_t conv_cycles;

    /* program the conversion cycles aka paramter C in the IAS */
    val = adcc_readl(id, ADCC_CTL1_0);
    conv_cycles = adcc_get_conv_cycle(id, conf->sampling_dur);
    val |= conv_cycles << ADC_CONV_CYC_SHFT;
    val |= adcc_get_wait_time(id, conf->sampling_dur) << ADC_WAIT_SEL_SHFT;
    adcc_writel(id, val, ADCC_CTL1_0);

    /* program init wait time */
    val = 2U + (wait_time + 3U) * 1U;
    adcc_writel(id, val, ADCC_TIMING_CTL0_0);

    /* program the averaging window of samples */
    val = conf->avg_window & ADCC_CH_SZ_MASK;
    adcc_writel(id, val, ADCC_SCAN_WINDOW_0);
}

SECTION_ADCC_TEXT static void
adcc_clear_power_down(const struct adcc_id *id)
{
    uint32_t val;

    val = adcc_readl(id, ADCC_CTL0_0);
    val &= ~ADCC_PD;
    adcc_writel(id, val, ADCC_CTL0_0);
}

SECTION_ADCC_TEXT static error_t
adcc_configure_clk_rst(const struct adcc_id *id, uint32_t clk_src)
{
    error_t ret;
    const struct tegra_clk  *clk;

    switch (clk_src) {
        case ADCC_CLK_SRC_PLLP:
            clk = tegra_clk_adcc_pll_p;
            ret = tegra_clk_set_rate(clk, 40000000);
            if (ret != E_SUCCESS) {
                ret = E_ADCC_CLK_EN;
                goto out;
            }
            break;
        case ADCC_CLK_SRC_OSC_UNDIV:
        default:
            clk = tegra_clk_adcc_osc_undiv;
            break;
    }

    ret = tegra_clk_enable(clk);
    if (ret != E_SUCCESS) {
        ret = E_ADCC_CLK_EN;
        goto out;
    }

    ret = tegra_clk_reset_pulse(id->rst, 2U);
    if (ret != E_SUCCESS) {
        ret = E_ADCC_CLK_RST;
    }

out:
    return ret;
}

SECTION_ADCC_TEXT error_t
adcc_read_data(const struct adcc_id *id,
               struct adcc_data *data)
{
    error_t   ret = E_SUCCESS;
    uint32_t  ch_base;
    uint32_t  val;
    uint32_t  chan;
    uint32_t  chans;

    if (id == NULL || data == NULL) {
        ret = E_ADCC_NULL_PTR;
        goto out;
    }

    if (!adcc_inited) {
        ret = E_ADCC_NO_INIT;
        goto out;
    }

    chans = adcc_readl(id, ADCC_SCAN_SEL_0) & ADCC_SCAN_SEL_MSK;
    data->enabled_chans = chans;

    /*
     * VLD bit is set for enabled channels in order. Instead of reading
     * every channel's VLD bit, read the last channel of the selected
     * channels. If this indicates VLD, rest of the channels are also
     * expected to as the HW sets them in order.
     */
    chan = 31U - __builtin_clz(chans);
    ch_base = ADCC_CH0_SAMPLE_0 + (chan * ADCC_CH_OFFSET);
    do {
        val = adcc_readl(id, ch_base);
    } while ((val & ADCC_SMPL_VALID) == 0UL);

    /* fetch all enabled channel samples */
    while (chans != 0UL) {
        chan = bit_number(chans);
        ch_base = ADCC_CH0_SAMPLE_0 + (chan * ADCC_CH_OFFSET);
        val = adcc_readl(id, ch_base);
        data->ch_data[chan] = val & ADCC_SMPL_VAL_MSK;
        val &= ~ADCC_SMPL_VALID;
        adcc_writel(id, val, ch_base);
        chans &= ~BIT(chan);
    }

out:
    return ret;
}

SECTION_ADCC_TEXT error_t
adcc_read_chan_data(const struct adcc_id *id,
                    uint32_t chan,
                    uint32_t *data)
{
    error_t   ret = E_SUCCESS;
    uint32_t  ch_base;
    uint32_t  val;

    if (id == NULL) {
        ret = E_ADCC_NULL_PTR;
        goto out;
    }

    if (!adcc_inited) {
        ret = E_ADCC_NO_INIT;
        goto out;
    }

    if (chan >= ADCC_NCHANS) {
        ret = E_ADCC_INVALID_CHAN;
        goto out;
    }

    val = adcc_readl(id, ADCC_SCAN_SEL_0) & ADCC_SCAN_SEL_MSK;
    if ((val & BIT(chan)) == 0UL) {
        ret = E_ADCC_CHAN_NOT_ENABLED;
        goto out;
    }

    ch_base = ADCC_CH0_SAMPLE_0 + (chan * ADCC_CH_OFFSET);
    do {
        val = adcc_readl(id, ch_base);
    } while ((val & ADCC_SMPL_VALID) == 0UL);
    *data = val & ADCC_SMPL_VAL_MSK;
    val &= ~ADCC_SMPL_VALID;
    adcc_writel(id, val, ch_base);

out:
    return ret;
}

SECTION_ADCC_TEXT error_t
adcc_stop_scan(const struct adcc_id *id)
{
    error_t   ret = E_SUCCESS;
    uint32_t  ctl_reg;

    if (id == NULL) {
        ret = E_ADCC_NULL_PTR;
        goto out;
    }

    if (!adcc_inited) {
        ret = E_ADCC_NO_INIT;
        goto out;
    }

    ctl_reg = adcc_readl(id, ADCC_CTL0_0);
    ctl_reg &= ~ADCC_SCAN_EN;
    adcc_writel(id, ctl_reg, ADCC_CTL0_0);
    /* FIXME: Use a timeout based and return timed out error below */
    do {
        ctl_reg = adcc_readl(id, ADCC_CTL0_0);
    } while ((ctl_reg & ADCC_SCAN_EN) != 0UL);

out:
    return ret;
}

SECTION_ADCC_TEXT error_t
adcc_start_scan(const struct adcc_id *id)
{
    error_t   ret = E_SUCCESS;
    uint32_t  ctl_reg;

    if (id == NULL) {
        ret = E_ADCC_NULL_PTR;
        goto out;
    }

    if (!adcc_inited) {
        ret = E_ADCC_NO_INIT;
        goto out;
    }

    adcc_reset_chans(id);
    ctl_reg = adcc_readl(id, ADCC_CTL0_0);
    ctl_reg |= ADCC_SCAN_EN;
    adcc_writel(id, ctl_reg, ADCC_CTL0_0);

out:
    return ret;
}

SECTION_ADCC_INIT_TEXT error_t
adcc_init(const struct adcc_id *id,
          const struct adcc_conf *conf)
{
    error_t   ret = E_SUCCESS;
    uint32_t  max_chan;
    uint32_t  enabled_chans;

    if (id == NULL) {
        ret = E_ADCC_NULL_PTR;
        goto out;
    }

    ret = adcc_configure_clk_rst(id, conf->clk_src);
    if (ret != E_SUCCESS) {
        goto out;
    }

    enabled_chans = conf->enabled_chans;
    max_chan = 31U - __builtin_clz(enabled_chans);
    if (max_chan >= ADCC_NCHANS) {
        ret = E_ADCC_INVALID_PARAM;
        goto out;
    }

    adcc_clear_power_down(id);
    adcc_configure_pad_ctl(id, conf);
    adcc_configure_scan_mode(id, !conf->mode);
    adcc_reset_chans(id);
    adcc_configure_control_regs(id, enabled_chans);

    adcc_inited = true;

out:
    return ret;
}
