/*
 * Copyright (c) 2015-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/* Compiler headers */
#include <stdbool.h>                // for bool
#include <stdint.h>                 // for uint32_t, int32_t, uint8_t

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT
#include <soc-common/hw-const.h>    /* Must appear before any hwinc files */

/* Hardware headers */
#include <address_map_new.h>        // for NV_ADDRESS_MAP_CAR_BASE
#include <arclk_rst.h>              // for CLK_RST_CONTROLLER_OSC_CTRL_0_OSC...

/* Late FSP headers */
#include <clk/clk-tegra.h>          // for tegra_clk_disable, tegra_clk_enable
#ifdef _NV_BUILD_FPGA_
#include <clk-tegra-hw-cpu.h>       /* Immune from CT_ASSERT protection */
#endif
#include <delay/delay.h>            // for udelay
#include <misc/bitops.h>            // for BIT
#include <misc/nvrm_drf.h>          // for NV_DRF_DEF, NV_FIELD_HIGHBIT, NV_...
#include <reg-access/reg-access.h>  // for writel, readl
#include <chipid/chip-id.h>         // for tegra_platform_is_fpga

/* Module-specific FSP header files */

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
CT_ASSERT(FSP__SOC_COMMON__HW_CONST_H, "Header file missing or invalid.")
CT_ASSERT(FSP__CLK__CLK_TEGRA_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DELAY__DELAY_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")
CT_ASSERT(FSP__MISC__NVRM_DRF_H, "Header file missing or invalid.")
CT_ASSERT(FSP__REG_ACCESS__REG_ACCESS_H, "Header file missing or invalid.")

#define CLK_RST_CONTROLLER_CLK_OUT_ENB_0                0x00
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_SET_0            0x04
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_CLR_0            0x08

#define CLK_RST_CONTROLLER_RST_DEV_0                    0x00
#define CLK_RST_CONTROLLER_RST_DEV_SET_0                0x04
#define CLK_RST_CONTROLLER_RST_DEV_CLR_0                0x08

#define LOWBIT     0
#define HIGHBIT    1
#define FIELD_RANGE(x) { \
        [LOWBIT]  = NV_FIELD_LOWBIT(x), \
        [HIGHBIT] = NV_FIELD_HIGHBIT(x) \
}

struct tegra_clk {
        uint32_t enb_base_reg;
        uint32_t src_reg;
        uint8_t div_range[2]; /* [0]=LOWBIT [1]=HIGHBIT */
        uint32_t div_enb; /* 0 if enable bit is not available */
        uint8_t frac_width;
        uint32_t clk_src;
        const struct tegra_parent_clk *parent;
};

struct tegra_parent_clk {
        uint32_t rate;
};

struct tegra_rst {
        uint32_t rst_base_reg;
};

struct tegra_parent_clk tegra_parent_clk_osc_undiv_data = {
        .rate = 0,
};

struct tegra_parent_clk tegra_parent_clk_pll_p_data = {
        .rate = 408000000,
};

struct tegra_parent_clk tegra_parent_clk_pll_aon_data = {
        .rate = 0,
};

const struct tegra_clk tegra_clk_uarta_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTA_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0_UARTA_CLK_DIVISOR),
#if defined(_NV_BUILD_FPGA_) || defined(_NV_BUILD_LINSIM_)
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, DISABLE),
#else
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, ENABLE),
#endif
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_uarta_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTA_0,
};

const struct tegra_clk tegra_clk_uartb_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTB_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTB_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_UARTB_0_UARTB_CLK_DIVISOR),
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTB, UARTB_DIV_ENB, ENABLE),
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTB, UARTB_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_uartb_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTB_0,
};

const struct tegra_clk tegra_clk_uartc_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTC_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTC_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_UARTC_0_UARTC_CLK_DIVISOR),
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTC, UARTC_DIV_ENB, ENABLE),
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTC, UARTC_CLK_SRC, OSC_UNDIV),
        .parent = &tegra_parent_clk_osc_undiv_data,
};

const struct tegra_rst tegra_rst_uartc_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTC_0,
};

const struct tegra_clk tegra_clk_uartf_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTF_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTF_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_UARTF_0_UARTF_CLK_DIVISOR),
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTF, UARTF_DIV_ENB, ENABLE),
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTF, UARTF_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_uartf_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTF_0,
};

const struct tegra_clk tegra_clk_uartg_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTG_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTG_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_UARTG_0_UARTG_CLK_DIVISOR),
#if defined(_NV_BUILD_FPGA_) || defined(_NV_BUILD_LINSIM_)
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTG, UARTG_DIV_ENB, DISABLE),
#else
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTG, UARTG_DIV_ENB, ENABLE),
#endif
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTG, UARTG_CLK_SRC, OSC_UNDIV),
        .parent = &tegra_parent_clk_osc_undiv_data,
};

const struct tegra_rst tegra_rst_uartg_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTG_0,
};

const struct tegra_clk tegra_clk_uarth_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTH_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTH_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_UARTH_0_UARTH_CLK_DIVISOR),
        .div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTH, UARTH_DIV_ENB, ENABLE),
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTH, UARTH_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_uarth_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTH_0,
};

const struct tegra_clk tegra_clk_aon_i2c_slow_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_AON_I2C_SLOW_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C_SLOW_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C_SLOW_0_AON_I2C_SLOW_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_AON_I2C_SLOW, AON_I2C_SLOW_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_clk tegra_clk_i2c_slow_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C_SLOW_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW_0_I2C_SLOW_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C_SLOW, I2C_SLOW_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_clk tegra_clk_i2c1_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C1_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0_I2C1_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 0,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C1, I2C1_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c1_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C1_0,
};

const struct tegra_clk tegra_clk_i2c2_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C2_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C2_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_I2C2_0_I2C2_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 0,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C2, I2C2_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c2_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C2_0,
};

const struct tegra_clk tegra_clk_i2c3_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C3_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C3_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_I2C3_0_I2C3_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 0,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C3, I2C3_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c3_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C3_0,
};

const struct tegra_clk tegra_clk_i2c8_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C8_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C8_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_I2C8_0_I2C8_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 0,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C8, I2C8_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c8_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C8_0,
};

const struct tegra_clk tegra_clk_i2c10_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C10_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C10_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_I2C10_0_I2C10_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 0,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C10, I2C10_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c10_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C10_0,
};

const struct tegra_rst tegra_rst_aon_gpcdma_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_AON_DMA_0,
};

const struct tegra_rst tegra_rst_sce_gpcdma_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SCE_DMA_0,
};

const struct tegra_rst tegra_rst_rce_gpcdma_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_RCE_DMA_0,
};

const struct tegra_clk tegra_clk_aodmic_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_DMIC5_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_DMIC5_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_DMIC5_0_DMIC5_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER,CLK_SOURCE_DMIC5, DMIC5_CLK_SRC, OSC_UNDIV),
        .parent = &tegra_parent_clk_osc_undiv_data,
};

const struct tegra_rst tegra_rst_aodmic_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_DMIC5_0,
};

const struct tegra_clk tegra_clk_spi1_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_SPI1_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_SPI1_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_SPI1_0_SPI1_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPI1, SPI1_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_spi1_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SPI1_0,
};

const struct tegra_clk tegra_clk_spi2_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_SPI2_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_SPI2_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_SPI2_0_SPI2_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPI2, SPI2_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_spi2_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SPI2_0,
};

const struct tegra_clk tegra_clk_spi3_data = {
        .enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_SPI3_0,
        .src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_SPI3_0,
        .div_range = FIELD_RANGE((uint8_t)CLK_RST_CONTROLLER_CLK_SOURCE_SPI3_0_SPI3_CLK_DIVISOR),
        .div_enb = 0,
        .frac_width = 1,
        .clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPI3, SPI3_CLK_SRC, PLLP_OUT0),
        .parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_spi3_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SPI3_0,
};

const struct tegra_rst tegra_rst_can0_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_CAN1_0,
};

const struct tegra_rst tegra_rst_can1_data = {
        .rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_CAN2_0,
};

int32_t tegra_clk_init(void)
{
        uint32_t osc_undiv_rate = 38400000;
        uint32_t val;
        uint32_t osc_freq;

        if (tegra_platform_is_fpga()) {
                tegra_parent_clk_osc_undiv_data.rate = 19200000;
                goto out;
        }

        val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_OSC_CTRL_0);

        osc_freq = NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, val);
        switch (osc_freq) {
        case 0:
                osc_undiv_rate = 13000000;
                break;
        case 1:
                osc_undiv_rate = 16800000;
                break;
        case 4:
                osc_undiv_rate = 19200000;
                break;
        case 5:
                osc_undiv_rate = 38400000;
                break;
        case 8:
                osc_undiv_rate = 12000000;
                break;
        case 9:
                osc_undiv_rate = 48000000;
                break;
        case 12:
                osc_undiv_rate = 26000000;
                break;
        default:
                osc_undiv_rate = 38400000;
                break;
        }

        tegra_parent_clk_osc_undiv_data.rate = osc_undiv_rate;
out:
        return 0;
}

int32_t tegra_clk_enable(const struct tegra_clk *clk)
{
        writel_base_offset(BIT(0), NV_ADDRESS_MAP_CAR_BASE + clk->enb_base_reg,
                           CLK_RST_CONTROLLER_CLK_OUT_ENB_SET_0);
        return 0;
}

int32_t tegra_clk_disable(const struct tegra_clk *clk)
{
        writel_base_offset(BIT(0), NV_ADDRESS_MAP_CAR_BASE + clk->enb_base_reg,
                           CLK_RST_CONTROLLER_CLK_OUT_ENB_CLR_0);
        return 0;
}

bool tegra_clk_is_clk_enabled(const struct tegra_clk *clk)
{
    uint32_t clk_en_sts;

    clk_en_sts = readl(NV_ADDRESS_MAP_CAR_BASE + clk->enb_base_reg);
    return ((BIT(0) & clk_en_sts) != 0UL);
}

bool tegra_reset_is_reset_deasserted(const struct tegra_rst *rst)
{
    uint32_t reset_sts;

    reset_sts = readl(NV_ADDRESS_MAP_CAR_BASE + rst->rst_base_reg);
    return ((BIT(0) & reset_sts) != 0UL);
}

int32_t tegra_clk_reset_set(const struct tegra_rst *rst)
{
        writel_base_offset(BIT(0), NV_ADDRESS_MAP_CAR_BASE + rst->rst_base_reg,
                           CLK_RST_CONTROLLER_RST_DEV_SET_0);
        return 0;
}

int32_t tegra_clk_reset_clear(const struct tegra_rst *rst)
{
        writel_base_offset(BIT(0), NV_ADDRESS_MAP_CAR_BASE + rst->rst_base_reg,
                           CLK_RST_CONTROLLER_RST_DEV_CLR_0);
        return 0;
}

int32_t tegra_clk_reset_pulse(const struct tegra_rst *rst, uint32_t delay_us)
{
        int ret;

        ret = tegra_clk_reset_set(rst);
        if (ret)
                return ret;
        udelay(delay_us);
        return tegra_clk_reset_clear(rst);
}

int32_t tegra_clk_set_rate(const struct tegra_clk *clk, uint32_t rate_hz)
{
        uint32_t divisor;
        uint32_t val;

        /*
         * For now, we always round to ensure the clock runs no faster than
         * the requested rate. If some clocks need other rounding options,
         * we can add flags to struct tegra_clk for that.
         */
        divisor = (((clk->parent->rate) << clk->frac_width) + rate_hz - 1UL) / rate_hz;
        divisor -= (1UL << clk->frac_width);
        if (divisor & ~NV_DYN_FIELD_MASK(clk->div_range[HIGHBIT], clk->div_range[LOWBIT]))
                return 1;

        val = NV_DYN_FIELD_BITS(divisor, clk->div_range[HIGHBIT], clk->div_range[LOWBIT]) |
              clk->clk_src | clk->div_enb;

        writel(val, NV_ADDRESS_MAP_CAR_BASE + clk->src_reg);

        return 0;
}

int32_t tegra_parent_clk_set_rate(struct tegra_parent_clk *clk, uint32_t rate_hz)
{
        clk->rate = rate_hz;
        return 0;
}
