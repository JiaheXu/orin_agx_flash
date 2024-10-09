/*
 * Copyright (c) 2015-2019 NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef SOC_COMMON__CLK_TEGRA_HW_H
#define SOC_COMMON__CLK_TEGRA_HW_H
#define FSP__SOC_COMMON__CLK_TEGRA_HW_H                 1

extern const struct tegra_clk tegra_clk_uarta_data;
extern const struct tegra_clk tegra_clk_uartb_data;
extern const struct tegra_clk tegra_clk_uartc_data;
extern const struct tegra_clk tegra_clk_uartf_data;
extern const struct tegra_clk tegra_clk_uartg_data;
extern const struct tegra_clk tegra_clk_uarth_data;
extern const struct tegra_clk tegra_clk_aon_i2c_slow_data;
extern const struct tegra_clk tegra_clk_i2c_slow_data;
extern const struct tegra_clk tegra_clk_i2c1_data;
extern const struct tegra_clk tegra_clk_i2c2_data;
extern const struct tegra_clk tegra_clk_i2c3_data;
extern const struct tegra_clk tegra_clk_i2c8_data;
extern const struct tegra_clk tegra_clk_i2c10_data;
extern const struct tegra_clk tegra_clk_aodmic_data;
extern const struct tegra_clk tegra_clk_spi1_data;
extern const struct tegra_clk tegra_clk_spi2_data;
extern const struct tegra_clk tegra_clk_spi3_data;

extern struct tegra_parent_clk tegra_parent_clk_pll_aon_data;
extern struct tegra_parent_clk tegra_parent_clk_pll_p_data;
extern struct tegra_parent_clk tegra_parent_clk_osc_undiv_data;

extern const struct tegra_rst tegra_rst_uarta_data;
extern const struct tegra_rst tegra_rst_uartb_data;
extern const struct tegra_rst tegra_rst_uartc_data;
extern const struct tegra_rst tegra_rst_uartf_data;
extern const struct tegra_rst tegra_rst_uartg_data;
extern const struct tegra_rst tegra_rst_uarth_data;
extern const struct tegra_rst tegra_rst_i2c1_data;
extern const struct tegra_rst tegra_rst_i2c2_data;
extern const struct tegra_rst tegra_rst_i2c3_data;
extern const struct tegra_rst tegra_rst_i2c8_data;
extern const struct tegra_rst tegra_rst_i2c10_data;
extern const struct tegra_rst tegra_rst_aodmic_data;
extern const struct tegra_rst tegra_rst_aon_gpcdma_data;
extern const struct tegra_rst tegra_rst_sce_gpcdma_data;
extern const struct tegra_rst tegra_rst_rce_gpcdma_data;
extern const struct tegra_rst tegra_rst_spi1_data;
extern const struct tegra_rst tegra_rst_spi2_data;
extern const struct tegra_rst tegra_rst_spi3_data;
extern const struct tegra_rst tegra_rst_can0_data;
extern const struct tegra_rst tegra_rst_can1_data;

#define tegra_clk_uarta (&tegra_clk_uarta_data)
#define tegra_clk_uartb (&tegra_clk_uartb_data)
#define tegra_clk_uartc (&tegra_clk_uartc_data)
#define tegra_clk_uartf (&tegra_clk_uartf_data)
#define tegra_clk_uartg (&tegra_clk_uartg_data)
#define tegra_clk_uarth (&tegra_clk_uarth_data)
#define tegra_clk_aon_i2c_slow (&tegra_clk_aon_i2c_slow_data)
#define tegra_clk_i2c_slow (&tegra_clk_i2c_slow_data)
#define tegra_clk_i2c1 (&tegra_clk_i2c1_data)
#define tegra_clk_i2c2 (&tegra_clk_i2c2_data)
#define tegra_clk_i2c3 (&tegra_clk_i2c3_data)
#define tegra_clk_i2c8 (&tegra_clk_i2c8_data)
#define tegra_clk_i2c10 (&tegra_clk_i2c10_data)
#define tegra_clk_aodmic (&tegra_clk_aodmic_data)
#define tegra_clk_spi1 (&tegra_clk_spi1_data)
#define tegra_clk_spi2 (&tegra_clk_spi2_data)
#define tegra_clk_spi3 (&tegra_clk_spi3_data)
#define tegra_parent_clk_pll_aon tegra_parent_clk_pll_aon_data
#define tegra_parent_clk_pll_p tegra_parent_clk_pll_p_data
#define tegra_parent_clk_osc_undiv tegra_parent_clk_osc_undiv_data

#define tegra_rst_uarta (&tegra_rst_uarta_data)
#define tegra_rst_uartb (&tegra_rst_uartb_data)
#define tegra_rst_uartc (&tegra_rst_uartc_data)
#define tegra_rst_uartf (&tegra_rst_uartf_data)
#define tegra_rst_uartg (&tegra_rst_uartg_data)
#define tegra_rst_uarth (&tegra_rst_uarth_data)
#define tegra_rst_i2c1 (&tegra_rst_i2c1_data)
#define tegra_rst_i2c2 (&tegra_rst_i2c2_data)
#define tegra_rst_i2c3 (&tegra_rst_i2c3_data)
#define tegra_rst_i2c8 (&tegra_rst_i2c8_data)
#define tegra_rst_i2c10 (&tegra_rst_i2c10_data)
#define tegra_rst_aodmic (&tegra_rst_aodmic_data)
#define tegra_rst_aon_gpcdma (&tegra_rst_aon_gpcdma_data)
#define tegra_rst_sce_gpcdma (&tegra_rst_sce_gpcdma_data)
#define tegra_rst_rce_gpcdma (&tegra_rst_rce_gpcdma_data)
#define tegra_rst_spi1 (&tegra_rst_spi1_data)
#define tegra_rst_spi2 (&tegra_rst_spi2_data)
#define tegra_rst_spi3 (&tegra_rst_spi3_data)
#define tegra_rst_can0 (&tegra_rst_can0_data)
#define tegra_rst_can1 (&tegra_rst_can1_data)

#endif
