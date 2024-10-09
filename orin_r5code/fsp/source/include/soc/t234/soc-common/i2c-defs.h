/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef I2C_DEFS_H
#define I2C_DEFS_H
#define FSP__SOC_COMMON__I2C_DEFS_H                  1

/* Compiler headers */

/* Early FSP headers */
#include <misc/ct-assert.h>  // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <misc/bitops.h>     // for BIT, FSP__MISC__BITOPS_H

/* Module-specific FSP headers */

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
CT_ASSERT(FSP__MISC__BITOPS_H, "Header file missing or invalid.")

/**
 * @brief Maximum I2C controller.
 *
 * Maximum number of I2C controller supported by the I2C drivers.
 */
#define TEGRA234_I2C1       1U
#define TEGRA234_I2C2       2U
#define TEGRA234_I2C3       3U
#define TEGRA234_I2C4       4U
#define TEGRA234_I2C5       5U
#define TEGRA234_I2C6       6U
#define TEGRA234_I2C7       7U
#define TEGRA234_I2C8       8U
#define TEGRA234_I2C9       9U
#define TEGRA234_I2C10      10U
#define TEGRA234_I2C_MAX    10U

#define I2C_MAX_CTRLS       TEGRA234_I2C_MAX

/**
 * @brief Defines for I2C configuration register
 *
 * @I2C_CNFG_REG Config register offset
 * @I2C_CNFG_DEBOUNCE_CNT_SHIFT Debounce count for sda and scl
 * @I2C_CNFG_PACKET_MODE_EN 1 - intiate transfer in packet mode
 * @I2C_CNFG_NEW_MASTER_FSM for compatibility with FSM
 */
#define I2C_CNFG_REG                        0x0U
#define I2C_CNFG_DEBOUNCE_CNT_SHIFT         12U
#define I2C_CNFG_MULTI_MASTER_MODE          BIT(17)
#define I2C_CNFG_PACKET_MODE_EN             BIT(10U)
#define I2C_CNFG_NEW_MASTER_FSM             BIT(11U)

/**
 * @brief Defines for I2C status register
 *
 * @I2C_STS_REG offset of Status register
 * @I2C_STATUS_BUSY 1 - Busy 0 - Not Busy
 */
#define I2C_STS_REG                         0x1cU
#define I2C_STATUS_BUSY                     BIT(8U)

/**
 * @brief Defines for I2C fifo register
 *
 * @I2C_TX_FIFO SW writes into this register
 * @I2C_RX_FIFO SW reads from this register
 */
#define I2C_TX_FIFO                         0x050U
#define I2C_RX_FIFO                         0x054U

/**
 * @brief Defines for I2C interrupt register
 *
 * @I2C_INT_MASK Interrupt mask register offset
 * @I2C_INT_STATUS Interrupt status register offset
 * @I2C_INT_SPURIOUS SW defined bit for spurious interrupts
 * @I2C_INT_TXN_TIMEOUT SW defined bit for transaction timeout
 * @I2C_INT_BUS_CLEAR_DONE Bus clear done status
 * @I2C_INT_PACKET_XFER_COMPLETE A packet has been transferred succesfully
 * @I2C_INT_ALL_PACKETS_XFER_COMPLETE All packets transferred succesfully
 * @I2C_INT_TX_FIFO_OVERFLOW TX fifo overflow
 * @I2C_INT_RX_FIFO_UNDERFLOW RX fifo overflow
 * @I2C_INT_NO_ACK No Acknowledge from slave
 * @I2C_INT_ARB_LOST Arbitration last
 * @I2C_INT_TX_FIFO_DATA_REQ TX fifo data request
 * @I2C_INT_RX_FIFO_DATA_REQ RX fifo data request
 */
#define I2C_INT_MASK                        0x064U
#define I2C_INT_STATUS                      0x068U
#define I2C_INT_SPURIOUS                    BIT(15U) /* sw-defined */
#define I2C_INT_TXN_TIMEOUT                 BIT(14U) /* sw-defined */
#define I2C_INT_DISABLE_PACKET_FAIL         BIT(13U) /* sw-defined */
#define I2C_INT_EVENT_BIT_SET_FAIL          BIT(12U) /* sw-defined */
#define I2C_INT_BUS_CLEAR_DONE              BIT(11U)
#define I2C_INT_PACKET_XFER_COMPLETE        BIT(7U)
#define I2C_INT_ALL_PACKETS_XFER_COMPLETE   BIT(6U)
#define I2C_INT_TX_FIFO_OVERFLOW            BIT(5U)
#define I2C_INT_RX_FIFO_UNDERFLOW           BIT(4U)
#define I2C_INT_NO_ACK                      BIT(3U)
#define I2C_INT_ARB_LOST                    BIT(2U)
#define I2C_INT_TX_FIFO_DATA_REQ            BIT(1U)
#define I2C_INT_RX_FIFO_DATA_REQ            BIT(0U)

/**
 * @brief Defines for I2C clock divisor register
 *
 * @I2C_CLK_DIVISOR clock divisor register offset
 * @I2C_CLK_DIVISOR_STD_FAST_MASK Standard Fast mode mask
 * @I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT Standard Fast mode shift bits
 * @I2C_CLK_MULTIPLIER_STD_FAST_MODE Multiplier for Standard fast mode
 */
#define I2C_CLK_DIVISOR                     0x06cU
#define I2C_CLK_DIVISOR_STD_FAST_MASK       (0xFFFFUL << 16U)
#define I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT 16U
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE    8U

/**
 * @brief Defines for packet header fields as per I2C packet transfer protocol
 *
 * @I2C_HEADER_HIGHSPEED_MODE enable HS mode of operation
 * @I2C_HEADER_CONT_ON_NAK Continue on No Ack from slave
 * @I2C_HEADER_READ Read transaction
 * @I2C_HEADER_10BIT_ADDR 10 bit slave address
 * @I2C_HEADER_REPEAT_START to continue or end transaction
 * @I2C_HEADER_CONTINUE_XFER Override Repeated start
 * @I2C_HEADER_MASTER_ADDR_SHIFT High Speed mode Master code shift
 * @I2C_HEADER_SLAVE_ADDR_SHIFT slave address shift
 */
#define I2C_HEADER_HIGHSPEED_MODE           BIT(22U)
#define I2C_HEADER_CONT_ON_NAK              BIT(21U)
#define I2C_HEADER_READ                     BIT(19U)
#define I2C_HEADER_10BIT_ADDR               BIT(18U)
#define I2C_HEADER_IE                       BIT(17U)
#define I2C_HEADER_REPEAT_START             BIT(16U)
#define I2C_HEADER_CONTINUE_XFER            BIT(15U)
#define I2C_HEADER_MASTER_ADDR_SHIFT        12U
#define I2C_HEADER_SLAVE_ADDR_SHIFT         1U

/**
 * @brief Defines for I2C configuration load register
 *
 * @I2C_CONFIG_LOAD_REG configuration load register offset
 * @I2C_MSTR_CONFIG_LOAD load master configuration
 * @I2C_SLV_CONFIG_LOAD load slave configuration
 * @I2C_TIMEOUT_CONFIG_LOAD load timeout configuration
 */
#define I2C_CONFIG_LOAD_REG                 0x8cU
#define I2C_MSTR_CONFIG_LOAD                BIT(0U)
#define I2C_SLV_CONFIG_LOAD                 BIT(1U)
#define I2C_TIMEOUT_CONFIG_LOAD             BIT(2U)

/**
 * @brief Defines for I2C clock enable override register
 *
 * @I2C_CLKEN_OVERRIDE clock enable override register offset
 * @I2C_MST_CORE_CLKEN_OVR override for 2nd-level clock enable for I2C master
 */
#define I2C_CLKEN_OVERRIDE                  0x90U
#define I2C_MST_CORE_CLKEN_OVR              1U

/**
 * @brief Defines for I2C interface timing Register
 *
 * @I2C_INTERFACE_TIMING Interface timing register offset
 * @I2C_INTERFACE_MASK Interface timing register mask
 * @I2C_THIGH_SHIFT bit shift value for THIGH
 */
#define I2C_INTERFACE_TIMING                0x94U
#define I2C_INTERFACE_MASK                  (0x3FU)
#define I2C_THIGH_SHIFT                     8U

/**
 * @brief Defines for I2C master reset register
 *
 * @I2C_MSTR_RESET master reset register offset
 * @I2C_MSTR_SOFT_RESET 1 - reset master internal state, 0 - normal operation
 */
#define I2C_MSTR_RESET                      0xa8U
#define I2C_MSTR_SOFT_RESET                 BIT(0U)

/*
 * FIFO size has been increased in T194 and to account for that,
 * the FIFO control/status registers have been split into
 * separate master and slave registers. Here we map the defines
 * to the master registers, since that is sufficient for our use case.
 */

/**
 * @brief Defines for I2C master packet transfer status register
 *
 * @I2C_PACKET_XFER_STATUS Number of packets transferred in the current packet
 */
#define I2C_PACKET_XFER_STATUS              0x0b0U

/**
 * @brief Defines for I2C master fifo control Register
 *
 * @I2C_FIFO_CONTROL Fifo control register offset
 * @I2C_FIFO_CONTROL_TX_FLUSH Flush the TX fifo
 * @I2C_FIFO_CONTROL_RX_FLUSH Flush the RX fifo
 * @I2C_FIFO_CONTROL_TX_TRIG_SHIFT Master Transmit fifo trigger level shift
 * @I2C_FIFO_CONTROL_RX_TRIG_SHIFT Master Receive fifo trigger level shift
 */
#define I2C_FIFO_CONTROL                    0x0b4U
#define I2C_FIFO_CONTROL_TX_FLUSH           BIT(1U)
#define I2C_FIFO_CONTROL_RX_FLUSH           BIT(0U)
#define I2C_FIFO_CONTROL_TX_TRIG_SHIFT      16U
#define I2C_FIFO_CONTROL_RX_TRIG_SHIFT      4U

/**
 * @brief Defines for I2C master fifo status Register
 *
 * @I2C_FIFO_STATUS Fifo status register offset
 * @I2C_FIFO_STATUS_TX_MASK Transfer fifo empty slot count mask
 * @I2C_FIFO_STATUS_TX_SHIFT Transfer fifo empty slot count shift
 * @I2C_FIFO_STATUS_RX_MASK Receive fifo full slot count mask
 * @I2C_FIFO_STATUS_RX_SHIFT Receive fifo full slot count shift
 */
#define I2C_FIFO_STATUS                     0x0b8U
#define I2C_FIFO_STATUS_TX_MASK             0xff0000U
#define I2C_FIFO_STATUS_TX_SHIFT            16U
#define I2C_FIFO_STATUS_RX_MASK             0xffU
#define I2C_FIFO_STATUS_RX_SHIFT            0U

/**
 * @brief Defines for I2C master fifo status Register
 *
 * @I2C_FIFO_STATUS Fifo status register offset
 * @I2C_FIFO_STATUS_TX_MASK Transfer fifo empty slot count mask
 * @I2C_FIFO_STATUS_TX_SHIFT Transfer fifo empty slot count shift
 * @I2C_FIFO_STATUS_RX_MASK Receive fifo full slot count mask
 * @I2C_FIFO_STATUS_RX_SHIFT Receive fifo full slot count shift
 */
#define I2C_FIFO_STATUS                     0x0b8U
#define I2C_FIFO_STATUS_TX_MASK             0xff0000U
#define I2C_FIFO_STATUS_TX_SHIFT            16U
#define I2C_FIFO_STATUS_RX_MASK             0xffU
#define I2C_FIFO_STATUS_RX_SHIFT            0U

#define PACKET_HEADER0_PACKET_ID_SHIFT      16U
#define PACKET_HEADER0_CONT_ID_SHIFT        12U
#define PACKET_HEADER0_PROTOCOL_I2C         BIT(4U)

#define I2C_BUS_STATUS                      0x0d0U
#define I2C_BUS_STATUS_BUSY_MASK            BIT(0U)
#define I2C_BUS_STATUS_SDA_MASK             BIT(1U)
#define I2C_BUS_STATUS_SCL_MASK             BIT(2U)

#endif
