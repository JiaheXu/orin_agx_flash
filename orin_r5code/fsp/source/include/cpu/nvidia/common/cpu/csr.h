/**
 * @file csr.h
 *
 * @brief NVRISC-V CSR Macros
 *
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS`` AND ANY
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

#ifndef CPU__CSR_H
#define CPU__CSR_H
#define FSP__CPU__CSR_H                       1

/*
 * @file csr.h
 * Macros to improve usability of NVRISC-V Control Status Register(s) (CSR).
 * Provides CSRs with no operating mode that will be translated to the
 * appropriate operating mode at compile-time. Also provides wrappers for
 * CSR assembly functions for ease of use.
 */

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance block(include) \
         (deviate MISRA_C_2012_Rule_20_10 \"Approval: Bug 200532008, DR: SWE-FSP-035-SWSADR.docx\") \
         (deviate MISRA_C_2012_Rule_2_5 \"Approval: Bug 3175244, DR: SWE-FSP-013-SWSADR.docx\")")
#endif

#if (__riscv_xlen == 32)
#   include NVRISCV32_MANUAL_CSR
#else
#   include NVRISCV64_MANUAL_CSR
#   ifndef NV_RISCV_CSR_MARCHID_ARCH_RST
#       define NV_RISCV_CSR_MARCHID_ARCH_RST           ((NV_RISCV_CSR_MARCHID_MSB_RST         << 63UL) | \
                                                        (NV_RISCV_CSR_MARCHID_RS1_RST         << 16UL) | \
                                                        (NV_RISCV_CSR_MARCHID_CORE_MAJOR_RST  << 12UL) | \
                                                        (NV_RISCV_CSR_MARCHID_CORE_MINOR_RST  <<  8UL))
#       define NV_RISCV_CSR_MARCHID_ARCH_RST__SHIFTMASK 0xFFFFFFFFFFFFFF00ULL
#   else
#       define NV_RISCV_CSR_MARCHID_ARCH_RST__SHIFTMASK 0xFFFFFFFFFFFFFFFFULL
#   endif
#endif // (__riscv_xlen == 32)

/*
 * @brief Translate CSRX into appropriate operating mode
 *
 * macro-title CSRX Operating Mode Translation
 *
 * Table of prefixes:
 *               NVRISCV_CONFIG_CPU_MODE
 *        Macro  | 3 | 2 | 1 | 0 |
 * NV_RISCV_CSRX | M |N/A| S |N/A|
 * NV_RISCV_CSRU | M |N/A|   |N/A|
 *
 * @NV_RISCV_CSRX1(_A)              NV_RISCV_CSRX + _A
 * @NV_RISCV_CSRX2(_A, _B)          NV_RISCV_CSRX + _A + _B
 * @NV_RISCV_CSRX3(_A, _B, _C)      NV_RISCV_CSRX + _A + _B + _C
 * @NV_RISCV_CSRU1(_A)              NV_RISCV_CSRU + _A
 * @NV_RISCV_CSRU2(_A, _B)          NV_RISCV_CSRU + _A + _B
 * @NV_RISCV_CSRU3(_A, _B, _C)      NV_RISCV_CSRU + _A + _B + _C
 */
#ifndef NVRISCV_CONFIG_CPU_MODE
// If the kernel mode hasn't been defined, X versions default to M mode, and a warning is generated
# warning "NVRISCV_CONFIG_CPU_MODE undefined, NV_RISCV_CSR_X* macros defaulting to M mode"
# define NV_RISCV_CSRX1(_A)            NV_RISCV_CSR_M ## _A
# define NV_RISCV_CSRX2(_A, _B)        NV_RISCV_CSR_M ## _A ## M ## _B
# define NV_RISCV_CSRX3(_A, _B, _C)    NV_RISCV_CSR_M ## _A ## M ## _B ## M ## _C
# define NV_RISCV_CSRU1(_A)            NV_RISCV_CSR_M ## _A
# define NV_RISCV_CSRU2(_A, _B)        NV_RISCV_CSR_M ## _A ## M ## _B
# define NV_RISCV_CSRU3(_A, _B, _C)    NV_RISCV_CSR_M ## _A ## M ## _B ## M ## _C
#elif NVRISCV_CONFIG_CPU_MODE == 3
// Machine mode
# define NV_RISCV_CSRX1(_A)            NV_RISCV_CSR_M ## _A
# define NV_RISCV_CSRX2(_A, _B)        NV_RISCV_CSR_M ## _A ## M ## _B
# define NV_RISCV_CSRX3(_A, _B, _C)    NV_RISCV_CSR_M ## _A ## M ## _B ## M ## _C
# define NV_RISCV_CSRU1(_A)            NV_RISCV_CSR_M ## _A
# define NV_RISCV_CSRU2(_A, _B)        NV_RISCV_CSR_M ## _A ## M ## _B
# define NV_RISCV_CSRU3(_A, _B, _C)    NV_RISCV_CSR_M ## _A ## M ## _B ## M ## _C
#elif NVRISCV_CONFIG_CPU_MODE == 2
// Hypervisor mode
# error "Hypervisor mode unsupported"
#elif NVRISCV_CONFIG_CPU_MODE == 1
// Supervisor mode
# define NV_RISCV_CSRX1(_A)            NV_RISCV_CSR_S ## _A
# define NV_RISCV_CSRX2(_A, _B)        NV_RISCV_CSR_S ## _A ## S ## _B
# define NV_RISCV_CSRX3(_A, _B, _C)    NV_RISCV_CSR_S ## _A ## S ## _B ## S ## _C
# define NV_RISCV_CSRU1(_A)            NV_RISCV_CSR_ ## _A
# define NV_RISCV_CSRU2(_A, _B)        NV_RISCV_CSR_ ## _A ## _B
# define NV_RISCV_CSRU3(_A, _B, _C)    NV_RISCV_CSR_ ## _A ## _B ## _C
#elif NVRISCV_CONFIG_CPU_MODE == 0
// User mode
# error "User mode unsupported"
#endif

//
// These got renamed from UACCESSATTR to MACCESSATTR.
// I believe they are otherwise completely identical.
//
// TODO VLAJA - rename to if NV_RISCV_CSR_MARCH_RST == 0 once manuals change
#ifdef NV_RISCV_CSR_MFETCHATTR
# define NV_RISCV_CSR_XACCESSATTR1(_A)  NV_RISCV_CSR_MLDSTATTR ## _A
# define NV_RISCV_CSR_XACCESSATTR       NV_RISCV_CSR_MLDSTATTR
#else
# define NV_RISCV_CSR_XACCESSATTR1(_A)  NV_RISCV_CSR_UACCESSATTR ## _A
# define NV_RISCV_CSR_XACCESSATTR       NV_RISCV_CSR_UACCESSATTR
#endif

#define NV_RISCV_CSR_XACCESSATTR_WPR__SHIFT \
    NV_RISCV_CSR_XACCESSATTR1(_WPR__SHIFT)
#define NV_RISCV_CSR_XACCESSATTR_WPR__SHIFTMASK \
    NV_RISCV_CSR_XACCESSATTR1(_WPR__SHIFTMASK)
#define NV_RISCV_CSR_XACCESSATTR_COHERENT__SHIFTMASK \
    NV_RISCV_CSR_XACCESSATTR1(_COHERENT__SHIFTMASK)
#define NV_RISCV_CSR_XACCESSATTR_CACHEABLE__SHIFTMASK \
    NV_RISCV_CSR_XACCESSATTR1(_CACHEABLE__SHIFTMASK)

//
// Unified CSRs from manuals
//
// We provide X letter which will automatically expand to M/S
//
#define NV_RISCV_CSR_XMPU_PAGE_SIZE             NV_RISCV_CSR_MPU_PAGE_SIZE

#define NV_RISCV_CSR_XCAUSE2                    NV_RISCV_CSRX1(CAUSE2)
#define NV_RISCV_CSR_XTVEC                      NV_RISCV_CSRX1(TVEC)

#define NV_RISCV_CSR_XCOUNTEREN                 NV_RISCV_CSRX1(COUNTEREN)
#define NV_RISCV_CSR_XCOUNTEREN_TM              NV_RISCV_CSRX1(COUNTEREN_TM)
#define NV_RISCV_CSR_XCOUNTEREN_TM_XMSB              NV_RISCV_CSRX1(COUNTEREN_TM_XMSB)
#define NV_RISCV_CSR_XCOUNTEREN_TM_XLSB              NV_RISCV_CSRX1(COUNTEREN_TM_XLSB)
#define NV_RISCV_CSR_XCOUNTEREN_CY              NV_RISCV_CSRX1(COUNTEREN_CY)
#define NV_RISCV_CSR_XCOUNTEREN_CY_XMSB              NV_RISCV_CSRX1(COUNTEREN_CY_XMSB)
#define NV_RISCV_CSR_XCOUNTEREN_CY_XLSB              NV_RISCV_CSRX1(COUNTEREN_CY_XLSB)
#define NV_RISCV_CSR_XCOUNTEREN_IR              NV_RISCV_CSRX1(COUNTEREN_IR)
#define NV_RISCV_CSR_XCOUNTEREN_IR_XMSB              NV_RISCV_CSRX1(COUNTEREN_IR_XMSB)
#define NV_RISCV_CSR_XCOUNTEREN_IR_XLSB              NV_RISCV_CSRX1(COUNTEREN_IR_XLSB)

#define NV_RISCV_CSR_XIE                        NV_RISCV_CSRX1(IE)
#define NV_RISCV_CSR_XIE_XMSB                        NV_RISCV_CSRX1(IE_XMSB)
#define NV_RISCV_CSR_XIE_XLSB                        NV_RISCV_CSRX1(IE_XLSB)
#define NV_RISCV_CSR_XIE_XEIE                   NV_RISCV_CSRX2(IE_, EIE)
#define NV_RISCV_CSR_XIE_XEIE_XMSB                   NV_RISCV_CSRX2(IE_, EIE_XMSB)
#define NV_RISCV_CSR_XIE_XEIE_XLSB                   NV_RISCV_CSRX2(IE_, EIE_XLSB)
#define NV_RISCV_CSR_XIE_XTIE                   NV_RISCV_CSRX2(IE_, TIE)
#define NV_RISCV_CSR_XIE_XTIE_XMSB                   NV_RISCV_CSRX2(IE_, TIE_XMSB)
#define NV_RISCV_CSR_XIE_XTIE_XLSB                   NV_RISCV_CSRX2(IE_, TIE_XLSB)
#define NV_RISCV_CSR_XIE_XSIE                   NV_RISCV_CSRX2(IE_, SIE)
#define NV_RISCV_CSR_XIE_XSIE_XMSB                   NV_RISCV_CSRX2(IE_, SIE_XMSB)
#define NV_RISCV_CSR_XIE_XSIE_XLSB                   NV_RISCV_CSRX2(IE_, SIE_XLSB)

#define NV_RISCV_CSR_XMPUATTR                   NV_RISCV_CSRX1(MPUATTR)
#define NV_RISCV_CSR_XMPUIDX                    NV_RISCV_CSRX1(MPUIDX)
#define NV_RISCV_CSR_XMPUIDX2                   NV_RISCV_CSRX1(MPUIDX2)
#define NV_RISCV_CSR_XMPUIDX2_IDX               NV_RISCV_CSRX1(MPUIDX2_IDX)
#define NV_RISCV_CSR_XMPUIDX2_IDX_XMSB               NV_RISCV_CSRX1(MPUIDX2_IDX_XMSB)
#define NV_RISCV_CSR_XMPUIDX2_IDX_XLSB               NV_RISCV_CSRX1(MPUIDX2_IDX_XLSB)
#define NV_RISCV_CSR_XMPUIDX_INDEX              NV_RISCV_CSRX1(MPUIDX_INDEX)
#define NV_RISCV_CSR_XMPUIDX_INDEX_XMSB              NV_RISCV_CSRX1(MPUIDX_INDEX_XMSB)
#define NV_RISCV_CSR_XMPUIDX_INDEX_XLSB              NV_RISCV_CSRX1(MPUIDX_INDEX_XLSB)
#define NV_RISCV_CSR_XMPUVA                     NV_RISCV_CSRX1(MPUVA)
#define NV_RISCV_CSR_XMPUPA                     NV_RISCV_CSRX1(MPUPA)
#define NV_RISCV_CSR_XMPUPA_BASE                NV_RISCV_CSRX1(MPUPA_BASE)
#define NV_RISCV_CSR_XMPUPA_BASE_XMSB                NV_RISCV_CSRX1(MPUPA_BASE_XMSB)
#define NV_RISCV_CSR_XMPUPA_BASE_XLSB                NV_RISCV_CSRX1(MPUPA_BASE_XLSB)
#define NV_RISCV_CSR_XMPURNG                    NV_RISCV_CSRX1(MPURNG)
#define NV_RISCV_CSR_XMPURNG_RANGE              NV_RISCV_CSRX1(MPURNG_RANGE)
#define NV_RISCV_CSR_XMPURNG_RANGE_XMSB              NV_RISCV_CSRX1(MPURNG_RANGE_XMSB)
#define NV_RISCV_CSR_XMPURNG_RANGE_XLSB              NV_RISCV_CSRX1(MPURNG_RANGE_XLSB)
#define NV_RISCV_CSR_XMPUATTR_WPR               NV_RISCV_CSRX1(MPUATTR_WPR)
#define NV_RISCV_CSR_XMPUATTR_WPR_XMSB               NV_RISCV_CSRX1(MPUATTR_WPR_XMSB)
#define NV_RISCV_CSR_XMPUATTR_WPR_XLSB               NV_RISCV_CSRX1(MPUATTR_WPR_XLSB)
#define NV_RISCV_CSR_XMPUATTR_CACHEABLE         NV_RISCV_CSRX1(MPUATTR_CACHEABLE)
#define NV_RISCV_CSR_XMPUATTR_CACHEABLE_XMSB         NV_RISCV_CSRX1(MPUATTR_CACHEABLE_XMSB)
#define NV_RISCV_CSR_XMPUATTR_CACHEABLE_XLSB         NV_RISCV_CSRX1(MPUATTR_CACHEABLE_XLSB)
#define NV_RISCV_CSR_XMPUATTR_COHERENT          NV_RISCV_CSRX1(MPUATTR_COHERENT)
#define NV_RISCV_CSR_XMPUATTR_COHERENT_XMSB          NV_RISCV_CSRX1(MPUATTR_COHERENT_XMSB)
#define NV_RISCV_CSR_XMPUATTR_COHERENT_XLSB          NV_RISCV_CSRX1(MPUATTR_COHERENT_XLSB)
#define NV_RISCV_CSR_XMPUATTR_L2C_WR            NV_RISCV_CSRX1(MPUATTR_L2C_WR)
#define NV_RISCV_CSR_XMPUATTR_L2C_WR_XMSB            NV_RISCV_CSRX1(MPUATTR_L2C_WR_XMSB)
#define NV_RISCV_CSR_XMPUATTR_L2C_WR_XLSB            NV_RISCV_CSRX1(MPUATTR_L2C_WR_XLSB)
#define NV_RISCV_CSR_XMPUATTR_L2C_RD            NV_RISCV_CSRX1(MPUATTR_L2C_RD)
#define NV_RISCV_CSR_XMPUATTR_L2C_RD_XMSB            NV_RISCV_CSRX1(MPUATTR_L2C_RD_XMSB)
#define NV_RISCV_CSR_XMPUATTR_L2C_RD_XLSB            NV_RISCV_CSRX1(MPUATTR_L2C_RD_XLSB)
#define NV_RISCV_CSR_XMPUATTR_UR                NV_RISCV_CSRX1(MPUATTR_UR)
#define NV_RISCV_CSR_XMPUATTR_UR_XMSB                NV_RISCV_CSRX1(MPUATTR_UR_XMSB)
#define NV_RISCV_CSR_XMPUATTR_UR_XLSB                NV_RISCV_CSRX1(MPUATTR_UR_XLSB)
#define NV_RISCV_CSR_XMPUATTR_XR                NV_RISCV_CSRX2(MPUATTR_, R)
#define NV_RISCV_CSR_XMPUATTR_XR_XMSB                NV_RISCV_CSRX2(MPUATTR_, R_XMSB)
#define NV_RISCV_CSR_XMPUATTR_XR_XLSB                NV_RISCV_CSRX2(MPUATTR_, R_XLSB)
#define NV_RISCV_CSR_XMPUATTR_UW                NV_RISCV_CSRX1(MPUATTR_UW)
#define NV_RISCV_CSR_XMPUATTR_UW_XMSB                NV_RISCV_CSRX1(MPUATTR_UW_XMSB)
#define NV_RISCV_CSR_XMPUATTR_UW_XLSB                NV_RISCV_CSRX1(MPUATTR_UW_XLSB)
#define NV_RISCV_CSR_XMPUATTR_XW                NV_RISCV_CSRX2(MPUATTR_, W)
#define NV_RISCV_CSR_XMPUATTR_XW_XMSB                NV_RISCV_CSRX2(MPUATTR_, W_XMSB)
#define NV_RISCV_CSR_XMPUATTR_XW_XLSB                NV_RISCV_CSRX2(MPUATTR_, W_XLSB)
#define NV_RISCV_CSR_XMPUATTR_UX                NV_RISCV_CSRX1(MPUATTR_UX)
#define NV_RISCV_CSR_XMPUATTR_UX_XMSB                NV_RISCV_CSRX1(MPUATTR_UX_XMSB)
#define NV_RISCV_CSR_XMPUATTR_UX_XLSB                NV_RISCV_CSRX1(MPUATTR_UX_XLSB)
#define NV_RISCV_CSR_XMPUATTR_XX                NV_RISCV_CSRX2(MPUATTR_, X)
#define NV_RISCV_CSR_XMPUATTR_XX_XMSB                NV_RISCV_CSRX2(MPUATTR_, X_XMSB)
#define NV_RISCV_CSR_XMPUATTR_XX_XLSB                NV_RISCV_CSRX2(MPUATTR_, X_XLSB)
#define NV_RISCV_CSR_XMPUVA_VLD                 NV_RISCV_CSRX1(MPUVA_VLD)
#define NV_RISCV_CSR_XMPUVA_VLD_XMSB                 NV_RISCV_CSRX1(MPUVA_VLD_XMSB)
#define NV_RISCV_CSR_XMPUVA_VLD_XLSB                 NV_RISCV_CSRX1(MPUVA_VLD_XLSB)

#define NV_RISCV_CSR_XSTATUS                    NV_RISCV_CSRX1(STATUS)
#define NV_RISCV_CSR_XSTATUS_XIE                NV_RISCV_CSRX2(STATUS_, IE)
#define NV_RISCV_CSR_XSTATUS_XIE_XMSB                NV_RISCV_CSRX2(STATUS_, IE_XMSB)
#define NV_RISCV_CSR_XSTATUS_XIE_XLSB                NV_RISCV_CSRX2(STATUS_, IE_XLSB)
#define NV_RISCV_CSR_XSTATUS_XIE_ENABLE         NV_RISCV_CSRX2(STATUS_, IE_ENABLE)
#define NV_RISCV_CSR_XSTATUS_XIE_XTIE           NV_RISCV_CSRX3(STATUS_, IE_, TIE)
#define NV_RISCV_CSR_XSTATUS_XIE_XTIE_XMSB           NV_RISCV_CSRX3(STATUS_, IE_, TIE_XMSB)
#define NV_RISCV_CSR_XSTATUS_XIE_XTIE_XLSB           NV_RISCV_CSRX3(STATUS_, IE_, TIE_XLSB)
#define NV_RISCV_CSR_XSTATUS_XPP                NV_RISCV_CSRX2(STATUS_, PP)
#define NV_RISCV_CSR_XSTATUS_XPP_XMSB                NV_RISCV_CSRX2(STATUS_, PP_XMSB)
#define NV_RISCV_CSR_XSTATUS_XPP_XLSB                NV_RISCV_CSRX2(STATUS_, PP_XLSB)
#define NV_RISCV_CSR_XSTATUS_FS                 NV_RISCV_CSRX1(STATUS_FS)
#define NV_RISCV_CSR_XSTATUS_FS_XMSB                 NV_RISCV_CSRX1(STATUS_FS_XMSB)
#define NV_RISCV_CSR_XSTATUS_FS_XLSB                 NV_RISCV_CSRX1(STATUS_FS_XLSB)
#define NV_RISCV_CSR_XSTATUS_FS_OFF             NV_RISCV_CSRX1(STATUS_FS_OFF)
#define NV_RISCV_CSR_XSTATUS_FS_INITIAL         NV_RISCV_CSRX1(STATUS_FS_INITIAL)
#define NV_RISCV_CSR_XSTATUS_FS_CLEAN           NV_RISCV_CSRX1(STATUS_FS_CLEAN)
#define NV_RISCV_CSR_XSTATUS_FS_DIRTY           NV_RISCV_CSRX1(STATUS_FS_DIRTY)

#define NV_RISCV_CSR_XDCACHEOP                  NV_RISCV_CSRU1(DCACHEOP)
#define NV_RISCV_CSR_XDCACHEOP_ADDR             NV_RISCV_CSRU1(DCACHEOP_ADDR)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_XMSB             NV_RISCV_CSRU1(DCACHEOP_ADDR_XMSB)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_XLSB             NV_RISCV_CSRU1(DCACHEOP_ADDR_XLSB)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_MODE        NV_RISCV_CSRU1(DCACHEOP_ADDR_MODE)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_MODE_XMSB        NV_RISCV_CSRU1(DCACHEOP_ADDR_MODE_XMSB)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_MODE_XLSB        NV_RISCV_CSRU1(DCACHEOP_ADDR_MODE_XLSB)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_MODE_VA     NV_RISCV_CSRU1(DCACHEOP_ADDR_MODE_VA)
#define NV_RISCV_CSR_XDCACHEOP_ADDR_MODE_PA     NV_RISCV_CSRU1(DCACHEOP_ADDR_MODE_PA)
#define NV_RISCV_CSR_XDCACHEOP_MODE             NV_RISCV_CSRU1(DCACHEOP_MODE)
#define NV_RISCV_CSR_XDCACHEOP_MODE_XMSB             NV_RISCV_CSRU1(DCACHEOP_MODE_XMSB)
#define NV_RISCV_CSR_XDCACHEOP_MODE_XLSB             NV_RISCV_CSRU1(DCACHEOP_MODE_XLSB)
#define NV_RISCV_CSR_XDCACHEOP_MODE_INV_LINE    NV_RISCV_CSRU1(DCACHEOP_MODE_INV_LINE)
#define NV_RISCV_CSR_XFLUSH                     NV_RISCV_CSRU1(FLUSH)
#define NV_RISCV_CSR_XSYSOPEN                   NV_RISCV_CSRX1(SYSOPEN)
#define NV_RISCV_CSR_XMISCOPEN                  NV_RISCV_CSRX1(MISCOPEN)

#define NV_RISCV_CSR_XCFG                       NV_RISCV_CSRX1(CFG)
#define NV_RISCV_CSR_XCFG_XPOSTIO               NV_RISCV_CSRX2(CFG_, POSTIO)
#define NV_RISCV_CSR_XCFG_XPOSTIO_XMSB               NV_RISCV_CSRX2(CFG_, POSTIO_XMSB)
#define NV_RISCV_CSR_XCFG_XPOSTIO_XLSB               NV_RISCV_CSRX2(CFG_, POSTIO_XLSB)
#define NV_RISCV_CSR_XCFG_XPOSTIO_TRUE          NV_RISCV_CSRX2(CFG_, POSTIO_TRUE)
#define NV_RISCV_CSR_XCFG_XPOSTIO_FALSE         NV_RISCV_CSRX2(CFG_, POSTIO_FALSE)

#define NV_RISCV_CSR_XRSP                       NV_RISCV_CSRX1(RSP)
#define NV_RISCV_CSR_XRSP_XRSEC                 NV_RISCV_CSRX2(RSP_, RSEC)
#define NV_RISCV_CSR_XRSP_XRSEC_XMSB                 NV_RISCV_CSRX2(RSP_, RSEC_XMSB)
#define NV_RISCV_CSR_XRSP_XRSEC_XLSB                 NV_RISCV_CSRX2(RSP_, RSEC_XLSB)
#define NV_RISCV_CSR_XRSP_XRSEC_INSEC           NV_RISCV_CSRX2(RSP_, RSEC_INSEC)
#define NV_RISCV_CSR_XRSP_XRSEC_SEC             NV_RISCV_CSRX2(RSP_, RSEC_SEC)

#define NV_RISCV_CSR_XSPM                       NV_RISCV_CSRX1(SPM)
#define NV_RISCV_CSR_XSPM_XSECM                 NV_RISCV_CSRX2(SPM_, SECM)
#define NV_RISCV_CSR_XSPM_XSECM_XMSB                NV_RISCV_CSRX2(SPM_, SECM_XMSB)
#define NV_RISCV_CSR_XSPM_XSECM_XLSB                NV_RISCV_CSRX2(SPM_, SECM_XLSB)
#define NV_RISCV_CSR_XSPM_XSECM_INSEC           NV_RISCV_CSRX2(SPM_, SECM_INSEC)
#define NV_RISCV_CSR_XSPM_XSECM_SEC             NV_RISCV_CSRX2(SPM_, SECM_SEC)

/**
 * @brief CSR read
 *
 * Read from a CSR
 *
 * @param[in] csrnum    CSR address
 *
 * @return              value read from the CSR address
 */
#define csr_read(csrnum) ({ unsigned long _tmp; \
    __asm__ volatile ("csrr %0, %1" : "=r"(_tmp) : "i"(csrnum)); \
    _tmp; })

/**
 * @brief CSR write
 *
 * Write to a CSR
 *
 * @param[in] csrnum    CSR address
 * @param[in] val       value to write
 */
#define csr_write(csrnum, val) ({ \
    __asm__ volatile ("csrw %0, %1" ::"i"(csrnum), "r"(val)); })

/**
 * @brief CSR set
 *
 * Using a mask set the bits of a CSR
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 */
#define csr_set(csrnum, mask) ({ \
    __asm__ volatile ("csrs %0, %1" ::"i"(csrnum), "r"(mask)); })

/**
 * @brief CSR clear
 *
 * Using a mask clear the bits of a CSR
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 */
#define csr_clear(csrnum, mask) ({ \
    __asm__ volatile ("csrc %0, %1" ::"i"(csrnum), "r"(mask)); })

/**
 * @brief CSR atomic read and clear
 *
 * Atomic CSR read and clear
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 *
 * @return              CSR value prior to clear
 */
#define csr_read_and_clear(csrnum, mask) ({ unsigned long _tmp; \
    __asm__ volatile ("csrrc %0, %1, %2" : "=r"(_tmp) :"i"(csrnum), "r"(mask)); \
    _tmp; })

/**
 * @brief CSR atomic read and set
 *
 * Atomic CSR read and set
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 *
 * @return              CSR value prior to set
 */
#define csr_read_and_set(csrnum, mask) ({ unsigned long _tmp; \
    __asm__ volatile ("csrrs %0, %1, %2" : "=r"(_tmp) :"i"(csrnum), "r"(mask)); \
    _tmp; })

/**
 * @brief CSR atomic read and clear immediate
 *
 * Atomic CSR read and clear, where the mask is a 5-bit
 * immediate.
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 *
 * @return              CSR value prior to clear
 */
#define csr_read_and_clear_imm(csrnum, mask) ({ unsigned long _tmp; \
    __asm__ volatile ("csrrc %0, %1, %2" : "=r"(_tmp) :"i"(csrnum), "i"(mask)); \
    _tmp; })

/**
 * @brief CSR atomic read and set immediate
 *
 * Atomic CSR read and set, where the mask is a 5-bit
 * immediate.
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 *
 * @return              CSR value prior to set
 */
#define csr_read_and_set_imm(csrnum, mask) ({ unsigned long _tmp; \
    __asm__ volatile ("csrrs %0, %1, %2" : "=r"(_tmp) :"i"(csrnum), "i"(mask)); \
    _tmp; })

/**
 * @brief CSR atomic read and write immediate
 *
 * Atomic CSR read and write, where the mask is a 5-bit
 * immediate.
 *
 * @param[in] csrnum    CSR address
 * @param[in] mask      bit mask
 *
 * @return              CSR value prior to write
 */
#define csr_read_and_write_imm(csrnum, mask) ({ unsigned long _tmp; \
    __asm__ volatile ("csrrwi %0, %1, %2" : "=r"(_tmp) :"i"(csrnum), "i"(mask)); \
    _tmp; })

#if (defined(FSP_COVERITY) && (FSP_COVERITY==1))
_Pragma("coverity compliance end_block(include) \
         MISRA_C_2012_Rule_20_10 MISRA_C_2012_Rule_2_5")
#endif

#endif /* FSP__CPU__CSR_H */
/** end of file **/
