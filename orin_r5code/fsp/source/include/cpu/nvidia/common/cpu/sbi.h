/**
 * @file sbi.h
 *
 * @brief NVRISC-V Software Binary Interface (SBI)
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

#ifndef CPU__SBI_H
#define CPU__SBI_H
#define FSP__CPU__SBI_H    1

/*
 * @file sbi.h
 * NVRISC-V Software Binary Interface specification
 */

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */
#include <misc/gcc_attrs.h>

/* Hardware headers */

/* Late FSP headers */
#include <misc/macros.h>                   // for STR

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
START_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
                MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
_Static_assert(FSP__MISC__MACROS_H, "Header file missing or invalid.");

//! @brief Standardized SBI error codes.
typedef enum
{
    SBI_SUCCESS             = 0,
    SBI_ERR_FAILURE         = -1,
    SBI_ERR_NOT_SUPPORTED   = -2,
    SBI_ERR_INVALID_PARAM   = -3,
    SBI_ERR_DENIED          = -4,
    SBI_ERR_INVALID_ADDRESS = -5,

    SBI_ERROR_CODE__PADDING = -0x7FFFFFFFFFFFFFFFLL, // force to be signed 64bit type
} SBI_ERROR_CODE;

_Static_assert(sizeof(SBI_ERROR_CODE) == sizeof(int64_t), "SBI_ERROR_CODE size must be 8 bytes.");

//! @brief SBI extension IDs.
typedef enum
{
    /*! @brief Legacy set_timer extension
     *
     * void sbi_set_timer(uint64_t stime_value);
     *
     * @param[in]   stime_value     New mtimecmp value
     */
    SBI_EXTENSION_SET_TIMER = 0x00,

    /*! @brief Legacy shutdown extension
     *
     * NORETURN void sbi_shutdown(void);
     */
    SBI_EXTENSION_SHUTDOWN = 0x08,

    /*! @brief NVIDIA-specified extension.
     *
     * Function IDs are of the enum type SBI_NVIDIA_FUNCTION_ID.
     */
    SBI_EXTENSION_NVIDIA = 0x090001EB,
} SBI_EXTENSION_ID;

//! @brief NVIDIA-specific SBI functions.
typedef enum
{
    SBI_NVFUNC_FIRST                 = 0,
    SBI_NVFUNC_PARTITION_SWITCH      = 0,
    SBI_NVFUNC_RELEASE_PRIV_LOCKDOWN = 1,
    SBI_NVFUNC_TRACECTL_SET          = 2,
    SBI_NVFUNC_FBIF_TRANSCFG_SET     = 3,
    SBI_NVFUNC_FBIF_REGIONCFG_SET    = 4,
    SBI_NVFUNC_TFBIF_TRANSCFG_SET    = 5,
    SBI_NVFUNC_TFBIF_REGIONCFG_SET   = 6,
} SBI_NVIDIA_FUNCTION_ID;

//! @brief SBI return value structure.
typedef struct
{
    //! @brief Error code.
    int64_t error;
    //! @brief Return value from the SBI call (or 0 if a void type SBI call).
    int64_t value;
} SBI_RETURN_VALUE;

//-------------------------------------------------------------------------------------------------
#define SBICALL GCC_ATTR_ALWAYSINLINE static inline
#define CLOBBER_SBICALL "s0", "s1", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "memory"
/* For some reason marking TP and GP as clobbers doesn't actually work, so we manually save them */
#define SBICALL_BODY \
  "mv s0, gp\n" \
  "mv s1, tp\n" \
  "ecall\n" \
  "mv tp, s1\n" \
  "mv gp, s0\n" \

/**
 * @brief SBI Call
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall0( int32_t extension, int32_t function )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0_out ), "+r"( a1_out ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL, "a2", "a3", "a4", "a5");

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

/**
 * @brief SBI Call with 1 Function Argument
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 * @param[in] arg0          function argument
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall1( int32_t extension, int32_t function, uint64_t arg0 )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    register uint64_t a0 __asm__( "a0" ) = arg0;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0 ), "+r"( a1_out ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL, "a2", "a3", "a4", "a5");

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

/**
 * @brief SBI Call with 2 Function Arguments
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 * @param[in] arg0..arg1    function arguments
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall2( int32_t extension, int32_t function, uint64_t arg0, uint64_t arg1 )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    register uint64_t a1 __asm__( "a1" ) = arg1;
    register uint64_t a0 __asm__( "a0" ) = arg0;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0 ), "+r"( a1 ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL, "a2", "a3", "a4", "a5");

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

/**
 * @brief SBI Call with 3 Function Arguments
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 * @param[in] arg0..arg2    function arguments
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall3( int32_t extension, int32_t function, uint64_t arg0, uint64_t arg1, uint64_t arg2 )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    register uint64_t a2 __asm__( "a2" ) = arg2;
    register uint64_t a1 __asm__( "a1" ) = arg1;
    register uint64_t a0 __asm__( "a0" ) = arg0;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0 ), "+r"( a1 ), "+r"( a2 ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL, "a3", "a4", "a5");

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

/**
 * @brief SBI Call with 4 Function Arguments
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 * @param[in] arg0..arg3    function arguments
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall4( int32_t extension, int32_t function, uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3 )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    register uint64_t a3 __asm__( "a3" ) = arg3;
    register uint64_t a2 __asm__( "a2" ) = arg2;
    register uint64_t a1 __asm__( "a1" ) = arg1;
    register uint64_t a0 __asm__( "a0" ) = arg0;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0 ), "+r"( a1 ), "+r"( a2 ), "+r"( a3 ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL, "a4", "a5");

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

/**
 * @brief SBI Call with 5 Function Arguments
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 * @param[in] arg0..arg4    function arguments
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall5( int32_t extension, int32_t function, uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4 )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    register uint64_t a4 __asm__( "a4" ) = arg4;
    register uint64_t a3 __asm__( "a3" ) = arg3;
    register uint64_t a2 __asm__( "a2" ) = arg2;
    register uint64_t a1 __asm__( "a1" ) = arg1;
    register uint64_t a0 __asm__( "a0" ) = arg0;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0 ), "+r"( a1 ), "+r"( a2 ), "+r"( a3 ), "+r"( a4 ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL, "a5");

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

/**
 * @brief SBI Call with 6 Function Arguments
 *
 * @param[in] extension     sbi extension id
 * @param[in] function      sbi function id
 * @param[in] arg0..arg5    function arguments
 *
 * @return                  sbi return struct
 */
SBICALL SBI_RETURN_VALUE sbicall6( int32_t extension, int32_t function, uint64_t arg0, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5 )
{
    register int64_t a0_out __asm__( "a0" ) = 0L;
    register int64_t a1_out __asm__( "a1" ) = 0L;

    register int32_t a7 __asm__( "a7" ) = extension;
    register int32_t a6 __asm__( "a6" ) = function;
    register uint64_t a5 __asm__( "a5" ) = arg5;
    register uint64_t a4 __asm__( "a4" ) = arg4;
    register uint64_t a3 __asm__( "a3" ) = arg3;
    register uint64_t a2 __asm__( "a2" ) = arg2;
    register uint64_t a1 __asm__( "a1" ) = arg1;
    register uint64_t a0 __asm__( "a0" ) = arg0;
    __asm__ volatile (SBICALL_BODY
                  : "+r"( a0 ), "+r"( a1 ), "+r"( a2 ), "+r"( a3 ), "+r"( a4 ), "+r"( a5 ), "+r"( a6 ), "+r"( a7 )
                  :
                  : "ra", CLOBBER_SBICALL);

    return (SBI_RETURN_VALUE) { .error = a0_out, .value = a1_out };
}

END_RFD_BLOCK(MISRA, DEVIATE, Rule_2_3, "Approval:  On-file, DR: SWE-FSP-010-SWSADR.docx",
              MISRA, DEVIATE, Directive_4_9, "Approval: Bug 200531995, DR: SWE-FSP-012-SWSADR.docx")
#endif /* CPU__SBI_H */
