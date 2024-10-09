/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef MISC__MATH_SECURE_H
#define MISC__MATH_SECURE_H
#define FSP__MISC__MATH_SECURE_H    1

/* Compiler headers */
#include <stdint.h>                 // for uint32_t, uint64_t, UINT32_MAX

/* Early FSP headers */
#include <misc/ct-assert.h>         // for CT_ASSERT

/* Hardware headers */

/* Late FSP headers */
#include <debug/abort.h>            // for tegra_abort, FSP__DEBUG__ABORT_H
#include <debug/abort-sys-codes.h>  // for ABORT_ASSERT, FSP__DEBUG__ABORT_S...
#include <misc/attributes.h>        // for UNUSED, FSP__MISC__ATTRIBUTES_H

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
CT_ASSERT(FSP__MISC__ATTRIBUTES_H, "Header file missing or invalid.")
CT_ASSERT(FSP__BASE__MODULE_ID_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_H, "Header file missing or invalid.")
CT_ASSERT(FSP__DEBUG__ABORT_SYS_CODES_H, "Header file missing or invalid.")

/**
 * @file fsp-math-secure.h
 * @brief functions that provide secure arithmetic operations on unsigned
 * types to ensure protection from wrapping.
 */

/**
 * Note: This minimalistic library is only useful for detecting and protecting
 * against overflow and not for a specific maxmum value.
 */

/**
 * @brief overflow action modes
 *
 * @macro-title Overflow Action Modes
 */
#define OVERFLOW_ACTION_0       0U
#define OVERFLOW_ACTION_MAX     1U
#define OVERFLOW_ACTION_VAL     2U
#define OVERFLOW_ACTION_ABORT   3U
#define OVERFLOW_ACTION_MODES   4U

/**
 * @brief error codes for overflow
 *
 * @macro-title Error Codes for Unsigned 32-bit Overflow
 */
#define E_UNSIGNED_OVERFLOW     MODULE_ERROR_VAL(SMATH, 0UL)

typedef uint32_t overflow_action_t;
typedef uint64_t overflow_action64_t;

/**
 * @brief overflow action function
 *
 * This is used to define the overflow action function that can be
 * is called when a user specifies an overflow action mode.
 *
 * @param[in] val value that resulted in an overflow
 *
 * @return 32-bit unsigned value
 */
typedef uint32_t (overflow_action)(uint32_t val);
typedef uint64_t (overflow_action64)(uint64_t val);

/**
 * @brief returns 0 when an overflow occurs
 *
 * This is used to return 0 when an overflow occurs.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return zero
 */
static inline uint32_t action_overflow_0(uint32_t val)
{
    UNUSED(val);

    return 0U;
}

/**
 * @brief returns the max value when an overflow occurs
 *
 * This is used to return the max uint32_t value when an overflow occurs.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return uint32_t max value
 */
static inline uint32_t action_overflow_max(uint32_t val)
{
    UNUSED(val);

    return UINT32_MAX;
}

/**
 * @brief returns the overflown value when an overflow occurs
 *
 * This is used to return the actual resulting value after the overflow.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return overflown value
 */
static inline uint32_t action_overflow_val(uint32_t val)
{
    return val;
}

/**
 * @brief abort when an overflow occurs
 *
 * This is used to abort the execution when an overflow is detected
 * and abort is specified as the resultant action of that.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return none
 */
static inline uint32_t action_overflow_abort(uint32_t val)
{
    UNUSED(val);

    tegra_abort(ABORT_ASSERT, E_UNSIGNED_OVERFLOW);

    return 0U;
}

static overflow_action *const overflow_actions[OVERFLOW_ACTION_MODES] = {
    [OVERFLOW_ACTION_0]     =   &action_overflow_0,
    [OVERFLOW_ACTION_MAX]   =   &action_overflow_max,
    [OVERFLOW_ACTION_VAL]   =   &action_overflow_val,
    [OVERFLOW_ACTION_ABORT] =   &action_overflow_abort,
};

/**
 * @brief returns 0 when an overflow occurs
 *
 * This is used to return 0 when an overflow occurs.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return zero
 */
static inline uint64_t action_overflow64_0(uint64_t val)
{
    UNUSED(val);

    return 0U;
}

/**
 * @brief returns the max value when an overflow occurs
 *
 * This is used to return the max uint64_t value when an overflow occurs.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return uint64_t max value
 */
static inline uint64_t action_overflow64_max(uint64_t val)
{
    UNUSED(val);

    return UINT64_MAX;
}

/**
 * @brief returns the overflown value when an overflow occurs
 *
 * This is used to return the actual resulting value after the overflow.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return overflown value
 */
static inline uint64_t action_overflow64_val(uint64_t val)
{
    return val;
}

/**
 * @brief abort when an overflow occurs
 *
 * This is used to abort the execution when an overflow is detected
 * and abort is specified as the resultant action of that.
 *
 * @param[in] val resulting value of an operation that caused the wrap/overflow
 *
 * @return none
 */
static inline uint64_t action_overflow64_abort(uint64_t val)
{
    UNUSED(val);

    tegra_abort(ABORT_ASSERT, E_UNSIGNED_OVERFLOW);

    return 0U;
}

static overflow_action64 *const overflow_actions64[OVERFLOW_ACTION_MODES] = {
    [OVERFLOW_ACTION_0]     =   &action_overflow64_0,
    [OVERFLOW_ACTION_MAX]   =   &action_overflow64_max,
    [OVERFLOW_ACTION_VAL]   =   &action_overflow64_val,
    [OVERFLOW_ACTION_ABORT] =   &action_overflow64_abort,
};

/**
 * @brief securely cast an unsigned 32-bit to unsigned 8-bit
 *
 * This is used to securely cast an unsigned 32-bit number to 8-bit
 * number by taking care of wrapping.
 *
 * @param[in] a unsigned 32-bit number
 *
 * @return unsigned 8-bit number
 */
static inline uint8_t scast_u32_to_u8(uint32_t a)
{
    return (a > (uint32_t)UINT8_MAX) ? 0U : (uint8_t)a;
}

/**
 * @brief securely cast an unsigned 64-bit to unsigned 64-bit
 *
 * This is used to securely cast an unsigned 64-bit number to 32-bit
 * number by taking care of wrapping.
 *
 * @param[in] a unsigned 64-bit number
 *
 * @return unsigned 32-bit number
 */
static inline uint32_t scast_u64_to_u32(uint64_t a)
{
    return (a > (uint64_t)UINT32_MAX) ? 0U : (uint32_t)a;
}

/**
 * @brief secure addition of unsigned 32-bit numbers
 *
 * This is used to add unsigned 32 bit numbers by taking care of
 * wrapping using postcondition test solution.
 *
 * @param[in] a operand1
 * @param[in] b operand2
 *
 * @return sum of the 2 params passed or 0 if wrapped
 */
static inline uint32_t sadd_u32(uint32_t a, uint32_t b, overflow_action_t action)
{
    uint32_t sum = a + b;

    return (sum >= a) ? sum : overflow_actions[action](sum);
}

/**
 * @brief secure substraction of unsigned 32-bits
 *
 * This is used to substract unsigned 32 bit numbers by taking care
 * of wrapping.
 *
 * @param[in] a operand1
 * @param[in] b operand2
 *
 * @return  diff of the 2 params passed or 0 if wrapped
 */
static inline uint32_t ssub_u32(uint32_t a, uint32_t b, overflow_action_t action)
{
    return (a > b) ? (a - b) : overflow_actions[action](a - b);
}

/**
 * @brief secure multiplication of unsigned 32-bits
 *
 * This is used to multiply unsigned 32 bit numbers by taking care
 * of wrapping.
 *
 * @param[in] a operand1
 * @param[in] b operand2
 *
 * @return product of the 2 params passed or 0 if wrapped
 */
static inline uint32_t smul_u32(uint32_t a, uint32_t b, overflow_action_t action)
{
    uint32_t res = 0U;

    if ((a == 0U) || (b == 0U)) {
        goto out;
    }

    res = (a < (UINT32_MAX / b)) ? (a * b) : overflow_actions[action](a * b);

out:
    return res;
}

/**
 * @brief secure addition of unsigned 64-bit numbers
 *
 * This is used to add unsigned 64 bit numbers by taking care of
 * wrapping using postcondition test solution.
 *
 * @param[in] a operand1
 * @param[in] b operand2
 *
 * @return sum of the 2 params passed or 0 if wrapped
 */
static inline uint64_t sadd_u64(uint64_t a, uint64_t b, overflow_action64_t action)
{
    uint64_t sum = a + b;

    return (sum >= a) ? sum : overflow_actions[action](sum);
}

/**
 * @brief secure substraction of unsigned 64-bits
 *
 * This is used to substract unsigned 64 bit numbers by taking care
 * of wrapping.
 *
 * @param[in] a operand1
 * @param[in] b operand2
 *
 * @return  diff of the 2 params passed or 0 if wrapped
 */
static inline uint64_t ssub_u64(uint64_t a, uint64_t b, overflow_action64_t action)
{
    return (a > b) ? (a - b) : overflow_actions[action](a - b);
}

/**
 * @brief secure multiplication of unsigned 64-bits
 *
 * This is used to multiply unsigned 64 bit numbers by taking care
 * of wrapping.
 *
 * @param[in] a operand1
 * @param[in] b operand2
 *
 * @return product of the 2 params passed or 0 if wrapped
 */
static inline uint64_t smul_u64(uint64_t a, uint64_t b, overflow_action64_t action)
{
    uint64_t res = 0U;

    if ((a == 0U) || (b == 0U)) {
        goto out;
    }

    res = (a < (UINT64_MAX / b)) ? (a * b) : overflow_actions[action](a * b);

out:
    return res;
}

#endif /* FSP__MISC__MATH_SECURE_H */
