/* Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef WATCHDOG__WATCHDOG_STATISTICS_H
#define WATCHDOG__WATCHDOG_STATISTICS_H
#define FSP__WATCHDOG__WATCHDOG_STATISTICS_H                 1

/* Compiler headers */
#include <stdint.h>

/* Early FSP headers */

/* Hardware headers */

/* Late FSP headers */

/* Module-specific FSP headers */

/**
 * @file watchdog-statistics.h
 * @brief Definitions for Watchdog Statistics
 */

/**
 * @brief Number of watchdog histogram buckets
 *
 * This is defined as a constant but all of the code really assumes
 * that the histogram is decimal based.
 */
#define WATCHDOG_STATS_NUM_BUCKETS      10U

/**
 * @brief Watchdog Statistics Structure
 */
typedef struct {
    uint64_t            count;
    uint64_t            accumulatedTime;
    uint64_t            minTime;
    uint64_t            maxTime;
    uint64_t            histogram[WATCHDOG_STATS_NUM_BUCKETS];
} WatchdogStatistics;

/**
 * @brief Watchdog Task Statistics Structure
 */
typedef struct {
    uint64_t            maxItemsInQueue;
    WatchdogStatistics  itemStats;
} WatchdogTaskStatistics;

#endif
