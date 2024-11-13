/*
**             Copyright 2023 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/
#ifndef TIMESPEC_H
#define TIMESPEC_H

#ifdef __KERNEL__

/**** Linux kernel ****/
#include <linux/version.h>
#include <linux/types.h>
#include <linux/jiffies.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)
#define jiffies_to_timespec64(x, y) jiffies_to_timespec((x), (y))
#endif /* LINUX_VERSION_CODE < 4.3 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#include <linux/time.h>
typedef struct timespec timespec_t;
#define timespec64_add_ns(timespec, ns) timespec_add_ns((timespec), (ns))
#else
#include <linux/time64.h>
typedef struct timespec64 timespec_t;
#endif /* LINUX_VERSION_CODE < 3.17 */

// Wrapper macros for portability
#define TIMESPEC_GET(timespec)                  (jiffies_to_timespec64(jiffies, &(timespec)))
#define TIMESPEC_ADD_MSEC(timespec, ms)         (timespec64_add_ns((&timespec), (ms)*1000 * 1000))
#define TIMESPEC_IS_AFTER(timespec)             (timespec_is_after(&(timespec)))
#define TIMESPEC_IS_AFTER_EQ(timespec)          (timespec_is_after_eq(&(timespec)))
#define TIMESPEC_IS_BEFORE(timespec)            (timespec_is_before(&(timespec)))
#define TIMESPEC_IS_BEFORE_EQ(timespec)         (timespec_is_before_eq(&(timespec)))
#define TIMESPEC_MS_DELTA(timespec1, timespec2) (timespec_ms_delta((timespec1), (timespec2)))
#define TIMESPEC_MS_DELTA_NOW(timespec)         (timespec_ms_delta_now((timespec)))
#else /* #ifdef __KERNEL__ */

/**** Linux user-space ****/
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "user_kernel.h"

typedef struct timespec timespec_t;

// Wrapper macros for portability
#define TIMESPEC_GET(timespec)                  (clock_gettime(CLOCK_MONOTONIC, &(timespec)))
#define TIMESPEC_ADD_MSEC(timespec, ms)         (timespec_add_ms((&timespec), (ms)))
#define TIMESPEC_IS_AFTER(timespec)             (timespec_is_after(&(timespec)))
#define TIMESPEC_IS_AFTER_EQ(timespec)          (timespec_is_after_eq(&(timespec)))
#define TIMESPEC_IS_BEFORE(timespec)            (timespec_is_before(&(timespec)))
#define TIMESPEC_IS_BEFORE_EQ(timespec)         (timespec_is_before_eq(&(timespec)))
#define TIMESPEC_MS_DELTA(timespec1, timespec2) (timespec_ms_delta((timespec1), (timespec2)))
#define TIMESPEC_MS_DELTA_NOW(timespec)         (timespec_ms_delta_now((timespec)))

#endif /* __KERNEL__ */

static inline void timespec_add_ms(timespec_t *timespec, u32 ms)
{
    timespec->tv_sec += ms / 1000;
    timespec->tv_nsec += (ms % 1000) * 1000000;
    if (timespec->tv_nsec >= 1000000000L) {
        timespec->tv_sec++;
        timespec->tv_nsec -= 1000000000L;
    }
}

/*
 * lhs < rhs:  return <0
 * lhs == rhs: return 0
 * lhs > rhs:  return >0
 */
static inline int timespec_cmp(const timespec_t *lhs, const timespec_t *rhs)
{
    if (lhs->tv_sec < rhs->tv_sec)
        return -1;
    if (lhs->tv_sec > rhs->tv_sec)
        return 1;
    return lhs->tv_nsec - rhs->tv_nsec;
}

/*
 * timspec < now:  return <0
 * timespec == now: return 0
 * timespec > now:  return >0
 */
static inline int timespec_cmp_now(const timespec_t *timespec)
{
    timespec_t now;

    TIMESPEC_GET(now);
    return timespec_cmp(timespec, &now);
}

static inline bool timespec_is_after(const timespec_t *timespec)
{
    return timespec_cmp_now(timespec) < 0;
}

static inline bool timespec_is_after_eq(const timespec_t *timespec)
{
    return timespec_cmp_now(timespec) <= 0;
}

static inline bool timespec_is_before(const timespec_t *timespec)
{
    return timespec_cmp_now(timespec) > 0;
}

static inline bool timespec_is_before_eq(const timespec_t *timespec)
{
    return timespec_cmp_now(timespec) >= 0;
}

/* delta = lhs - rhs */
static inline timespec_t timespec_delta(timespec_t lhs, timespec_t rhs)
{
    timespec_t ts_delta;

    ts_delta.tv_sec = lhs.tv_sec - rhs.tv_sec;
    ts_delta.tv_nsec = lhs.tv_nsec - rhs.tv_nsec;

    return ts_delta;
}

static inline u32 timespec_to_ms(timespec_t timespec)
{
    return timespec.tv_sec * 1000 + (timespec.tv_nsec / 1000 / 1000);
}

static inline u32 timespec_ms_delta(timespec_t lhs, timespec_t rhs)
{
    return timespec_to_ms(timespec_delta(lhs, rhs));
}

static inline u32 timespec_ms_delta_now(const timespec_t timespec)
{
    timespec_t now;

    TIMESPEC_GET(now);
    return timespec_ms_delta(now, timespec);
}

#endif /* TIMESPEC_H */
