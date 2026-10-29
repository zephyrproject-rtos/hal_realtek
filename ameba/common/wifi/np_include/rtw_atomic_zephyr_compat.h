/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pre-included for SDK wifi sources compiled under Zephyr: os_wrapper.h pulls
 * <zephyr/kernel.h>, whose atomic_t typedef clashes with the vendor
 * rtw_atomic.h one.  Zephyr's atomic_t (long) is layout-compatible with the
 * vendor struct { volatile int } used for the sk_buff refcount, so suppress
 * the vendor header.
 */
#ifndef RTW_ATOMIC_ZEPHYR_COMPAT_H
#define RTW_ATOMIC_ZEPHYR_COMPAT_H

#include <zephyr/sys/atomic.h>

_Static_assert(sizeof(atomic_t) == 4, "vendor ABI expects a 4-byte atomic_t");

#define __RTW_ATOMIC_H_
#define ATOMIC_T atomic_t

#endif
