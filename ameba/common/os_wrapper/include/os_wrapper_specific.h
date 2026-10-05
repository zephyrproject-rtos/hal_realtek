/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __OS_WRAPPER_SPECIFIC_H__
#define __OS_WRAPPER_SPECIFIC_H__

#define RTOS_CONVERT_MS_TO_TICKS(MS) k_ms_to_cyc_floor32(MS)

#ifndef configNUM_CORES
#define configNUM_CORES CONFIG_MP_MAX_NUM_CPUS
#endif

#endif
