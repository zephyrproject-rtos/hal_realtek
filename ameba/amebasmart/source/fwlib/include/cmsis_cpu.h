/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CMSIS_CPU_H__
#define __CMSIS_CPU_H__

#include "ameba_vector_table.h"

#if defined (CONFIG_ARM_CORE_CM4)

/* ========  Configuration of Core Peripherals  ================================== */
#define __CM55_REV                0x0001U   /* Core revision r0p1 */
// #define __ARMv81MML_REV           0x0001U   /* Core revision r0p1 */
#define __SAUREGION_PRESENT       1U        /* SAU regions present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __NVIC_PRIO_BITS          3U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1U        /* FPU present */
#define __FPU_DP                  0U        /* double precision FPU */
#define __DSP_PRESENT             1U        /* DSP extension present */
#define __PMU_PRESENT             1U        /* PMU present */
#define __PMU_NUM_EVENTCNT        4U        /* Number of PMU event counters */
#define __ICACHE_PRESENT          1U        /* Instruction Cache present */
#define __DCACHE_PRESENT          1U        /* Data Cache present */

/* CM33 does not support Cache and PMU */
#include "core_cm55.h"                 /* Processor and core peripherals */
// #include "core_armv81mml.h"
#include "cmsis_ridr.h"

#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
#include <arm_cmse.h>
#endif /* __ARM_FEATURE_CMSE */

#elif defined (CONFIG_ARM_CORE_CM0)

/* --------  Configuration of the Ameba KM0 (Armv8-M Baseline) core  -------------- */
// #define __CM23_REV                0x0000U   /* Core revision r0p1 */
#define __ARMv8MBL_REV            0x0000U   /* Core revision r0p0 */
#define __MPU_PRESENT             1U        /* MPU present */
#define __SAUREGION_PRESENT       0U        /* SAU regions present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __NVIC_PRIO_BITS          2U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             0U        /* no FPU present */
#define __DSP_PRESENT             0U        /* no DSP extension present */

#define RTK_DCACHE_2WAY           1U        /* 2-way Cache */

/*
 * The RTK __NVIC_SetPriority() in core_armv8mbl.h clamps to MAX_IRQ_PRIORITY_VALUE.
 * That macro is normally provided by ameba_vector.h (3 for KM0, since
 * __NVIC_PRIO_BITS == 2), but ameba_vector.h is included after this core header,
 * so define it here first.  Guarded and kept identical to ameba_vector.h's value
 * so the later (unguarded) definition there is a harmless identical redefinition.
 */
#ifndef MAX_IRQ_PRIORITY_VALUE
#define MAX_IRQ_PRIORITY_VALUE    3
#endif

/*
 * The Ameba KM0 core is Armv8-M baseline (Cortex-M23 compatible) but, unlike a
 * stock Cortex-M23, implements an RTK 2-way cache that exposes ARMv7-M-style SCB
 * cache-maintenance registers.  The vendored core_armv8mbl.h is the ARM CMSIS
 * Armv8-M Baseline core header extended with those SCB cache registers (the
 * upstream vendor's "core_cm23_km0.h" is not shipped in this tree).  It provides
 * the SCB cache registers that armv7m_cachel1.h relies on, so the stock CMSIS
 * cache helper compiles and ameba_cache.h / ameba_ipc_api.c resolve.
 */
#include "core_armv8mbl.h"

/*
 * Cache present at the core level (matches CPU_HAS_I/DCACHE forced by the SoC
 * series), so keep __I/DCACHE_PRESENT=1 to satisfy Zephyr's cmsis_core_m.h
 * consistency check.
 */
#define __ICACHE_PRESENT          1U        /* Instruction Cache present */
#define __DCACHE_PRESENT          1U        /* Data Cache present */

/*
 * ##########################  Cache functions  ####################################
 * core_armv8mbl.h (Armv8-M baseline) does not pull in the ARMv7-M cache helper
 * the way core_cm55.h does, so include it explicitly (CMSIS-6 layout).
 */
#if ((defined (__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)) || \
     (defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)))
#include "m-profile/armv7m_cachel1.h"
#endif

/*
 * ##########################   MPU functions  #####################################
 * Likewise, core_armv8mbl.h defines the MPU_Type registers but not the ARM_MPU_*
 * helper functions.  core_cm55.h pulls in m-profile/armv8m_mpu.h for those; do
 * the same here so Zephyr's Armv8-M MPU driver (arm_mpu.c) links.
 */
#if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)
#include "m-profile/armv8m_mpu.h"
#endif

#elif defined (CONFIG_ARM_CORE_CA32)

#include <string.h>
#include <stdlib.h>

#define __FPU_PRESENT			1
#define __CORTEX_A				7
#include "core_ca.h"
#include "cmsis_cp15.h"

// #include "irq_ctrl.h"

#endif

#endif /* __CMSIS_CPU_H__ */
