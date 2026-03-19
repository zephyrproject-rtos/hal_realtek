/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AMEBA_RESET_H_
#define _AMEBA_RESET_H_

/* AUTO_GEN_START */


/* AUTO_GEN_END */

#define	AON_BIT_RSTF_DSLP		BIT_BOOT_DSLP_RESET_HAPPEN
#define	AON_BIT_RSTF_BOR		BIT_BOOT_BOD_RESET_HAPPEN

#define IS_WDG_RESET(reason)	((reason) & (BIT_BOOT_WDG_RESET_HAPPEN | BIT_BOOT_KM4WDG_RESET_HAPPEN))
#define IS_SYS_RESET(reason)	((reason) & (BIT_BOOT_SYS_RESET_HAPPEN | BIT_BOOT_KM4SYS_RESET_HAPPEN))

extern void System_Reset(void);

#endif
