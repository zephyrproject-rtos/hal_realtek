/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WLANCONFIG_H
#define WLANCONFIG_H

/*
 * Include user defined options first. Anything not defined in these files
 * will be set to standard values. Override anything you dont like!
 */
#include "platform_autoconf.h"

#ifndef CONFIG_LITTLE_ENDIAN
#define CONFIG_LITTLE_ENDIAN
#endif

#define WIFI_LOGO_CERTIFICATION 0

#define MACID_HW_MAX_NUM 16
#define NET_IF_NUM 2

#endif //WLANCONFIG_H
