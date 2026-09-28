# Copyright (c) 2026, Realtek Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_RTL87X2G inc/rtl87x2g)

add_subdirectory_ifdef(CONFIG_REALTEK_BEE_EFUSE src)
