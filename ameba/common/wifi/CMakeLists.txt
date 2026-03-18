# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(source)
zephyr_include_directories(../../usrcfg/common)

zephyr_library_sources_ifdef(CONFIG_WIFI source/wifi_feature_dis_antdiv.c)
zephyr_library_sources_ifdef(CONFIG_WIFI source/wifi_init_zephyr.c)
