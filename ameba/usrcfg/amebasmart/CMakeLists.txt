# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)
zephyr_library_sources_ifdef(CONFIG_MCUBOOT
	ameba_bootcfg.c
	ameba_flashcfg.c
	ameba_boot_trustzonecfg.c
)
zephyr_library_sources_ifdef(CONFIG_PM ameba_sleepcfg.c)
