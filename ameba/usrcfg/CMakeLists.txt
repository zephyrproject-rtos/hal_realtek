# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# common/ holds the host-flavor wifi_user_conf; the NP build resolves it from
# the ameba-rtos SDK instead (see common/wifi/CMakeLists.txt).
if(NOT CONFIG_AS_INIC_NP)
  zephyr_include_directories(common)
endif()

add_subdirectory(${CONFIG_SOC_SERIES})
