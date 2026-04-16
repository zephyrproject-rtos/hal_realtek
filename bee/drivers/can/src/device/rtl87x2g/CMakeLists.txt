# Copyright (c) 2026, Realtek Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

set(STUB_DIR ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/bee/drivers/stubs)
set(CAN_STUB_FILE ${STUB_DIR}/rtl87x2g/can_stubs.c)
set(CAN_LIB_PATH
    ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/bee/drivers/rtl87x2g/can/librtl87x2g_can.a
)

add_blob_or_stub(
    ${ZEPHYR_CURRENT_LIBRARY}
    can
    "${CAN_LIB_PATH}"
    "${CAN_STUB_FILE}"
)
