# Copyright (c) 2026, Realtek Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

set(STUB_DIR ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/bee/drivers/stubs)
set(USB_STUB_FILE ${STUB_DIR}/rtl87x2g/usb_stubs.c)
set(USB_LIB_PATH
    ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/bee/drivers/rtl87x2g/usb/librtl87x2g_usb.a
)
set(USB_SDK_LIB_PATH
    ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/bee/drivers/rtl87x2g/sdk/librtl87x2g_sdk.a
)

add_blob_or_stub(
    ${ZEPHYR_CURRENT_LIBRARY}
    usb
    "${USB_LIB_PATH}"
    "${USB_STUB_FILE}"
)

if(NOT CONFIG_BUILD_ONLY_NO_BLOBS AND EXISTS "${USB_SDK_LIB_PATH}")
    target_link_libraries(${ZEPHYR_CURRENT_LIBRARY} PUBLIC "${USB_SDK_LIB_PATH}")
endif()
