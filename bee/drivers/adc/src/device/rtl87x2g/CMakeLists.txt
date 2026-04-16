# Copyright (c) 2026, Realtek Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

file(GLOB ADC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
zephyr_library_sources(${ADC_SOURCES})
zephyr_include_directories(.)

set(STUB_DIR ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/bee/drivers/stubs)
set(ADC_STUB_FILE ${STUB_DIR}/rtl87x2g/adc_stubs.c)
set(ADC_LIB_PATH
  ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/bee/drivers/rtl87x2g/adc/librtl87x2g_adc.a
)

add_blob_or_stub(
    ${ZEPHYR_CURRENT_LIBRARY}
    adc
    "${ADC_LIB_PATH}"
    "${ADC_STUB_FILE}"
)
