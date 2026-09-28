# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_py "${CMAKE_CURRENT_LIST_DIR}/toolchain_check.py")
if(NOT EXISTS "${_py}")
  message(FATAL_ERROR "toolchain_check.py not found: ${_py}")
endif()

if(NOT DEFINED CONFIG_SOC_SERIES OR "${CONFIG_SOC_SERIES}" STREQUAL "")
  message(FATAL_ERROR "CONFIG_SOC_SERIES is not set")
endif()

execute_process(
  COMMAND ${Python3_EXECUTABLE} "${_py}" --chip "${CONFIG_SOC_SERIES}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
  RESULT_VARIABLE _ret
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)

if(_out)
  message(STATUS "${_out}")
endif()
if(NOT _ret EQUAL 0)
  message(WARNING "${_err}")
endif()
