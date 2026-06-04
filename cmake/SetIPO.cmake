# SPDX-FileCopyrightText: 2026 Kon Hyong Kim <konhyong@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported)
if(ipo_supported)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION True)
  message(STATUS "Enabling Interprocedural Optimization")
endif()
