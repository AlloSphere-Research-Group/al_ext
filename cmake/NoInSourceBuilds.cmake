# SPDX-FileCopyrightText: 2026 Kon Hyong Kim <konhyong@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
    "\nIn-source builds are not allowed.\n"
    "Remove CMakeFiles/, CMakeCache.txt and retry with a path to build tree:\n"
    " e.g.) cmake -B <destination>\n"
  )
endif()
