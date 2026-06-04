# SPDX-FileCopyrightText: 2026 AlloSphere Research Group <allosphere@ucsb.edu>
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.30)

project(al_ext DESCRIPTION "Extensions for allolib" LANGUAGES CXX VERSION 0.0.1)

message(STATUS "[al_ext] configuring allolib extension modules..")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(NoInSourceBuilds)
include(SetIPO)

option(
  AL_EXT_BUILD_TESTS
  "Enable building tests for al_ext"
  ${PROJECT_IS_TOP_LEVEL}
)

option(
  AL_EXT_BUILD_EXAMPLES
  "Enable building examples for al_ext"
  ${PROJECT_IS_TOP_LEVEL}
)

option(AL_EXT_BUILD_VIDEO "Enable building video extension" OFF)

# remove all extensions from default build
set_directory_properties(
  PROPERTIES EXCLUDE_FROM_ALL ON EXCLUDE_FROM_DEFAULT_BUILD ON
)

if(AL_EXT_BUILD_VIDEO)
  add_subdirectory(modules/video)
endif()

message(STATUS "[al_ext] ** DONE Processing extensions **")
