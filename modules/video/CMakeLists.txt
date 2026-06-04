# SPDX-FileCopyrightText: 2026 AlloSphere Research Group <allosphere@ucsb.edu>
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.30)

project(
  al_video
  DESCRIPTION "Video extension for allolib"
  LANGUAGES CXX
  VERSION 0.0.1
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# TODO: add shared library option for extensions
add_library(al_video STATIC)
add_library(al::video ALIAS al_video)

target_sources(
  al_video
  PRIVATE src/al_VideoDecoder.cpp
  PUBLIC
    FILE_SET HEADERS
      BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
      BASE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/include
      FILES include/al_ext/video/al_VideoDecoder.hpp
      FILES include/al_ext/video/al_MediaBuffer.hpp
)

target_include_directories(
  al_video
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(NOT TARGET alsystem)
  message(
    FATAL_ERROR
    "alsystem target not found. al_video extension requires alsystem."
  )
endif()

target_link_libraries(al_video PRIVATE alsystem)

find_package(FFMPEG REQUIRED)

target_include_directories(al_video PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_directories(al_video PRIVATE ${FFMPEG_LIBRARY_DIRS})
target_link_libraries(al_video PRIVATE ${FFMPEG_LIBRARIES})

# message(STATUS "[al_video] FFMPEG include dirs: ${FFMPEG_INCLUDE_DIRS}")
# message(STATUS "[al_video] FFMPEG directories: ${FFMPEG_LIBRARY_DIRS}")
# message(STATUS "[al_video] FFMPEG libraries: ${FFMPEG_LIBRARIES}")

set_target_properties(
  al_video
  PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    VERIFY_INTERFACE_HEADER_SETS ON
)

#TODO: add install code here

if(AL_EXT_BUILD_EXAMPLES)
  message(STATUS "[al_video] Building examples")
  add_subdirectory(examples)
endif()

# FFMPEG
# find_path(
#   AVCODEC_INCLUDE_DIR
#   libavcodec/avcodec.h
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "include"
# )
# find_library(
#   AVCODEC_LIBRARY
#   NAMES avcodec avcodec-58
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "lib" "bin"
# )

# find_path(
#   AVFORMAT_INCLUDE_DIR
#   libavformat/avformat.h
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "include"
# )
# find_library(
#   AVFORMAT_LIBRARY
#   NAMES avformat avformat-58
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "lib" "bin"
# )

# find_path(
#   AVUTIL_INCLUDE_DIR
#   libavutil/avutil.h
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "include"
# )
# find_library(
#   AVUTIL_LIBRARY
#   avutil
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "lib" "bin"
# )

# find_path(
#   AVDEVICE_INCLUDE_DIR
#   libavdevice/avdevice.h
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "include"
# )
# find_library(
#   AVDEVICE_LIBRARY
#   avdevice
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "lib" "bin"
# )

# find_path(
#   SWSCALE_INCLUDE_DIR
#   libswscale/swscale.h
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "include"
# )
# find_library(
#   SWSCALE_LIBRARY
#   swscale
#   PATHS ${FFMPEG_ROOT}
#   PATH_SUFFIXES "lib" "bin"
# )

# if(NOT AVCODEC_LIBRARY)
#   message(
#     WARNING
#     "[al_video] Libav not found. Not building al_video extension."
#   )
# else()
#   message("[al_video] Using Libav ${AVCODEC_LIBRARY}")
#   set(THIS_EXTENSION_SRC "${CMAKE_CURRENT_LIST_DIR}/src/al_VideoDecoder.cpp")

#   set(
#     THIS_EXTENSION_HEADERS
#     "${CMAKE_CURRENT_LIST_DIR}/include/al_ext/video/al_VideoDecoder.hpp"
#   )

#   set(THIS_EXTENSION_LIBRARY_NAME al_video)

#   add_library(
#     ${THIS_EXTENSION_LIBRARY_NAME}
#     ${AL_LIBRARY_TYPE}
#     ${THIS_EXTENSION_SRC}
#     ${THIS_EXTENSION_HEADERS}
#   )

#   target_link_libraries(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC alsystem)
#   set_target_properties(
#     ${THIS_EXTENSION_LIBRARY_NAME}
#     PROPERTIES CXX_STANDARD 14
#   )
#   # foreach(CV_LIB "${OpenCV_LIBS}")
#   target_link_libraries(
#     ${THIS_EXTENSION_LIBRARY_NAME}
#     PUBLIC
#       "${AVCODEC_LIBRARY}"
#       "${AVFORMAT_LIBRARY}"
#       "${AVUTIL_LIBRARY}"
#       "${ADEVICE_LIBRARY}"
#       "${SWSCALE_LIBRARY}"
#   )
#   # endforeach()

#   # target_include_directories(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC include ${FFMPEG_INCLUDE_DIRS})
#   # target_link_directories(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC ${FFMPEG_LIBRARY_DIRS})
#   # target_link_libraries(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC ${FFMPEG_LIBRARIES})
#   target_compile_definitions(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC AL_EXT_LIBAV)

#   target_include_directories(
#     ${THIS_EXTENSION_LIBRARY_NAME}
#     PUBLIC
#       include
#       "${AVCODEC_INCLUDE_DIR}"
#       "${AVFORMAT_INCLUDE_DIR}"
#       "${AVUTIL_INCLUDE_DIR}"
#       "${AVDEVICE_INCLUDE_DIR}"
#       "${SWSCALE_INCLUDE_DIR}"
#   )

#   if(BUILD_EXAMPLES)
#     message("[al_video] Building Libav examples")
#     buildexamples("${CMAKE_CURRENT_LIST_DIR}/examples" ${THIS_EXTENSION_LIBRARY_NAME} al_video)
#   endif(BUILD_EXAMPLES)
# endif()
