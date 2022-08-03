
list(APPEND CMAKE_MODULE_PATH
${CMAKE_CURRENT_LIST_DIR}
)

include("../BuildExamples.cmake")

# C:/Users/Andres/source/repos/ffmpeg-n4.4-78-g031c0cb0b4-win64-gpl-shared-4.4/
set(FFMPEG_ROOT "/alloshare/code/kenny/ffmpeg_install/" CACHE STRING "Path to FFMPEG lib and include directories")
message("Using FFMPEG at ${FFMPEG_ROOT}")

# FFMPEG
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "include")
find_library(AVCODEC_LIBRARY NAMES avcodec avcodec-58 PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "lib" "bin")

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "include")
find_library(AVFORMAT_LIBRARY NAMES avformat avformat-58 PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "lib" "bin")

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "include")
find_library(AVUTIL_LIBRARY avutil PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "lib" "bin")

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "include")
find_library(AVDEVICE_LIBRARY avdevice PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "lib" "bin")

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "include")
find_library(SWSCALE_LIBRARY swscale PATHS ${FFMPEG_ROOT} PATH_SUFFIXES "lib" "bin")


##################################################

#$ENV{GSTREAMER_1_0_ROOT_MSVC_X86_64}

# Install gstreamer from: https://gstreamer.freedesktop.org/download/
# Install both the runtime and the development installers
# On the installer install all plugins (not sure which ines are actually required here)

set(GSTREAMER_DIR "C:/gstreamer/1.0/msvc_x86_64")

if (EXISTS ${GSTREAMER_DIR}/include/gstreamer-1.0/gst/gst.h)
  set(GSTREAMER_FOUND TRUE)
  set(GSTREAMER_VERSION_STRING 1.0 CACHE STRING "")
  set(GSTREAMER_INCLUDE_DIRS
    ${GSTREAMER_DIR}/include/gstreamer-1.0/
    ${GSTREAMER_DIR}/include/glib-2.0/
    ${GSTREAMER_DIR}/lib/glib-2.0/include/
    ${GSTREAMER_DIR}/lib/gstreamer-1.0/include/
    ${GSTREAMER_DIR}/include/libxml2/
    CACHE STRING ""
  )

  file(GLOB gst_lib_files ${GSTREAMER_DIR}/lib/*.lib)
  set(GSTREAMER_LIBRARIES ${gst_lib_files} CACHE STRING "")

  mark_as_advanced(GSTREAMER_INCLUDE_DIRS GSTREAMER_LIBRARIES)

endif()





#################################
if (NOT AVCODEC_LIBRARY)
    message("Libav not found. Not building al_video extension.")
else()

    message("Using Libav ${AVCODEC_LIBRARY} SDK from: ${FFMPEG_ROOT}")
    set(THIS_EXTENSION_SRC
      "${CMAKE_CURRENT_LIST_DIR}/src/al_VideoDecoder.cpp"
      "${CMAKE_CURRENT_LIST_DIR}/src/al_VideoTexture.cpp"
      "${CMAKE_CURRENT_LIST_DIR}/src/al_VideoGstreamer.cpp"
    )

    set(THIS_EXTENSION_HEADERS
      "${CMAKE_CURRENT_LIST_DIR}/al_VideoDecoder.hpp"
      "${CMAKE_CURRENT_LIST_DIR}/al_VideoTexture.hpp"
      "${CMAKE_CURRENT_LIST_DIR}/al_VideoGstreamer.hpp"
    )

    set(THIS_EXTENSION_LIBRARY_NAME al_video)

    add_library(${THIS_EXTENSION_LIBRARY_NAME} ${AL_LIBRARY_TYPE} ${THIS_EXTENSION_SRC} ${THIS_EXTENSION_HEADERS})


    target_link_libraries(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC al)
    set_target_properties(${THIS_EXTENSION_LIBRARY_NAME} PROPERTIES
      CXX_STANDARD 14
    )
    target_link_libraries(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC  "${GSTREAMER_LIBRARIES}")
    target_include_directories(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC "${GSTREAMER_INCLUDE_DIRS}")

    target_link_libraries(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC   "${AVCODEC_LIBRARY}" "${AVFORMAT_LIBRARY}" "${AVUTIL_LIBRARY}" "${ADEVICE_LIBRARY}" "${SWSCALE_LIBRARY}")

    target_compile_definitions(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC AL_EXT_LIBAV)

    target_include_directories(${THIS_EXTENSION_LIBRARY_NAME} PUBLIC "${AVCODEC_INCLUDE_DIR}" "${AVFORMAT_INCLUDE_DIR}" "${AVUTIL_INCLUDE_DIR}" "${AVDEVICE_INCLUDE_DIR}" "${SWSCALE_INCLUDE_DIR}")


    if(BUILD_EXAMPLES)
        message("Building Libav examples")
        BuildExamples("${CMAKE_CURRENT_LIST_DIR}/examples" ${THIS_EXTENSION_LIBRARY_NAME} al_video)
    endif(BUILD_EXAMPLES)

#  # unit tests
#  add_executable(soundfileBufferedRecordTests ${CMAKE_CURRENT_LIST_DIR}/unitTests/utSoundfileBufferedRecord.cpp)
#  target_link_libraries(soundfileBufferedRecordTests al ${SPATIALAUDIO_LINK_LIBRARIES} )
#  target_include_directories(soundfileBufferedRecordTests PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/external/catch")
#  set_target_properties(soundfileBufferedRecordTests PROPERTIES
#    CXX_STANDARD 14
#    CXX_STANDARD_REQUIRED ON
#    )
#  add_test(NAME soundfileBufferedRecordTests
#    COMMAND $<TARGET_FILE:soundfileBufferedRecordTests> ${TEST_ARGS})

#  # Connections to outside
#  set(CURRENT_EXTENSION_LIBRARIES ${THIS_EXTENSION_LIBRARY_NAME} ${THIS_EXTENSION_LIBRARIES})
#  set(CURRENT_EXTENSION_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR} ${ASSIMP_INCLUDE_DIR})

#  set(CURRENT_EXTENSION_DEFINITIONS -DAL_EXT_ASSETS3D)
#  get_filename_component(ASSIMP_ROOT_DIR "${ASSIMP_LIBRARY}" DIRECTORY)
#  set(CURRENT_EXTENSION_DLLS "${ASSIMP_ROOT_DIR}/../../bin/x64/assimp-vc140-mt.dll")

  add_subdirectory(tests)

endif()
