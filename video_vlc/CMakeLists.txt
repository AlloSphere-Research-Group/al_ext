# al_ext/video/CMakeLists.txt
cmake_minimum_required(VERSION 3.15)

list(APPEND CMAKE_MODULE_PATH
${CMAKE_CURRENT_LIST_DIR}
)

include("../BuildExamples.cmake")

project(al_video_vlc)

# macOS specific VLC paths when installed via Homebrew Cask
if(APPLE)
    # Try to find VLC.app in common locations
    set(VLC_APP_PATHS
        "/Applications/VLC.app"
        "$ENV{HOME}/Applications/VLC.app"
    )

    # Find VLC.app
    set(VLC_APP_PATH "")
    foreach(path ${VLC_APP_PATHS})
        if(EXISTS ${path})
            set(VLC_APP_PATH ${path})
            break()
        endif()
    endforeach()

    if(VLC_APP_PATH)
        message(STATUS "Found VLC.app at: ${VLC_APP_PATH}")
        
        # Set VLC framework paths
        set(VLC_INCLUDE_DIR "${VLC_APP_PATH}/Contents/MacOS/include")
        set(VLC_LIBRARY_DIR "${VLC_APP_PATH}/Contents/MacOS/lib")
        
        # Find libvlc
        find_library(VLC_LIBRARY NAMES vlc libvlc
            PATHS ${VLC_LIBRARY_DIR}
            NO_DEFAULT_PATH
        )
        
        # Find libvlccore
        find_library(VLCCORE_LIBRARY NAMES vlccore libvlccore
            PATHS ${VLC_LIBRARY_DIR}
            NO_DEFAULT_PATH
        )

        if(VLC_LIBRARY AND VLCCORE_LIBRARY)
            set(VLC_FOUND TRUE)
            set(VLC_LIBRARIES ${VLC_LIBRARY} ${VLCCORE_LIBRARY})
            set(VLC_INCLUDE_DIRS ${VLC_INCLUDE_DIR})
        endif()
    else()
        message(FATAL_ERROR "Could not find VLC.app. Please install VLC using Homebrew Cask: brew install --cask vlc")
    endif()
else()
    # Non-macOS systems can use pkg-config
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(VLC REQUIRED libvlc)
endif()

# Verify VLC was found
if(NOT VLC_FOUND)
    message(FATAL_ERROR "Could not find libvlc. Please make sure VLC is installed correctly.")
endif()
# Create library
add_library(al_video_vlc
    src/al_VideoTexture.cpp
    src/al_VideoPlayer.cpp
)

set_target_properties(al_video_vlc PROPERTIES
CXX_STANDARD 14
)

target_include_directories(al_video_vlc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VLC_INCLUDE_DIRS}
)

target_link_libraries(al_video_vlc PUBLIC
    alsystem
    alapp
    ${VLC_LIBRARIES}
)

# Installation
install(TARGETS al_video_vlc
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
)