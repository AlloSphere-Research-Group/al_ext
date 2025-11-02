cmake_minimum_required(VERSION 3.15)

list(APPEND CMAKE_MODULE_PATH
${CMAKE_CURRENT_LIST_DIR}
)

include("../BuildExamples.cmake")

project(al_ndi)

# Allow user to specify NDI SDK path
set(NDI_SDK_PATH "" CACHE PATH "Path to NDI SDK")

# Try to find NDI SDK in common locations if not specified
if(NOT NDI_SDK_PATH)
    if(WIN32)
        set(NDI_SDK_PATH "C:/Program Files/NDI/NDI 5 SDK")
    elseif(APPLE)
        set(NDI_SDK_PATH "/Library/NDI SDK for Apple")
    else() # Linux
        set(NDI_SDK_PATH "/usr/local/NDI SDK")
    endif()
endif()

# Verify NDI SDK exists
if(NOT EXISTS ${NDI_SDK_PATH})
    message(FATAL_ERROR "NDI SDK not found. Please set NDI_SDK_PATH to your NDI SDK directory")
endif()


# Platform specific library setup
if(WIN32)
    set(NDI_LIB_PATH "${NDI_SDK_PATH}/Lib/x64")
    set(NDI_LIB_NAME "Processing.NDI.Lib.x64")
elseif(APPLE)
    set(NDI_LIB_PATH "${NDI_SDK_PATH}/lib/macOS")
    set(NDI_LIB_NAME "libndi.dylib")
else() # Linux
    set(NDI_LIB_PATH "${NDI_SDK_PATH}/lib/x86_64-linux-gnu")
    set(NDI_LIB_NAME "libndi.so")
endif()

# Add library search path
link_directories(${NDI_LIB_PATH})

# Create library
add_library(al_ndi
    src/al_NDIReceiver.cpp
    src/al_NDISender.cpp
)

set_target_properties(al_ndi PROPERTIES
CXX_STANDARD 14
)

# Add NDI include directories
target_include_directories(al_ndi PUBLIC
    ${NDI_SDK_PATH}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against NDI library
if(WIN32)
    target_link_libraries(al_ndi ${NDI_LIB_NAME})
else()
    # For Unix-like systems, we need to find the actual library
    find_library(NDI_LIBRARY
        NAMES ndi libndi.so libndi.dylib
        PATHS ${NDI_LIB_PATH}
        NO_DEFAULT_PATH
    )
    
    if(NOT NDI_LIBRARY)
        message(FATAL_ERROR "NDI library not found in ${NDI_LIB_PATH}")
    endif()
    
    target_link_libraries(al_ndi algraphics ${NDI_LIBRARY})
endif()

# Installation
install(TARGETS al_ndi
    DESTINATION lib
)
install(DIRECTORY include/
    DESTINATION include
)

# Print status
message(STATUS "NDI SDK Path: ${NDI_SDK_PATH}")
message(STATUS "NDI Library Path: ${NDI_LIB_PATH}")
if(NOT WIN32)
    message(STATUS "NDI Library Found: ${NDI_LIBRARY}")
endif()