cmake_minimum_required(VERSION 3.10)

project("al_ext")
message("[al_ext] Building allolib extension modules..")

macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Allows al_ext directory to be added to searchpath for modules such as find_package()
list(APPEND CMAKE_MODULE_PATH
${CMAKE_CURRENT_LIST_DIR}
)

set(EXTENSIONS_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
subdirlist(SUBDIRS "${CMAKE_CURRENT_LIST_DIR}")

#include_directories("${CMAKE_CURRENT_LIST_DIR}/../include")
include_directories("${CMAKE_CURRENT_LIST_DIR}/..")

# remove all extensions from defualt build
set_directory_properties(PROPERTIES
  EXCLUDE_FROM_ALL ON  
  EXCLUDE_FROM_DEFAULT_BUILD ON
)

add_custom_target(al_ext)
# message("al_ext at: ${CMAKE_CURRENT_LIST_DIR}")

foreach(subdir ${SUBDIRS})
  # message("al_ext module directory at: ${subdir}")
  if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt")
    message("[al_ext] Found ${subdir}")
    add_subdirectory(${subdir})
    if (TARGET al_${subdir})
      message("[al_ext] Building Extension: ${subdir}")
      add_dependencies(al_ext al_${subdir})
      list(APPEND AL_EXT_LIBRARIES al_${subdir})
    else()
      message("[al_ext] NOT Building Extension: ${subdir}")
    endif()
  endif()
endforeach()
message("[al_ext] ** DONE Processing extensions **")

# Expose libraries to the outside
set_target_properties(al_ext PROPERTIES AL_EXT_LIBRARIES "${AL_EXT_LIBRARIES}")
