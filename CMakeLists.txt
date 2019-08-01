cmake_minimum_required(VERSION 3.10)

project("al_ext")
message("Building al_ext")

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

list(APPEND CMAKE_MODULE_PATH
${CMAKE_CURRENT_LIST_DIR}
)

# Allolib
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_CURRENT_LIST_DIR}/build")

set(EXTENSIONS_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
subdirlist(SUBDIRS "${CMAKE_CURRENT_LIST_DIR}")

include_directories("${CMAKE_CURRENT_LIST_DIR}/../include")
include_directories("${CMAKE_CURRENT_LIST_DIR}")

add_custom_target(al_ext)

foreach(subdir ${SUBDIRS})
  if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt")
    message(" * Extension: ${subdir}")
    add_subdirectory(${subdir})
    add_dependencies(al_ext "al_${subdir}")
  endif()
endforeach()
message("** DONE Processing extensions **")

