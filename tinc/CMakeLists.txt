
include("../BuildExamples.cmake")


  set(THIS_EXTENSION_SRC

      ${CMAKE_CURRENT_LIST_DIR}/src/al_DataScript.cpp
      ${CMAKE_CURRENT_LIST_DIR}/src/al_VASPReader.cpp
    )

  set(THIS_EXTENSION_HEADERS
      ${CMAKE_CURRENT_LIST_DIR}/al_DataScript.hpp
      ${CMAKE_CURRENT_LIST_DIR}/al_BufferManager.hpp
      ${CMAKE_CURRENT_LIST_DIR}/al_DiskBuffer.hpp
      ${CMAKE_CURRENT_LIST_DIR}/al_DeferredComputation.hpp
      ${CMAKE_CURRENT_LIST_DIR}/al_PeriodicTask.hpp
      ${CMAKE_CURRENT_LIST_DIR}/al_VASPReader.hpp
      ${CMAKE_CURRENT_LIST_DIR}/al_VASPRender.hpp
  )

  set(THIS_EXTENSION_LIBRARY_NAME al_tinc)


  add_library(${THIS_EXTENSION_LIBRARY_NAME} ${THIS_EXTENSION_SRC} ${THIS_EXTENSION_HEADERS})

  set_target_properties(${THIS_EXTENSION_LIBRARY_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    )

##  set(THIS_EXTENSION_LIBRARIES  ${FFTW_LIBRARY} ${GAMMA_LIBRARY})

  target_link_libraries(${THIS_EXTENSION_LIBRARY_NAME} al ${THIS_EXTENSION_LIBRARIES})
#  target_include_directories(${THIS_EXTENSION_LIBRARY_NAME} PRIVATE ${ZITACONVOLVER_DIRECTORY}  ${CMAKE_CURRENT_LIST_DIR}/fftw3)

#  # Connections to outside
##  set(CURRENT_EXTENSION_LIBRARIES ${THIS_EXTENSION_LIBRARY_NAME} ${THIS_EXTENSION_LIBRARIES})
  set(CURRENT_EXTENSION_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})

#  # unit tests

##  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/catch")
##      add_executable(convolverTests ${CMAKE_CURRENT_LIST_DIR}/unitTests/utConvolver.cpp)
##      target_link_libraries(convolverTests al ${SPATIALAUDIO_LINK_LIBRARIES} )
##      target_include_directories(convolverTests PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/external/catch")
##      add_test(NAME convolverTests
##          COMMAND $<TARGET_FILE:convolverTests> ${TEST_ARGS})
##      set_target_properties(convolverTests PROPERTIES
##          CXX_STANDARD 14
##          CXX_STANDARD_REQUIRED ON
##          )

##      #  add_memcheck_test(convolverTests)

##      add_executable(decorrelationTests ${CMAKE_CURRENT_LIST_DIR}/unitTests/utDecorrelation.cpp)
##      target_link_libraries(decorrelationTests al ${SPATIALAUDIO_LINK_LIBRARIES})
##      target_include_directories(decorrelationTests PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/external/catch")
##      add_test(NAME decorrelationTests
##          COMMAND $<TARGET_FILE:decorrelationTests> ${TEST_ARGS})
##      set_target_properties(decorrelationTests PROPERTIES
##          CXX_STANDARD 14
##          CXX_STANDARD_REQUIRED ON
##          )
##      #  add_memcheck_test(decorrelationTests)

##  endif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/catch")

  if(BUILD_EXAMPLES)
      BuildExamples("${CMAKE_CURRENT_LIST_DIR}/examples" ${THIS_EXTENSION_LIBRARY_NAME} "")
  endif(BUILD_EXAMPLES)

