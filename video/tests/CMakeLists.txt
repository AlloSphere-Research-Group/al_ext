# Get and build googletest
if(TARGET gtest)

# Unit tests application
set (videotest_src
    main.cpp
    src/test_decode.cpp
)

add_executable(al_ext_video_test ${videotest_src})
set_target_properties(al_ext_video_test PROPERTIES DEBUG_POSTFIX _debug)
set_target_properties(al_ext_video_test PROPERTIES CXX_STANDARD 14)
set_target_properties(al_ext_video_test PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(al_ext_video_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set_target_properties(al_ext_video_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/bin)
set_target_properties(al_ext_video_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/bin)

target_link_libraries(al_ext_video_test PRIVATE gtest al al_video)

# Running the unit tests
add_test(NAME al_ext_video_test COMMAND al_ext_video_test)

else()
    message("Not building tests for al_ext video")
endif(TARGET gtest)
