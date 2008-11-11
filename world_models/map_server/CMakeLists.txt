cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(map_server)

rospack_add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image)

rospack_add_executable(map_server src/main.cpp)
target_link_libraries(map_server image_loader SDL SDL_image)

rospack_add_gtest(utest test/utest.cpp test/test_constants.cpp)
target_link_libraries(utest image_loader SDL SDL_image)
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

rospack_add_executable(rtest test/rtest.cpp test/test_constants.cpp)
rospack_add_gtest_build_flags(rtest)
set_target_properties(rtest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

rospack_add_rostest(test/rtest.xml)
