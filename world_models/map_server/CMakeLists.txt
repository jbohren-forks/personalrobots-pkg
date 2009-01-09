cmake_minimum_required(VERSION 2.4.6 FATAL_ERROR)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(map_server)

rospack_add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image)

rospack_add_executable(map_server src/main.cpp)
target_link_libraries(map_server image_loader SDL SDL_image)

rospack_add_gtest(test/utest test/utest.cpp test/test_constants.cpp)
target_link_libraries(test/utest image_loader SDL SDL_image)

rospack_add_executable(test/rtest test/rtest.cpp test/test_constants.cpp)
rospack_add_gtest_build_flags(test/rtest)

rospack_add_rostest(test/rtest.xml)
