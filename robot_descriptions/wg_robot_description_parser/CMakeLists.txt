cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_description_parser)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

rospack_add_library(URDF src/URDF.cpp src/parser.cpp)
rospack_add_compile_flags(URDF -Wextra)

find_package( Boost COMPONENTS regex )
if( NOT ${Boost_regex_FOUND} )
    message(SEND_ERROR "could not find boost-regex")  
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(URDF ${Boost_LIBRARIES})

rospack_add_executable(parse src/parse.cpp)
target_link_libraries(parse URDF)

rospack_add_executable(merge src/merge.cpp)

# Tests
rospack_add_gtest(test_cpp_parser test/test_cpp_parser.cpp)
target_link_libraries(test_cpp_parser URDF)
