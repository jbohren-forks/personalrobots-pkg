cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(resource_locator)
rospack_add_library(resource_locator src/reslocator.cpp)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

find_package( Boost COMPONENTS regex )
if( NOT ${Boost_regex_FOUND} )
    message(SEND_ERROR "could not find boost-regex")  
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(resource_locator ${Boost_LIBRARIES})
