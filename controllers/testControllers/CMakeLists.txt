cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(testControllers)
add_definitions(-Wall)

rospack_add_executable(testControllers src/test_base.cpp)
target_link_libraries(testControllers etherdrive_hardware pr2Controllers)
rospack_add_executable(testURDF src/test_load_xml.cpp)

#set(my_libraries etherdrive_hardware pr2Controllers rosControllers)
#rospack_add_executable(testControllers src/test_ros.cpp)
#rospack_add_executable(testRosListener src/test_ros_get.cpp)
#target_link_libraries(testControllers ${my_libraries})
#target_link_libraries(testRosListener ${my_libraries})
