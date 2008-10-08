cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(mechanism_model)
rospack_add_library( mechanism_model
  src/simple_transmission.cpp
  src/gripper_transmission.cpp
  src/joint.cpp
  src/robot.cpp
  src/link.cpp)

rospack_add_gtest(test_FK test/test_FK.cpp)
set_target_properties(test_FK PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(test_FK mechanism_model)
