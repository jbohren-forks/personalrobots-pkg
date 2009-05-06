cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(tf_conversions)

rospack_add_boost_directories()

genmsg()
gensrv()

rospack_add_library(tf_conversions
    src/tf_kdl.cpp
    src/tf_eigen.cpp
    )

rospack_add_executable(test_eigen_tf EXCLUDE_FROM_ALL test/test_eigen_tf.cpp)
rospack_declare_test(test_eigen_tf)
target_link_libraries(test_eigen_tf tf_conversions gtest)

rospack_add_rostest(test_eigen_tf.xml)
