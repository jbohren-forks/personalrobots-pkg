cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_executive_trex_pr2)


find_ros_package(executive_trex_pr2)
include(${executive_trex_pr2_PACKAGE_PATH}/NDDL.cmake)



rospack_add_nddl("wpc.0/pr2.exec.nddl")
rospack_add_nddl("wpc.1/pr2.exec.nddl")
rospack_add_nddl("wpc.2/pr2.exec.nddl")
rospack_add_nddl("wpc.3/pr2.exec.nddl")
rospack_add_nddl("wpc_gazebo.0/pr2.exec.nddl")
rospack_add_nddl("wpc_gazebo.1/pr2.exec.nddl")
rospack_add_nddl("miles.0/pr2.exec.nddl")
rospack_add_nddl("miles.2/pr2.exec.nddl")
rospack_add_nddl("miles.4/pr2.exec.nddl")
rospack_add_nddl("arm.0/pr2.exec.nddl")
rospack_add_nddl("arm.1/pr2.exec.nddl")
rospack_add_nddl("arm.2/pr2.exec.nddl")
rospack_add_nddl("nav.0/pr2.exec.nddl")


# Integrated test cases for the nav stack running with the pr2 executive and model.
rospack_add_rostest(wpc.0/test.xml) 
rospack_add_rostest(wpc.1/test.xml) 
rospack_add_rostest(wpc.2/test.xml) 
rospack_add_rostest(wpc.3/test.xml) 


#rospack_add_rostest_graphical(miles.0/test.xml)
#rospack_add_rostest_future(miles.0/test.xml) 
#rospack_add_rostest_graphical(miles.2/test.xml)
#rospack_add_rostest_future(miles.2/test.xml) 
#rospack_add_rostest_graphical(arm.0/launch.xml)
#rospack_add_rostest_future(arm.0/launch.xml)
