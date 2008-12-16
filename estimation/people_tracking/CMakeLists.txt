cmake_minimum_required(VERSION 2.4)
include(rosbuild)
rospack(people_tracking)
genmsg()
rospack_add_executable(people_tracking 
                       src/gaussian_pos_vel.cpp 
		       src/tracker.cpp )

