cmake_minimum_required(VERSION 2.4)
include(rosbuild)
rospack(people_tracking)
genmsg()
rospack_add_executable(people_tracking 
                       src/gaussian_vector.cpp 
                       src/gaussian_pos_vel.cpp 
                       src/mcpdf_pos_vel.cpp
                       src/sysmodel_pos_vel.cpp
                       src/measmodel_pos.cpp
		       src/tracker_particle.cpp 
		       src/tracker_kalman.cpp 
		       src/people_tracking_node.cpp )

