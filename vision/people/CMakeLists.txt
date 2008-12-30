cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(people)
#genmsg()
#find_library(curl REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(people_tracking
                       src/filter/gaussian_vector.cpp 
                       src/filter/gaussian_pos_vel.cpp 
                       src/filter/mcpdf_pos_vel.cpp
                       src/filter/sysmodel_pos_vel.cpp
                       src/filter/measmodel_pos.cpp
		       src/filter/tracker_particle.cpp 
		       src/filter/tracker_kalman.cpp 
		       src/filter/people_tracking_node.cpp )

rospack_add_executable(face_detection src/face_detection.cpp src/people.cpp src/utils.cpp)
rospack_add_executable(stereo_face_color_tracker src/stereo_face_color_tracker.cpp src/people.cpp src/utils.cpp)
rospack_add_executable(track_starter_gui src/track_starter_gui.cpp src/utils.cpp)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

#rospack_add_library(pyface_detection src/py.cpp src/face_detection.cpp src/people.cpp)
#set_target_properties(pyface_detection PROPERTIES OUTPUT_NAME face_detection PREFIX "")

rospack_add_executable(leg_detector src/leg_detector.cpp src/calc_leg_features.cpp)
rospack_add_executable(train_leg_detector src/train_leg_detector.cpp src/calc_leg_features.cpp)

#rospack_add_pyunit(test/directed.py)
