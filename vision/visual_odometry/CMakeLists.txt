cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(visual_odometry)
genmsg()

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(visual_odometry src/Cv3DPoseEstimateStereo.cpp src/CvMat3X3.cpp src/CvMatUtils.cpp src/CvPoseEstErrMeas.cpp src/CvPoseEstErrMeasDisp.cpp src/CvRandomTripletSetGenerator.cpp src/CvTestTimer.cpp src/LevMarq2.cpp src/LevMarqSparseBundleAdj.cpp src/LevMarqTransform.cpp src/LevMarqTransformDispSpace.cpp src/PathRecon.cpp src/PoseEstimate.cpp src/PoseEstimateDisp.cpp src/stereolib.cpp src/VisOdom2.cpp src/VisOdom.cpp src/KeypointDescriptors.cpp src/ost_stereolib.cpp src/PointTracks.cpp src/VOSparseBundleAdj.cpp)

rospack_add_library(votools_py src/py.cpp src/Cv3DPoseEstimateStereo.cpp src/CvMat3X3.cpp src/CvMatUtils.cpp src/CvPoseEstErrMeas.cpp src/CvPoseEstErrMeasDisp.cpp src/CvRandomTripletSetGenerator.cpp src/CvTestTimer.cpp src/LevMarq2.cpp src/LevMarqSparseBundleAdj.cpp src/LevMarqTransform.cpp src/LevMarqTransformDispSpace.cpp src/PathRecon.cpp src/PoseEstimate.cpp src/PoseEstimateDisp.cpp src/stereolib.cpp src/VisOdom2.cpp src/VisOdom.cpp src/KeypointDescriptors.cpp src/ost_stereolib.cpp src/PointTracks.cpp src/VOSparseBundleAdj.cpp src/imwin.cpp)

set_target_properties(votools_py PROPERTIES OUTPUT_NAME votools PREFIX "")
rospack_add_compile_flags(votools_py -g -O3 -Wno-missing-field-initializers -march=pentium3 -msse3)
#rospack_add_compile_flags(votools_py -O3 -DNDEBUG -Wno-missing-field-initializers )
#rospack_add_compile_flags(votools_py -O3 -DNDEBUG -Wno-missing-field-initializers -march=pentium3 -msse3)
target_link_libraries(votools_py fltk)

# a non-ros testing program
rospack_add_executable(test/testVisOdom test/CvTest3DPoseEstimate.cpp test/testVisOdom.cpp)
target_link_libraries(test/testVisOdom visual_odometry)

rospack_add_compile_flags(visual_odometry -O3 -DNDEBUG -Wno-missing-field-initializers -march=pentium3 -msse3)
#rospack_add_compile_flags(visual_odometry -g -O0 -DDEBUG=1 -Wno-missing-field-initializers -march=pentium3 -msse3)
#rospack_add_compile_flags(visual_odometry -O3 -DNDEBUG -Wno-missing-field-initializers )

# ROS unit tests
rospack_add_gtest(test/utest test/CvTest3DPoseEstimate.cpp test/utest.cpp)
target_link_libraries(test/utest visual_odometry)

rospack_add_pyunit(test/directed.py)
