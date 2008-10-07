cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(visual_odometry)
rospack_add_library(visual_odometry src/Cv3DPoseEstimate_Deprecated.cpp src/Cv3DPoseEstimateDispSpaceRef_Deprecated.cpp src/Cv3DPoseEstimateSSE.cpp src/Cv3DPoseEstimateStereo.cpp src/CvMat3X3.cpp src/CvMatUtils.cpp src/CvPoseEstErrMeas.cpp src/CvPoseEstErrMeasDisp.cpp src/CvRandomTripletSetGenerator.cpp src/CvStereoCamModel.cpp src/CvStereoCamParams.cpp src/CvTestTimer.cpp src/LevMarq2.cpp src/LevMarqBundleAdj.cpp src/LevMarqPartitioned.cpp src/LevMarqTransform.cpp src/LevMarqTransformDispSpace.cpp src/PathRecon.cpp src/PoseEstimate.cpp src/PoseEstimateDisp.cpp src/stereolib.cpp src/VisOdom2.cpp src/VisOdom.cpp src/KeypointDescriptors.cpp src/ost_stereolib.cpp)
# src/VisOdomBundleAdj.cpp 

rospack_add_gtest(utest test/CvTest3DPoseEstimate.cpp )
set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(utest visual_odometry)
