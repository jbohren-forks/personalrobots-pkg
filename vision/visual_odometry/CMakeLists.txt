cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(visual_odometry)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

rospack_add_library(visual_odometry src/Cv3DPoseEstimateStereo.cpp src/CvMat3X3.cpp src/CvMatUtils.cpp src/CvPoseEstErrMeas.cpp src/CvPoseEstErrMeasDisp.cpp src/CvRandomTripletSetGenerator.cpp src/CvStereoCamModel.cpp src/CvStereoCamParams.cpp src/CvTestTimer.cpp src/LevMarq2.cpp src/LevMarqSparseBundleAdj.cpp src/LevMarqPartitioned.cpp src/LevMarqTransform.cpp src/LevMarqTransformDispSpace.cpp src/PathRecon.cpp src/PoseEstimate.cpp src/PoseEstimateDisp.cpp src/stereolib.cpp src/VisOdom2.cpp src/VisOdom.cpp src/KeypointDescriptors.cpp src/ost_stereolib.cpp src/VOSparseBundleAdj.cpp)

rospack_add_library(visual_odometry_py src/py.cpp src/Cv3DPoseEstimateStereo.cpp src/CvMat3X3.cpp src/CvMatUtils.cpp src/CvPoseEstErrMeas.cpp src/CvPoseEstErrMeasDisp.cpp src/CvRandomTripletSetGenerator.cpp src/CvStereoCamModel.cpp src/CvStereoCamParams.cpp src/CvTestTimer.cpp src/LevMarq2.cpp src/LevMarqSparseBundleAdj.cpp src/LevMarqPartitioned.cpp src/LevMarqTransform.cpp src/LevMarqTransformDispSpace.cpp src/PathRecon.cpp src/PoseEstimate.cpp src/PoseEstimateDisp.cpp src/stereolib.cpp src/VisOdom2.cpp src/VisOdom.cpp src/KeypointDescriptors.cpp src/ost_stereolib.cpp src/VOSparseBundleAdj.cpp)

set_target_properties(visual_odometry_py PROPERTIES OUTPUT_NAME visual_odometry PREFIX "")
#rospack_add_compile_flags(visual_odometry -O3 -DNDEBUG -Wno-missing-field-initializers -march=pentium3 -msse3)
#rospack_add_compile_flags(visual_odometry -g -O0 -Wno-missing-field-initializers -march=pentium3 -msse3)
#rospack_add_compile_flags(visual_odometry -O3 -DNDEBUG -Wno-missing-field-initializers )
rospack_add_compile_flags(visual_odometry_py -g -O0 -DDEBUG=1 -Wno-missing-field-initializers )

#rospack_add_gtest(utest test/CvTest3DPoseEstimate.cpp )
#set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#target_link_libraries(utest visual_odometry)

rospack_add_pyunit(test/directed.py)
