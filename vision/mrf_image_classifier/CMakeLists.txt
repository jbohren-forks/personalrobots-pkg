cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(mrf_image_classifier)

include_directories(src /opt/boost/include/boost-1_35)

# FIXME: build unit tests

rospack_add_library(mrf_image_classifier
  src/features/SuperpixelBlobber.cc 
  src/features/FeatureMatrix.cc 
  src/objective/BinarySubmodularObjective.cc  
  src/objective/BinarySubmodularEnergy.cc  
  src/objective/GraphCutMinimizer.cc 
  src/classifier/BinarySubmodularImageClassifier.cc 
  src/classifier/NaryImageClassifier.cc 
  src/util/Dvec.cc 
  src/util/uBLASUtils.cc)

#rospack_add_executable(binaryTrainBatch
#  src/input/binaryTrainBatch.cc)
#target_link_libraries(binaryTrainBatch mrf_image_classifier)

rospack_add_executable(naryTrainBatch
  src/input/naryTrainBatch.cc)

target_link_libraries(naryTrainBatch mrf_image_classifier)

rospack_add_executable(mrf_image_classifier_ros 
  src/ros/mrf_image_classifier.cpp)

target_link_libraries(mrf_image_classifier_ros mrf_image_classifier)

# -DNDEBUG is critical for remotely decent ublas performance
# FIXME: hack! ROS boost package broken!
set_target_properties(mrf_image_classifier
  PROPERTIES
  COMPILE_FLAGS "-O2 -DNDEBUG"
  LINK_FLAGS "-L/opt/boost/lib -lboost_thread-gcc42-mt-1_35")
