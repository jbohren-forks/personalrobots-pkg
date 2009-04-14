#!/bin/bash

echo Setting up dist-dcam distribution

ROS_PACKAGE_PATH=~/code/ros-pkg
mkdir dist-dcam dist-dcam/src dist-dcam/include dist-dcam/lib dist-dcam/bin dist-dcam/src/ost dist-dcam/src/acquire dist-dcam/src/dcam dist-dcam/src/imwin dist-dcam/src/stlib dist-dcam/include/dcam/ dist-dcam/include/imwin
cp include/dcam/*.h dist-dcam/include/dcam
cp include/imwin/*.h dist-dcam/include/imwin
cp src/acquire/*.cpp dist-dcam/src/acquire
cp src/libdcam/*.cpp dist-dcam/src/dcam
cp src/libimwin/*.cpp dist-dcam/src/imwin
cp src/ost/*.cpp dist-dcam/src/ost
cp src/ost/*.h dist-dcam/include
cp $ROS_PACKAGE_PATH/vision/stereo_image_proc/include/*.h dist-dcam/include
cp $ROS_PACKAGE_PATH/vision/stereo_image_proc/src/*.c dist-dcam/src/stlib
cp $ROS_PACKAGE_PATH/vision/stereo_image_proc/src/*.cpp dist-dcam/src/stlib

