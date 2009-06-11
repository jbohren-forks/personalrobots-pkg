#!/bin/bash

echo Setting up dist-dcam distribution

ROS_PACKAGE_PATH=~/code/ros-pkg
DIST_DIR=ost-1.0

mkdir -p $DIST_DIR $DIST_DIR/src $DIST_DIR/include $DIST_DIR/lib $DIST_DIR/bin $DIST_DIR/src/ost $DIST_DIR/src/acquire $DIST_DIR/src/dcam $DIST_DIR/src/imwin $DIST_DIR/src/stlib $DIST_DIR/include/dcam/ $DIST_DIR/include/imwin
cp README-OST $DIST_DIR
cp include/dcam/*.h $DIST_DIR/include/dcam
cp include/imwin/*.h $DIST_DIR/include/imwin
cp src/acquire/*.cpp $DIST_DIR/src/acquire
cp src/libdcam/*.cpp $DIST_DIR/src/dcam
cp src/libimwin/*.cpp $DIST_DIR/src/imwin
cp src/ost/*.cpp $DIST_DIR/src/ost
cp src/ost/*.h $DIST_DIR/include
cp $ROS_PACKAGE_PATH/vision/stereo_image_proc/include/*.h $DIST_DIR/include
cp $ROS_PACKAGE_PATH/vision/stereo_image_proc/src/*.c $DIST_DIR/src/stlib
cp $ROS_PACKAGE_PATH/vision/stereo_image_proc/src/*.cpp $DIST_DIR/src/stlib


echo Making tar distribution file
tar -cvzf $DIST_DIR.tgz $DIST_DIR

