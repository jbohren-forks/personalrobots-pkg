#!/bin/sh

	gcc -ggdb -I${HOME}/ros/ros-pkg/3rdparty/opencv_latest/opencv/include/opencv -L${HOME}/ros/ros-pkg/3rdparty/opencv_latest/opencv/lib  -lcxcore -lcv -lhighgui -lcvaux -lml  mark_rectangles.cpp -o mark_rectangle
