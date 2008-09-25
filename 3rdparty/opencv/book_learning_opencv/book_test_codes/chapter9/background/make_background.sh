#!/bin/sh
gcc `pkg-config opencv --cflags --libs` AvgBackground.cpp cv_yuv_codebook.cpp backgroundAVG.cpp -o background

