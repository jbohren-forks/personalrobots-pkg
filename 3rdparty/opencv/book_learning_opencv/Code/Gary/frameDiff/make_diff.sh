#!/bin/sh
gcc `pkg-config opencv --cflags --libs` frameDiff.cpp ../background/cv_yuv_codebook.cpp -I../background -o frameDiff

