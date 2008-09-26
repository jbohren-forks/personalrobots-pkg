#!/bin/sh
gcc `pkg-config opencv --cflags --libs` readvideo.cpp  -o readvideo

