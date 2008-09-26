#!/bin/sh
gcc `pkg-config opencv --cflags --libs` pyrMeanShift.cpp -o pyrMeanShift

