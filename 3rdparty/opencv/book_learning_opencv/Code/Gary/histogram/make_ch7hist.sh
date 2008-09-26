#!/bin/sh
gcc `pkg-config opencv --cflags --libs` ch7hist.cpp -o ch7hist

