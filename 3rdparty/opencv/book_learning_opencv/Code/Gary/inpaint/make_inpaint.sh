#!/bin/sh
gcc `pkg-config opencv --cflags --libs` inpaint.cpp -o inpaint

