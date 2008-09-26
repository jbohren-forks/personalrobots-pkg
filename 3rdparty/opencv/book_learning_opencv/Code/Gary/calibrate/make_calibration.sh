#!/bin/sh
gcc `pkg-config opencv --cflags --libs` calibration.cpp -o calibration

