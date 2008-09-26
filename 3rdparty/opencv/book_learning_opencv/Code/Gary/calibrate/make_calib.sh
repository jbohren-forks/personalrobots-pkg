#!/bin/sh
gcc `pkg-config opencv --cflags --libs` calib.cpp -o calib

