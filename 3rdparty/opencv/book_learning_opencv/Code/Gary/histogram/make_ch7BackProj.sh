#!/bin/sh
gcc `pkg-config opencv --cflags --libs` ch7BackProj.cpp -o ch7BackProj

