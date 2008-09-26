#!/bin/sh
gcc `pkg-config opencv --cflags --libs` linesample.cpp  -o linesample

