#!/bin/sh
gcc `pkg-config opencv --cflags --libs` watershed.cpp -o watershed

