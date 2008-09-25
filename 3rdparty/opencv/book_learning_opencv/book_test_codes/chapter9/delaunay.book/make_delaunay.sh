#!/bin/sh
gcc `pkg-config opencv --cflags --libs` delaunay.c -o delaunay

