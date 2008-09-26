#!/bin/sh
gcc `pkg-config opencv --cflags --libs` demhist.c -o demhist

