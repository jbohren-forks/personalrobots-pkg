#!/bin/sh
gcc `pkg-config opencv --cflags --libs` ch7hist.c -o chhist

