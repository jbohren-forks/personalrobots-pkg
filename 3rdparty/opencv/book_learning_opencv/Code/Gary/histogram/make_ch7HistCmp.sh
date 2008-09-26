#!/bin/sh
gcc `pkg-config opencv --cflags --libs` ch7HistCmp.cpp -o ch7HistCmp

