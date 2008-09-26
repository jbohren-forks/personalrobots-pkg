#!/bin/sh
gcc `pkg-config opencv --cflags --libs` birdseye.cpp -o birdseye

