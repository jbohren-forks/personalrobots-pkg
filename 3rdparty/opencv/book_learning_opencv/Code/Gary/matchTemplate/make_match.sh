#!/bin/sh
gcc `pkg-config opencv --cflags --libs` matchTemplate.cpp -o matchTemplate

