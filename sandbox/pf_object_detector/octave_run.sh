#!/usr/bin/env bash


echo $*

D=$1
echo $D
octave -p $D/src --eval pf_detector_node