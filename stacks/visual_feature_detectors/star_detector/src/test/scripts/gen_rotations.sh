#!/bin/bash

ANGLES=`seq 15 15 180`
SOURCE=graf/img1.pgm
OUT_DIR=rotated

for a in ${ANGLES}; do
    ./transform_image -i ${SOURCE} -o ${OUT_DIR}/rot${a}.pgm -t ${OUT_DIR}/rot${a}.xfm -a ${a}
done
