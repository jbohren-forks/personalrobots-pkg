#!/bin/bash

DETECT="./willow_detect"
OPTIONS="-thres 0 -p 500"
SCALES=`seq 3 2 21`
IMG_1=graf/img1.pgm
IMG_2=graf/img2.pgm
XFM=graf/H1to2p
OUT_DIR=scale_data

for s in ${SCALES}; do
    KEY_1=${OUT_DIR}/img1-2.willow-${s}s.key
    KEY_2=${OUT_DIR}/img2.willow-${s}s.key
    ${DETECT} ${OPTIONS} -i ${IMG_1} -o ${KEY_1} -t ${XFM} -i2 ${IMG_2} -o2 ${KEY_2} -s ${s}
    ./repeatability -plain -k1 ${KEY_1} -k2 ${KEY_2} -t ${XFM} > ${OUT_DIR}/img1-2.willow-${s}s.rep
done
