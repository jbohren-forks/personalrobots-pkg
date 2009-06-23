#!/bin/bash

#DETECTORS="willow willow9 sift surf"
#DETECTORS="willow willow9"
DETECTORS=willow_mod2
SCALES=`seq 0.5 0.1 2.0`
SOURCE=graf/img1.pgm
OUT_DIR=scaled
OPTIONS="-thres 0 -p 500"
#OPTIONS="-thres 0 -line 8 -p 500"

for d in ${DETECTORS}; do
    DETECT=./${d}_detect
    for s in ${SCALES}; do
	XFM=${OUT_DIR}/scaled${s}.xfm
	SCALED=${OUT_DIR}/scaled${s}.pgm
	./transform_image -i ${SOURCE} -o ${SCALED} -t ${XFM} -s ${s}
	SRC_KEY=${OUT_DIR}/src${s}.${d}.key
	SCALED_KEY=${OUT_DIR}/scaled${s}.${d}.key
	${DETECT} ${OPTIONS} -i ${SOURCE} -o ${SRC_KEY} -t ${XFM} -i2 ${SCALED} -o2 ${SCALED_KEY}
	./repeatability -plain -k1 ${SRC_KEY} -k2 ${SCALED_KEY} -t ${XFM} > ${OUT_DIR}/scaled${s}.${d}.rep -s
    done
done
