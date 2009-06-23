#!/bin/bash

#DETECTORS="willow willow9 sift surf"
#DETECTORS="willow willow9"
DETECTORS=willow_mod2
ANGLES=`seq 15 15 180`
SOURCE=graf/img1.pgm
ROT_DIR=rotated
OPTIONS="-thres 0 -p 500"
#OPTIONS="-thres 0 -line 8 -p 500"

for d in ${DETECTORS}; do
    DETECT=./${d}_detect
    for a in ${ANGLES}; do
	SRC_KEY=${ROT_DIR}/src${a}.${d}.key
	ROT_KEY=${ROT_DIR}/rot${a}.${d}.key
	XFM=${ROT_DIR}/rot${a}.xfm
	${DETECT} ${OPTIONS} -i ${SOURCE} -o ${SRC_KEY} -t ${XFM} -i2 ${ROT_DIR}/rot${a}.pgm -o2 ${ROT_KEY}
	./repeatability -plain -k1 ${SRC_KEY} -k2 ${ROT_KEY} -t ${XFM} > ${ROT_DIR}/rot${a}.${d}.rep
    done
done
