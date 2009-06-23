#!/bin/bash

#DETECTORS="willow willow9 sift surf"
#DETECTORS="willow willow9"
DETECTORS=willow_mod2
NUMPTS=800
INDICES=`seq 2 6`

#NUMPTS="`seq 10 10 50` `seq 100 50 500` `seq 600 100 800`"
#INDICES=2

SRC_DIR=graf
OUT_DIR=data

IMG_1=${SRC_DIR}/img1.pgm
for d in ${DETECTORS}; do
    DETECT=./${d}_detect
    for i in ${INDICES}; do
	IMG_I=${SRC_DIR}/img${i}.pgm
	XFM=${SRC_DIR}/H1to${i}p
	for p in ${NUMPTS}; do
	    INFIX=${d}-${p}p
	    OPTIONS="-thres 0 -p $p"
	    #OPTIONS="-thres 0 -line 8 -p $p"
	    KEY_1=${OUT_DIR}/img1-${i}.${INFIX}.key
	    KEY_I=${OUT_DIR}/img${i}.${INFIX}.key
	    REP=${OUT_DIR}/img1-${i}.${INFIX}.rep
	    ${DETECT} ${OPTIONS} -i ${IMG_1} -o ${KEY_1} -t ${XFM} -i2 ${IMG_I} -o2 ${KEY_I}
	    ./repeatability -plain -k1 ${KEY_1} -k2 ${KEY_I} -t ${XFM} > ${REP}
	done
    done
done
