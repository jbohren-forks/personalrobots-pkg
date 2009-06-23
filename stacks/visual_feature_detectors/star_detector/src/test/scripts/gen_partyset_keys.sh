#!/bin/bash
OPTIONS="-thres 0 -p 75"
LETTERS="a b c d e f g h i j k l m n o p"
SIDES="L R"
DETECT=./willow_detect
SRC_DIR=party_set
OUT_DIR=data

for l in ${LETTERS}; do
    for s in ${SIDES}; do
	${DETECT} ${OPTIONS} -i ${SRC_DIR}/3${l}-${s}.bmp -o ${OUT_DIR}/ps3${l}-${s}.key
    done
done
