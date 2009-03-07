#!/bin/bash


#function scanDirectory {
#    HEADERS="`ls $1/*.h 2> /dev/null` `ls $1/*.hh 2> /dev/null`"
#    for HEADER in $HEADERS
#    do
#	cp $HEADER $INCLUDE_DIR
#    done
#    FILES=`ls -l $1 | grep "^d" | awk '{ print $8 }'`
#    for FILE in $FILES
#    do
#	scanDirectory "$1/$FILE"
#    done
#    
#}

cd `rospack find trex`

rm -r include 2> /dev/null
mkdir include
INCLUDE_DIR="`pwd`/include"
echo "Warning, this is not the real include directory. It will be deleted and recreated by make every run. Includes should be next to .cc files." > $INCLUDE_DIR/WARNING.txt
#scanDirectory `pwd`

PACKAGES="TREX PLASMA PlanWorks"


for PACKAGE in $PACKAGES; do
    cd `rospack find trex`/$PACKAGE
    mkdir $INCLUDE_DIR/$PACKAGE
    DIRS=`find \`pwd\` -type d`
    mkdir -p include
    for d in $DIRS; do
	HEADERS=`find $d -maxdepth 1 -type f -regex ".*\.hh?$"`
	if [ "$HEADERS" ]; then
	    cp -p $HEADERS $INCLUDE_DIR/$PACKAGE/
	fi
    done
    echo "Warning, this is not the real include directory. It will be deleted and recreated by make every run. Includes should be next to .cc files." > $INCLUDE_DIR/$PACKAGE/WARNING.txt
done


