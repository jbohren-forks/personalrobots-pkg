#!/bin/sh
if [ $# > 0 ] ; then
        base=`basename $1`
        echo "compiling $base.c"
	gcc `pkg-config opencv --cflags --libs` $base.c -o $base 
else
	echo "Usage: sh make.sh basefilename"
fi
