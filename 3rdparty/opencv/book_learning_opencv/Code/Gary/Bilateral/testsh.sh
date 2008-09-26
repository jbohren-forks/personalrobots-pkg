#!/bin/sh
if [ $# > 0 ] ; then
        base=`basename $1`
        echo "compiling $base"
	echo "That would be $base.c"
else
	echo "else"
fi
