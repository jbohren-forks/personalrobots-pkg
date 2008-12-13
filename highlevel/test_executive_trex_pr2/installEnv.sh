#!/bin/bash

# A tiny program to generate an include file for Jam

echo "# This is an auto-generated file built using rospack" > env.jam

echo "TREX_PKG_ROOT = `rospack find trex` ;" >> env.jam
echo "EXEC_HOME = `rospack find executive_trex_pr2` ;" >> env.jam
echo "TESTEXEC_HOME = `rospack find test_executive_trex_pr2` ;" >> env.jam

if [ -z "$JAVA_HOME" ]; then
    echo "# Set up JAVA_HOME" >> env.jam
    TMP_VAR=`which javac`
    echo "JAVA_HOME = `dirname $TMP_VAR`/../ ;" >> env.jam
fi

echo "" >> env.jam



