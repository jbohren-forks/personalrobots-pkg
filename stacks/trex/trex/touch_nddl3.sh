#!/bin/bash

#this hack shell script goes in and touches two files, nddl3.g and nddl3tree.g. This is done as a work around to a PLASMA build issue.
echo "Touching the NDDL3*.g files."
touch `rospack find trex`/PLASMA/src/PLASMA/NDDL/base/antlr/NDDL3.g 
touch `rospack find trex`/PLASMA/src/PLASMA/NDDL/base/antlr/NDDL3Tree.g 



