#!/bin/sh

# script for checking if things that depend on mpglue still build

RDEPS="mpbench sbpl_planner_node sbpl_door_planner_action"
for rdep in $RDEPS
do
    rosmake $rdep
    if [ $? -ne 0 ]; then
	echo "=================================================="
	echo "You broke $rdep"
	exit 42
    fi
done
echo "=================================================="
echo "Checked $RDEPS"
echo "Seems like nothing broke"
