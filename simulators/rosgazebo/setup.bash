#!/bin/bash
export SIM_TOP=`rospack find gazebo`/gazebo-git
export SIM_PLUGIN=`rospack find gazebo_plugin`
export PR2API=`rospack find libpr2API`
export PR2HW=`rospack find libprHW`

export LD_LIBRARY_PATH=$PR2API/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$PR2HW/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$SIM_PLUGIN/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$SIM_TOP/root/lib:$LD_LIBRARY_PATH
export PATH=$SIM_TOP/root/bin:$PATH

export GAZEBO_RESOURCE_PATH=$SIM_TOP/root/share/gazebo
export OGRE_RESOURCE_PATH=$SIM_TOP/root/lib/OGRE

echo
echo Current SIM_TOP is set to $SIM_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

