#!/bin/bash
export GAZ_TOP=`rospack find gazebo`/gazebo
export OGRE_TOP=`rospack find ogre`/ogre
export CG_TOP=`rospack find Cg`/Cg
export SIM_PLUGIN=`rospack find gazebo_plugin`
export PR2MEDIA=`rospack find gazebo_robot_description`/world


export LD_LIBRARY_PATH=''
export LD_LIBRARY_PATH=$SIM_PLUGIN/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$GAZ_TOP/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$OGRE_TOP/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$OGRE_TOP/lib/OGRE:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$CG_TOP/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$SIM_PLUGIN/lib:$LD_LIBRARY_PATH
export PATH=$GAZ_TOP/bin:$PATH

export GAZEBO_RESOURCE_PATH=$PR2MEDIA
export OGRE_RESOURCE_PATH=$OGRE_TOP/lib/OGRE

echo
echo Current GAZ_TOP is set to $GAZ_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

