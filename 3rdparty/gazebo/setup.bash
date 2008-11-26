#!/bin/bash
export GAZ_TOP=`rospack find gazebo`/gazebo
export OGRE_TOP=`rospack find ogre`/ogre
export CG_TOP=`rospack find Cg`/Cg
export BOOST_TOP=`rospack find boost`/boost
export SIM_PLUGIN=`rospack find gazebo_plugin`
export PR2MEDIA=`rospack find gazebo_robot_description`/world


export LD_LIBRARY_PATH=$SIM_PLUGIN/lib:$GAZ_TOP/lib:$CG_TOP/lib:$BOOST_TOP/lib:$LD_LIBRARY_PATH
export PATH=$GAZ_TOP/bin:$PATH

export GAZEBO_RESOURCE_PATH=$PR2MEDIA
export OGRE_RESOURCE_PATH=$OGRE_TOP/lib/OGRE
export MC_RESOURCE_PATH=$PR2MEDIA

echo
echo Current GAZ_TOP is set to $GAZ_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

