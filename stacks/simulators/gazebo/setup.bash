#!/bin/bash
export GAZ_TOP=`rospack find gazebo`/gazebo
export OGRE_TOP=`rospack find ogre`/ogre
export BOOST_TOP=`rosboost-cfg --root`
export SIM_PLUGIN=`rospack find gazebo_plugin`
export PR2MEDIA=`rospack find gazebo_worlds`
export PR2MEDIA=`rospack find pr2_ogre`:`rospack find gazebo_worlds`
export GAZMEDIA=`rospack find gazebo`/gazebo/share/gazebo


export LD_LIBRARY_PATH=$SIM_PLUGIN/lib:$GAZ_TOP/lib:$BOOST_TOP/lib:$LD_LIBRARY_PATH
export PATH=$GAZ_TOP/bin:$PATH

export GAZEBO_RESOURCE_PATH=$PR2MEDIA:$GAZMEDIA
export OGRE_RESOURCE_PATH=$OGRE_TOP/lib/OGRE

echo
echo Current GAZ_TOP is set to $GAZ_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

