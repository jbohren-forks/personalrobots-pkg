#!/bin/tcsh
setenv GAZ_TOP `rospack find gazebo`/gazebo
setenv OGRE_TOP `rospack find ogre`/ogre
setenv CG_TOP `rospack find Cg`/Cg
setenv BOOST_TOP `rospack find boost`/boost
setenv SIM_PLUGIN `rospack find gazebo_plugin`
setenv PR2MEDIA `rospack find gazebo_robot_description`/world

setenv LD_LIBRARY_PATH $SIM_PLUGIN/lib:$GAZ_TOP/lib:$CG_TOP/lib:$BOOST_TOP/lib:$LD_LIBRARY_PATH
setenv PATH $GAZ_TOP/bin:$PATH

setenv GAZEBO_RESOURCE_PATH $PR2MEDIA
setenv OGRE_RESOURCE_PATH $OGRE_TOP/lib/OGRE
setenv MC_RESOURCE_PATH $PR2MEDIA

echo
echo Current GAZ_TOP is set to $GAZ_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

