#!/bin/tcsh
setenv GAZ_TOP `rospack find gazebo`/gazebo
setenv OGRE_TOP `rospack find ogre`/ogre
setenv BOOST_TOP `rosboost-cfg --root`
setenv SIM_PLUGIN `rospack find gazebo_plugin`
setenv PR2MEDIA `rospack find pr2_ogre`:`rospack find gazebo_worlds`
setenv GAZMEDIA `rospack find gazebo`/gazebo/share/gazebo

setenv LD_LIBRARY_PATH $SIM_PLUGIN/lib:$GAZ_TOP/lib:$BOOST_TOP/lib:$LD_LIBRARY_PATH
setenv PATH $GAZ_TOP/bin:$PATH

setenv GAZEBO_RESOURCE_PATH ${PR2MEDIA}:${GAZMEDIA}
setenv OGRE_RESOURCE_PATH $OGRE_TOP/lib/OGRE

echo
echo Current GAZ_TOP is set to $GAZ_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

