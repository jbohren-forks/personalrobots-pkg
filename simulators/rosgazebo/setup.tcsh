#!/bin/tcsh
setenv SIM_TOP `rospack find gazebo`/gazebo-git

if (! $?LD_LIBRARY_PATH) setenv LD_LIBRARY_PATH ''
setenv LD_LIBRARY_PATH $SIM_TOP/root/lib:$LD_LIBRARY_PATH
setenv PATH $SIM_TOP/root/bin:$PATH

setenv GAZEBO_RESOURCE_PATH $SIM_TOP/root/share/gazebo
setenv OGRE_RESOURCE_PATH $SIM_TOP/root/lib/OGRE

echo
echo Current SIM_TOP is set to $SIM_TOP
echo Paths have been set accordingly.
echo
echo Now run gazebo [...world file...]
echo

