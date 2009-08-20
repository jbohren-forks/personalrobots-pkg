#! /bin/bash

XACRO=`rospack find xacro`/xacro.py
URDF2GAZEBO=`rospack find gazebo_plugin`/bin/urdf2file

if [ -z "$1" ]; then
    echo "$0 <world> <urdf>"
    exit 0
fi

WORLD=$1
URDF=$2
shift; shift

EXPANDED=`mktemp` || exit 1
MODEL=`mktemp` || exit 1
FINAL_WORLD=`mktemp` || exit 1


$XACRO $URDF > $EXPANDED
$URDF2GAZEBO $EXPANDED $MODEL


# Inserts the include definition for the robot model.  For now, we
# just use find-replace to insert the include before the world closing
# tag.

INCLUDE_DEF="\
<model:physical name=\"robot_model1\"> \\n\
<include embedded=\"true\"> \
  <xi:include href=\"${MODEL}\" /> \
</include> \\n\
</model:physical> \\n"
sed < ${WORLD} -e "s#</gazebo:world>#${INCLUDE_DEF}&#" > $FINAL_WORLD

# Runs gazebo on the resulting world/model file
. `rospack find gazebo`/setup.bash
gazebo $FINAL_WORLD $*

# Cleans up temporary files
rm -f $FINAL_WORLD $MODEL $EXPANDED
