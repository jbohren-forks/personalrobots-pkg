#!/bin/bash
. `rospack find gazebo`/setup.bash
gazebo $*
#gdb gazebo
