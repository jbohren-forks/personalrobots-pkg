#!/bin/bash

for bagfile in "$@"
do

#bagfile=$1
time_per_frame="30.00"
save_to="$HOME/bags/output"
record_topics="/box_detector/observations"

mkdir -p $save_to


test -f $bagfile || echo -e "Bag file not found, exiting: '$bagfile'\n"
test -f $bagfile || exit

basename=$(echo $(basename $bagfile .bag)_ | awk '//{print substr($0,1,index($0,"_")-1)}')

tmpfile=$(tempfile)
rosplay -c $1 > $tmpfile

frames=$(cat $tmpfile | awk '/raw_stereo/{c=0} (c==1)&&/count/{print $2} //{c++}')
length_ns=$(cat $tmpfile | awk '/length/{print $2}')
topicname=$(cat $tmpfile | awk '/raw_stereo/{print substr($3,2,index($3,"raw_stereo")-3)}')
topics=$(cat $tmpfile| awk '/name:/{print $3}')
rm $tmpfile

length=$(echo "scale=5; $length_ns/1000000000" | bc)

fps=$(echo "scale=5; $frames/$length" | bc)

newbag=$save_to/$basename
rate=$(echo "scale=5; 1/($time_per_frame*$fps)" | bc)
newlength=$(echo "scale=5; $length/$rate" | bc)

launchsuffix=""
echo $bagfile | grep internhouse && launchsuffix=_internhouse > /dev/null

clear
echo "==========================================================================="
date
echo "All files: $@"
echo "Processing time: $time_per_frame sec/frame"
echo "Original bag file: $bagfile"
echo "  Frames: $frames"
echo "  Topics: $topics"
echo "  Length: $length sec"
echo "  Fps: $fps fps"
echo "  raw_stereo Topic: $topicname"
echo "Computed playback rate: $rate x"
echo "New bag file: $newbag"
echo "  New length: $newlength sec"
echo "  Launch suffix: $launchsuffix"


TERM=$(which gnome-terminal)

function start {
  echo
#  echo gnome-terminal --geometry $1 --command "bash -i -c \". $HOME/.bashrc; echo \"$2\"; read; $2; read\""
#  echo

  echo $2

#  gnome-terminal --geometry $1 --command "bash -i -c \". $HOME/.bashrc; pwd; echo $2; $2; echo Exiting..; sleep 3\""

#  $2 &> /dev/null &
 $2 &
}

start 80x6-0-0 "roscore"

sleep 3

start 80x6-0-200 "rosrun rosrecord rosrecord -f $newbag $topics $record_topics /mocap_eval/eval"

sleep 1

start 80x6-0-400 "roslaunch $HOME/ros/ros-pkg/stacks/imaging_pipeline/stereo_image_proc/narrow_stereoproc$launchsuffix.launch"

sleep 1

start 80x6-0-600 "roslaunch $HOME/ros/ros-pkg/sandbox/planar_objects/narrow_box_detector$launchsuffix.launch"

sleep 1

start 80x6-0-800 "roslaunch $HOME/ros/ros-pkg/sandbox/planar_objects/narrow_mocap_eval$launchsuffix.launch"

sleep 1

rosplay -r $rate $bagfile

sleep 1

echo
echo "killing rosrecord and roslaunch"

list="$(pidof rosrecord) $(ps aux | grep roslaunch | grep -v roscore | awk '/python/{print $2}')"
echo "kill -INT $list;"
kill -INT $list

sleep 3

echo
echo "killing roscore"

list="$(ps aux | grep roscore | awk '/python/{print $2}')"
echo "kill -INT $list;"
test -z "$list" || kill -INT $list

sleep 3

leftover="$(pidof rosrecord) $(ps aux | grep roslaunch | awk '/python/{print $2}') $(ps aux | grep roscore | awk '/python/{print $2}')"
echo "kill $leftover;"

test "$leftover" != "   " || kill $leftover

echo "Done"

done
