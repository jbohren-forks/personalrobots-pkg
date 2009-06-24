#
#   commands.bashrc
#
#   General Startup Commands
#
#   These are sub-divided into console and robot commands, to be
#   run on the console (any user station) and robot (PR or simulator)
#   respectively.
#


# Hardware dependent ROBOT commands.
if [[ ${HOSTNAME} == pr* ]]; then

    function robpower()
    {
	rosrun pr2_power_board power_node &
    }

    function robstart()
    {
	roslaunch `rospack find pr2_alpha`/${PR}.launch &
    }
else
    function robstart()
    {
	roslaunch `rospack find pr2_gazebo`/pr2_simple.launch &
    }
fi


#
#  ROBOT commands.
#
function robrosmake()
{
    rosmake pr2_power_board pr2_alpha teleop_base dcam stereo_image_proc jpeg
}

function robmanual()
{
    rosrun pr2_default_controllers tuckarm.py r &
    roslaunch `rospack find pr2_alpha`/teleop_joystick.launch &
}

function robstream()
{
    if [[${PR} == prf]]; then
	env PRCAM=${PR3} roslaunch ${HOME}/ros/ros-pkg/sandbox/teleoperation/launch/camstream.launch &
    else
	env PRCAM=${PR4} roslaunch ${HOME}/ros/ros-pkg/sandbox/teleoperation/launch/camstream.launch &
    fi	
}

function robtele()
{
    echo "KILL TUCKARM"
    roslaunch `rospack find pr2_alpha`/teleop_base_spaceball.launch &
    roslaunch ${HOME}/ros/ros-pkg/sandbox/teleoperation/launch/telearms.launch &
    robstream
}


#
#  CONSOLE commands.
#
function conrosmake()
{
    rosmake pr2_dashboard spacenav spacenav_node rviz poseviz joy jpeg image_view
}

function concore()
{  
    xterm -bg palegoldenrod -fg black -geometry 80x40+0+0 -title "ROS CORE @ ${PR2}" \
							   -e ssh    -t ${PR2} env ${PR_ENV} roscore &	       
    xterm -bg lightcyan     -fg black -geometry 100x57-0+0 -e ssh -X -t ${PR2} env PR=${PR} bash -l &
    xterm -bg rosybrown1    -fg black -geometry 100x40+0-0 -e ssh -X -t harder env PR=${PR} bash -l &
    xterm -bg darkseagreen1 -fg black -geometry 100x57-0-0 &
    sleep 10
    rosrun pr2_dashboard pr2_dashboard &
}

function conspacenavd()
{
    sudo `rospack find spacenav`/spacenavd/bin/spacenavd
}

function conviz()
{
    rosrun rviz rviz &
    # rosrun poseviz poseviz pose:=/r_arm_cartesian_pose_controller/state/pose &
    # rosrun poseviz poseviz pose:=/thumpnode/command_pose_right &
    rosrun poseviz poseviz pose:=/r_arm_cartesian_pose_controller/command &
}

function conwatch()
{
    rosrun jpeg decoder &
    rosrun image_view image_view image:=/decompressed_image &
}

function contele()
{
    rosrun joy joy __name:=foot _dev:=/dev/input/js0 _deadzone:=0 joy:=foot &
    rosrun spacenav_node spacenav_node &
    conwatch
}
