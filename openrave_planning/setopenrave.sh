# sets the openrave environment variables to recognize the ROS directories
# highly recommend to put these commands in the local ~..bashrc file
# otherwise, use source setopenrave.sh
export PATH=`rospack find openrave`/bin:$PATH
export OPENRAVE_DATA=`rospack find openrave_robot_description`:`rospack find ormanipulation`:`openrave-config --prefix`/share/openrave/:$OPENRAVE_DATA
export OPENRAVE_PLUGINS=`rospack find orplugins`:`openrave-config --prefix`/share/openrave/plugins:$OPENRAVE_PLUGINS
