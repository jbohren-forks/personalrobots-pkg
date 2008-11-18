# sets the openrave environment variables to recognize the ROS directories
# for bash, use source setopenrave.sh
export PATH=`rospack find openrave`/bin:$PATH
export OPENRAVE_DATA=`rospack find openrave_robot_description`:`openrave-config --prefix`/share/openrave/:$OPENRAVE_DATA
export OPENRAVE_PLUGINS=`rospack find or_plugins`:`openrave-config --prefix`/share/openrave/plugins:$OPENRAVE_PLUGINS
